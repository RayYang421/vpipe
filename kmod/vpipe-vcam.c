// SPDX-License-Identifier: MIT

#define pr_fmt(fmt) "vpipe-vcam: " fmt

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/iosys-map.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/vmalloc.h>

#include <media/videobuf2-v4l2.h>

#include "vpipe-internal.h"
#include "vpipe-source.h"


static unsigned int vcam_fps = 30;
module_param(vcam_fps, uint, 0644);
MODULE_PARM_DESC(vcam_fps,
                 "VCAM-derived backend: simulated capture FPS for the "
                 "source-side timestamp cadence (0 disables advancing)");

static unsigned int vcam_drop_every;
module_param(vcam_drop_every, uint, 0644);
MODULE_PARM_DESC(vcam_drop_every,
                 "VCAM-derived backend: drop every N-th frame (0 disables)");


struct vcam_fb {
    struct kref refcount;
    void *vaddr;
    unsigned long size;
};

struct vpipe_vcam_priv {
    struct vcam_fb *fb;
    u32 pattern_offset;
    u32 frames_produced;
    u32 drop_every_n;
    u64 base_ts_ns;
    u64 frame_interval_ns;
};

static void vcam_fb_release(struct kref *kref)
{
    struct vcam_fb *fb = container_of(kref, struct vcam_fb, refcount);

    vfree(fb->vaddr);
    kfree(fb);
}

static struct vcam_fb *vcam_fb_alloc(unsigned long bytes)
{
    struct vcam_fb *fb;

    fb = kzalloc(sizeof(*fb), GFP_KERNEL);
    if (!fb)
        return NULL;

    fb->size = PAGE_ALIGN(bytes);
    fb->vaddr = vmalloc_user(fb->size);
    if (!fb->vaddr) {
        kfree(fb);
        return NULL;
    }

    kref_init(&fb->refcount);
    return fb;
}

static void vcam_fill_gradient(u8 *dst, u32 width, u32 height, u32 stride,
                               u32 offset)
{
    u32 x, y;

    for (y = 0; y < height; y++) {
        u8 *row = dst + y * stride;

        for (x = 0; x < width; x++)
            row[x] = (u8) ((x + y + offset) & 0xFF);
    }

    dst[0] = (u8) (offset & 0xFF);
}


static struct sg_table *vcam_dmabuf_map(struct dma_buf_attachment *att,
                                        enum dma_data_direction dir)
{
    struct vcam_fb *fb = att->dmabuf->priv;
    unsigned int npages = fb->size >> PAGE_SHIFT;
    struct scatterlist *sg;
    struct sg_table *sgt;
    unsigned int i;
    int ret;

    sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
    if (!sgt)
        return ERR_PTR(-ENOMEM);

    ret = sg_alloc_table(sgt, npages, GFP_KERNEL);
    if (ret) {
        kfree(sgt);
        return ERR_PTR(ret);
    }

    sg = sgt->sgl;
    for (i = 0; i < npages; i++) {
        struct page *page =
            vmalloc_to_page(fb->vaddr + ((size_t) i << PAGE_SHIFT));

        if (!page) {
            sg_free_table(sgt);
            kfree(sgt);
            return ERR_PTR(-EFAULT);
        }
        sg_set_page(sg, page, PAGE_SIZE, 0);
        sg = sg_next(sg);
    }

    ret = dma_map_sgtable(att->dev, sgt, dir, 0);
    if (ret) {
        sg_free_table(sgt);
        kfree(sgt);
        return ERR_PTR(ret);
    }

    return sgt;
}

static void vcam_dmabuf_unmap(struct dma_buf_attachment *att,
                              struct sg_table *sgt,
                              enum dma_data_direction dir)
{
    dma_unmap_sgtable(att->dev, sgt, dir, 0);
    sg_free_table(sgt);
    kfree(sgt);
}


static int vcam_dmabuf_begin_cpu(struct dma_buf *dmabuf,
                                 enum dma_data_direction dir)
{
    return 0;
}

static int vcam_dmabuf_end_cpu(struct dma_buf *dmabuf,
                               enum dma_data_direction dir)
{
    return 0;
}

static int vcam_dmabuf_mmap(struct dma_buf *dmabuf,
                            struct vm_area_struct *vma)
{
    struct vcam_fb *fb = dmabuf->priv;

    if (vma->vm_end - vma->vm_start > fb->size)
        return -EINVAL;

    return remap_vmalloc_range(vma, fb->vaddr, vma->vm_pgoff);
}

static int vcam_dmabuf_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
    struct vcam_fb *fb = dmabuf->priv;

    iosys_map_set_vaddr(map, fb->vaddr);
    return 0;
}

static void vcam_dmabuf_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
    iosys_map_clear(map);
}

static void vcam_dmabuf_release(struct dma_buf *dmabuf)
{
    struct vcam_fb *fb = dmabuf->priv;

    kref_put(&fb->refcount, vcam_fb_release);
}

static const struct dma_buf_ops vcam_dmabuf_ops = {
    .map_dma_buf = vcam_dmabuf_map,
    .unmap_dma_buf = vcam_dmabuf_unmap,
    .begin_cpu_access = vcam_dmabuf_begin_cpu,
    .end_cpu_access = vcam_dmabuf_end_cpu,
    .mmap = vcam_dmabuf_mmap,
    .vmap = vcam_dmabuf_vmap,
    .vunmap = vcam_dmabuf_vunmap,
    .release = vcam_dmabuf_release,
};


static int vcam_start(struct vpipe_source *src, struct vpipe_ctx *ctx)
{
    struct vpipe_vcam_priv *priv;

    if (!ctx || ctx->out_q.sizeimage == 0)
        return -EINVAL;

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    priv->fb = vcam_fb_alloc(ctx->out_q.sizeimage);
    if (!priv->fb) {
        kfree(priv);
        return -ENOMEM;
    }

    priv->pattern_offset = 0;
    priv->frames_produced = 0;
    priv->drop_every_n = vcam_drop_every;
    priv->base_ts_ns = ktime_get_ns();
    priv->frame_interval_ns =
        vcam_fps ? div_u64(NSEC_PER_SEC, vcam_fps) : 0;

    src->priv = priv;
    pr_info("started: fps=%u drop_every_n=%u sizeimage=%u\n", vcam_fps,
            priv->drop_every_n, ctx->out_q.sizeimage);
    return 0;
}

static void vcam_stop(struct vpipe_source *src)
{
    struct vpipe_vcam_priv *priv = src->priv;

    if (!priv)
        return;

    if (priv->fb)
        kref_put(&priv->fb->refcount, vcam_fb_release);
    kfree(priv);
    src->priv = NULL;
    pr_info("stopped\n");
}

static int vcam_reset(struct vpipe_source *src)
{
    struct vpipe_vcam_priv *priv = src->priv;

    if (priv) {
        priv->pattern_offset = 0;
        priv->frames_produced = 0;
        priv->base_ts_ns = ktime_get_ns();
    }
    return 0;
}

static int vcam_next_frame(struct vpipe_source *src,
                           struct vpipe_buffer *buf,
                           struct vpipe_frame_desc *desc)
{
    struct vpipe_vcam_priv *priv = src->priv;
    size_t frame_bytes;
    void *dst;

    if (!priv || !priv->fb)
        return -EINVAL;

    if (desc->width == 0 || desc->height == 0 || desc->stride == 0)
        return -EINVAL;

    frame_bytes = (size_t) desc->stride * desc->height;
    if (frame_bytes > priv->fb->size)
        return -EINVAL;

    dst = vb2_plane_vaddr(&buf->m2m_buf.vb.vb2_buf, 0);
    if (!dst)
        return -EFAULT;

    desc->source_timestamp_ns =
        priv->base_ts_ns + priv->frames_produced * priv->frame_interval_ns;

    if (priv->drop_every_n &&
        (priv->frames_produced % priv->drop_every_n) == 0) {
        desc->flags |= VPIPE_FRAME_FLAG_DROPPED;
        priv->frames_produced++;
        return 0;
    }


    vcam_fill_gradient(priv->fb->vaddr, desc->width, desc->height,
                       desc->stride, priv->pattern_offset);
    if (dst != priv->fb->vaddr)
        memcpy(dst, priv->fb->vaddr, frame_bytes);

    desc->byteused = (u32) frame_bytes;
    priv->pattern_offset++;
    priv->frames_produced++;
    return 0;
}

static int vcam_export_dmabuf(struct vpipe_source *src,
                              struct vpipe_buffer *buf)
{
    DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
    struct vpipe_vcam_priv *priv = src->priv;
    struct dma_buf *dmabuf;
    int fd;


    if (!priv || !priv->fb)
        return -EINVAL;


    kref_get(&priv->fb->refcount);

    exp_info.ops = &vcam_dmabuf_ops;
    exp_info.size = priv->fb->size;
    exp_info.flags = O_RDWR | O_CLOEXEC;
    exp_info.priv = priv->fb;

    dmabuf = dma_buf_export(&exp_info);
    if (IS_ERR(dmabuf)) {
        kref_put(&priv->fb->refcount, vcam_fb_release);
        return PTR_ERR(dmabuf);
    }

    fd = dma_buf_fd(dmabuf, O_CLOEXEC);
    if (fd < 0)
        dma_buf_put(dmabuf);

    return fd;
}

static const struct vpipe_source_ops vcam_ops = {
    .start = vcam_start,
    .stop = vcam_stop,
    .reset = vcam_reset,
    .next_frame = vcam_next_frame,
    .export_dmabuf = vcam_export_dmabuf,
};

int vpipe_source_vcam_register(void)
{
    return vpipe_source_register(VPIPE_BACKEND_VCAM_DERIVED, &vcam_ops,
                                 "vcam-derived");
}

MODULE_IMPORT_NS("DMA_BUF");