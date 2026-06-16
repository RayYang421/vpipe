// SPDX-License-Identifier: MIT
/*
 * vpipe VCAM-derived source backend
 *
 * Models the camera-like producer behaviour distilled from vcam without
 * turning vpipe into a full camera driver. The backend keeps an internal
 * "framebuffer" (the vcam-style injection surface), mutates it
 * deterministically per frame, and copies it into the OUTPUT vb2 buffer
 * when the m2m layer calls next_frame(). That copy is the source-side
 * injection cost the Task 4 benchmark attributes.
 *
 * Properties this backend provides (Task 3):
 *
 *   - deterministic frame production: the framebuffer content is a pure
 *     function of (geometry, frame index), so a given frame index always
 *     yields the same bytes and CRC.
 *   - fixed FPS mode: source_timestamp_ns advances by exactly one
 *     1/fps interval per produced frame, modelling a hardware capture
 *     clock. Real pacing of QBUF is still the userspace runner's job, so
 *     no in-kernel timer or kthread is introduced (matches the vpipe
 *     integration rule of "userspace controls fixed-FPS rhythm").
 *   - fixed frame sequence: the framework stamps desc->sequence from
 *     src->seq; this backend tracks its own frames_produced in lockstep.
 *   - synthetic framebuffer mutation: a moving gradient plus a per-frame
 *     marker byte advances each frame.
 *   - optional dropped-frame simulation: vcam_drop_every drops every
 *     N-th frame (flagged, not filled), mirroring the SYNTHETIC backend.
 *   - source-side timestamp: provided by the backend, not the framework.
 *   - source-side DMA-BUF export: the internal framebuffer is exported
 *     as a dma-buf fd (export_dmabuf), letting a downstream importer map
 *     the same producer pages. The backing storage is kref-counted so the
 *     exported dma-buf safely outlives STREAMOFF / backend teardown.
 */

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

/* Simulated capture rate. Drives the source-side timestamp cadence only;
 * actual QBUF pacing stays in userspace. Read at start() time.
 */
static unsigned int vcam_fps = 30;
module_param(vcam_fps, uint, 0644);
MODULE_PARM_DESC(vcam_fps,
                 "VCAM-derived backend: simulated capture FPS for the "
                 "source-side timestamp cadence (0 disables advancing)");

/* Drop every N-th frame (0 disables). Read at start() time. */
static unsigned int vcam_drop_every;
module_param(vcam_drop_every, uint, 0644);
MODULE_PARM_DESC(vcam_drop_every,
                 "VCAM-derived backend: drop every N-th frame (0 disables)");

/*
 * Refcounted backing store for the producer framebuffer.
 *
 * Held by the backend (one ref from start()) and by every exported
 * dma-buf (one ref each). Freed when the last ref drops, so a userspace
 * consumer holding an exported fd past STREAMOFF never dereferences freed
 * memory. Allocated with vmalloc_user() so the dma-buf .mmap path can hand
 * the same pages to userspace via remap_vmalloc_range().
 */
struct vcam_fb {
    struct kref refcount;
    void *vaddr;          /* vmalloc_user area, page-multiple */
    unsigned long size;   /* page-aligned byte length */
};

struct vpipe_vcam_priv {
    struct vcam_fb *fb;     /* producer surface, shared with dma-buf exports */
    u32 pattern_offset;     /* mutation phase, advanced per produced frame */
    u32 frames_produced;    /* total next_frame() invocations */
    u32 drop_every_n;       /* snapshot of vcam_drop_every */
    u64 base_ts_ns;         /* synthetic capture-clock anchor */
    u64 frame_interval_ns;  /* 1/fps in ns, 0 when fps == 0 */
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

/*
 * Deterministic mutation: moving gradient with a per-frame marker in the
 * first byte. Output depends only on (width, height, stride, offset), so a
 * frame index is reproducible across runs for CRC validation.
 */
static void vcam_fill_gradient(u8 *dst, u32 width, u32 height, u32 stride,
                               u32 offset)
{
    u32 x, y;

    for (y = 0; y < height; y++) {
        u8 *row = dst + y * stride;

        for (x = 0; x < width; x++)
            row[x] = (u8) ((x + y + offset) & 0xFF);
    }

    /* Marker so consecutive frames differ even at offset wrap-around. */
    dst[0] = (u8) (offset & 0xFF);
}

/* ---- dma-buf exporter over the producer framebuffer -------------------- */

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

/*
 * CPU-access fences. No-ops here because the producer surface is a plain
 * CPU mapping with no device-private caches, but kept as explicit hook
 * points: this is where a real hardware-backed source would invalidate or
 * flush caches, and where the Task 4 benchmark attributes synchronization
 * overhead and buffer-ownership transfer cost.
 */
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

/* ---- backend ops ------------------------------------------------------- */

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

    /* Source-side timestamp from the synthetic capture clock. Set before
     * any early return so dropped frames still carry a cadence-correct
     * timestamp and the framework does not overwrite it with ktime.
     */
    desc->source_timestamp_ns =
        priv->base_ts_ns + priv->frames_produced * priv->frame_interval_ns;

    /* Dropped-frame simulation: flag and skip the fill/copy. The framework
     * still stamps backend_id, sequence, and the timestamp above, so
     * downstream sees a flagged frame rather than a sequence gap.
     */
    if (priv->drop_every_n &&
        (priv->frames_produced % priv->drop_every_n) == 0) {
        desc->flags |= VPIPE_FRAME_FLAG_DROPPED;
        priv->frames_produced++;
        return 0;
    }

    /* Mutate the producer framebuffer, then inject it into the OUTPUT
     * buffer. The memcpy is the source-side injection cost Task 4 measures
     * against the DMA-BUF export path.
     */
    vcam_fill_gradient(priv->fb->vaddr, desc->width, desc->height,
                       desc->stride, priv->pattern_offset);
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

    /* Export requires an active producer surface (allocated at start()). */
    if (!priv || !priv->fb)
        return -EINVAL;

    /* Hand the exported dma-buf its own reference on the backing store. */
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
        dma_buf_put(dmabuf); /* drops the export ref via .release */

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