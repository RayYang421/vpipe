// SPDX-License-Identifier: MIT

#define pr_fmt(fmt) "vpipe-synthetic: " fmt

#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <media/videobuf2-v4l2.h>

#include "vpipe-internal.h"
#include "vpipe-source.h"

static unsigned int synthetic_drop_every;
module_param(synthetic_drop_every, uint, 0644);
MODULE_PARM_DESC(synthetic_drop_every,
                 "SYNTHETIC backend: drop every N-th frame (0 disables)");

struct vpipe_synthetic_priv {
    u32 pattern_offset;
    u32 frames_produced;
    u32 drop_every_n;
};


static void synthetic_fill_gradient(u8 *dst, u32 width, u32 height,
                                    u32 stride, u32 offset)
{
    u32 x, y;

    for (y = 0; y < height; y++) {
        u8 *row = dst + y * stride;

        for (x = 0; x < width; x++)
            row[x] = (u8) ((x + y + offset) & 0xFF);
    }
}

static int synthetic_start(struct vpipe_source *src, struct vpipe_ctx *ctx)
{
    struct vpipe_synthetic_priv *priv;

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    priv->pattern_offset = 0;
    priv->frames_produced = 0;
    priv->drop_every_n = synthetic_drop_every;

    src->priv = priv;
    pr_info("started: drop_every_n=%u\n", priv->drop_every_n);
    return 0;
}

static void synthetic_stop(struct vpipe_source *src)
{
    kfree(src->priv);
    src->priv = NULL;
    pr_info("stopped\n");
}

static int synthetic_reset(struct vpipe_source *src)
{
    struct vpipe_synthetic_priv *priv = src->priv;

    if (priv) {
        priv->pattern_offset = 0;
        priv->frames_produced = 0;
    }
    return 0;
}

static int synthetic_next_frame(struct vpipe_source *src,
                                struct vpipe_buffer *buf,
                                struct vpipe_frame_desc *desc)
{
    struct vpipe_synthetic_priv *priv = src->priv;
    void *vaddr;

    if (!priv)
        return -EINVAL;

    if (desc->width == 0 || desc->height == 0 || desc->stride == 0)
        return -EINVAL;

    vaddr = vb2_plane_vaddr(&buf->m2m_buf.vb.vb2_buf, 0);
    if (!vaddr)
        return -EFAULT;

    priv->frames_produced++;

    if (priv->drop_every_n &&
        (priv->frames_produced % priv->drop_every_n) == 0) {
        desc->flags |= VPIPE_FRAME_FLAG_DROPPED;
        return 0;
    }

    synthetic_fill_gradient(vaddr, desc->width, desc->height,
                            desc->stride, priv->pattern_offset);

    priv->pattern_offset++;

    return 0;
}

static const struct vpipe_source_ops synthetic_ops = {
    .start = synthetic_start,
    .stop = synthetic_stop,
    .reset = synthetic_reset,
    .next_frame = synthetic_next_frame,
};

int vpipe_source_synthetic_register(void)
{
    return vpipe_source_register(VPIPE_BACKEND_SYNTHETIC, &synthetic_ops,
                                 "synthetic");
}
