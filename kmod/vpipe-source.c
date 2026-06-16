// SPDX-License-Identifier: MIT

#define pr_fmt(fmt) "vpipe-source: " fmt

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include "vpipe-internal.h"
#include "vpipe-source.h"

static const struct vpipe_source_ops *vpipe_backend_ops[VPIPE_BACKEND_MAX];
static const char *vpipe_backend_name[VPIPE_BACKEND_MAX];

static bool vpipe_backend_id_valid(enum vpipe_backend_id id)
{

    return id > VPIPE_BACKEND_NONE && id < VPIPE_BACKEND_MAX;
}

int vpipe_source_register(enum vpipe_backend_id id,
                          const struct vpipe_source_ops *ops,
                          const char *name)
{
    if (!ops || !ops->next_frame)
        return -EINVAL;
    if (!vpipe_backend_id_valid(id))
        return -EINVAL;
    if (vpipe_backend_ops[id])
        return -EBUSY;

    vpipe_backend_ops[id]  = ops;
    vpipe_backend_name[id] = name ? name : "unnamed";

    pr_info("registered backend %u (%s)\n", id, vpipe_backend_name[id]);
    return 0;
}

struct vpipe_source *vpipe_source_create(enum vpipe_backend_id id)
{
    struct vpipe_source *src;

    if (!vpipe_backend_id_valid(id))
        return ERR_PTR(-EINVAL);
    if (!vpipe_backend_ops[id])
        return ERR_PTR(-ENODEV);

    src = kzalloc(sizeof(*src), GFP_KERNEL);
    if (!src)
        return ERR_PTR(-ENOMEM);

    src->backend_id = id;
    src->ops        = vpipe_backend_ops[id];
    src->name       = vpipe_backend_name[id];
    src->seq        = 0;
    src->priv       = NULL;

    return src;
}

void vpipe_source_destroy(struct vpipe_source *src)
{
    if (!src)
        return;
    if (src->ops && src->ops->stop)
        src->ops->stop(src);
    kfree(src);
}

int vpipe_source_start(struct vpipe_source *src, struct vpipe_ctx *ctx)
{
    if (!src || !src->ops)
        return -EINVAL;
    if (!src->ops->start)
        return 0;
    return src->ops->start(src, ctx);
}

void vpipe_source_stop(struct vpipe_source *src)
{
    if (!src || !src->ops)
        return;
    if (src->ops->stop)
        src->ops->stop(src);
}

int vpipe_source_reset(struct vpipe_source *src)
{
    if (!src || !src->ops)
        return -EINVAL;
    src->seq = 0;
    if (!src->ops->reset)
        return 0;
    return src->ops->reset(src);
}

int vpipe_source_next_frame(struct vpipe_source *src,
                            struct vpipe_buffer *buf,
                            struct vpipe_frame_desc *desc)
{
    u32 width, height, stride;
    int ret;

    if (!src || !src->ops || !src->ops->next_frame || !buf || !desc)
        return -EINVAL;

    /* Preserve geometry hints across the descriptor reset so backends
     * see consistent dimensions regardless of memset ordering.
     */
    width = desc->width;
    height = desc->height;
    stride = desc->stride;

    memset(desc, 0, sizeof(*desc));

    desc->width = width;
    desc->height = height;
    desc->stride = stride;

    ret = src->ops->next_frame(src, buf, desc);
    if (ret)
        return ret;

    if (!desc->source_timestamp_ns)
        desc->source_timestamp_ns = ktime_get_ns();

    desc->backend_id = src->backend_id;
    desc->sequence = src->seq++;
    return 0;
}

int vpipe_source_export_dmabuf(struct vpipe_source *src,
                               struct vpipe_buffer *buf)
{
    /* buf may be NULL: source-side exporters (vcam-derived) export the
     * backend's own producer surface rather than a specific m2m buffer.
     */
    if (!src || !src->ops)
        return -EINVAL;
    if (!src->ops->export_dmabuf)
        return -EOPNOTSUPP;
    return src->ops->export_dmabuf(src, buf);
}