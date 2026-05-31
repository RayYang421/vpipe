// SPDX-License-Identifier: MIT
#ifndef __VPIPE_SOURCE_H__
#define __VPIPE_SOURCE_H__

#include <linux/types.h>
#include <linux/ktime.h>


struct vpipe_source;
struct vpipe_ctx;
struct vpipe_buffer;

enum vpipe_backend_id {
    VPIPE_BACKEND_NONE = 0,
    VPIPE_BACKEND_FIXTURE = 1,
    VPIPE_BACKEND_SYNTHETIC = 2,
    VPIPE_BACKEND_VCAM_DERIVED = 3,
    VPIPE_BACKEND_MAX,
};

struct vpipe_frame_desc {
    u64 source_timestamp_ns;
    u32 sequence;
    u32 backend_id;
    u32 byteused;
    u32 flags;
};

struct vpipe_source_ops {
    int (*start)(struct vpipe_source *src, struct vpipe_ctx *ctx);
    void (*stop)(struct vpipe_source *src);
    int (*reset)(struct vpipe_source *src);

    int (*next_frame)(struct vpipe_source *src, struct vpipe_buffer *buf, struct vpipe_frame_desc *desc);
};

struct vpipe_source {
    enum vpipe_backend_id backend_id;
    const struct vpipe_source_ops *ops;
    void *priv;
    u32 seq;
    const char *name;
};

struct vpipe_source *vpipe_source_create(enum vpipe_backend_id id);

void vpipe_source_destroy(struct vpipe_source *src);

int vpipe_source_start(struct vpipe_source *src, struct vpipe_ctx *ctx);
void vpipe_source_stop(struct vpipe_source *src);
int vpipe_source_reset(struct vpipe_source *src);

int vpipe_source_next_frame(struct vpipe_source *src, struct vpipe_buffer *buf, struct vpipe_frame_desc *desc);

int vpipe_source_register(enum vpipe_backend_id id, const struct vpipe_source_ops *ops, const char *name);

#endif /* __VPIPE_SOURCE_H__ */