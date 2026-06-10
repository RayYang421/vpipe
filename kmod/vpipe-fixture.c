// SPDX-License-Identifier: MIT

#define pr_fmt(fmt) "vpipe-fixture: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/printk.h>

#include "vpipe-internal.h"
#include "vpipe-source.h"

static int fixture_next_frame(struct vpipe_source *src,
                              struct vpipe_buffer *buf,
                              struct vpipe_frame_desc *desc)
{
    return 0;
}

static const struct vpipe_source_ops fixture_ops = {
    .next_frame = fixture_next_frame,
};

int vpipe_source_fixture_register(void)
{
    return vpipe_source_register(VPIPE_BACKEND_FIXTURE,
                                 &fixture_ops, "fixture");
}
