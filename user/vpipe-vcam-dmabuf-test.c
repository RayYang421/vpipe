/* SPDX-License-Identifier: MIT */

#define _POSIX_C_SOURCE 200809L

#include "vpipe-common.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define BUFFER_COUNT 4
#define WIDTH 640
#define HEIGHT 480

#define VPIPE_BACKEND_VCAM_DERIVED 3

static int set_ctrl(int fd, uint32_t id, int32_t value)
{
    struct v4l2_control ctrl;

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = id;
    ctrl.value = value;
    return vpipe_xioctl(fd, VIDIOC_S_CTRL, &ctrl);
}

static int parse_uint_range(const char *s,
                            unsigned long min,
                            unsigned long max,
                            unsigned long *out)
{
    char *end;
    unsigned long v;

    if (!s || *s == '\0' || *s == '-')
        return -1;
    errno = 0;
    v = strtoul(s, &end, 10);
    if (errno || *end != '\0' || v < min || v > max)
        return -1;
    *out = v;
    return 0;
}

static unsigned long verify_gradient(const uint8_t *pix,
                                     unsigned int width,
                                     unsigned int height,
                                     unsigned int stride)
{
    uint8_t offset = pix[0];
    unsigned long mismatches = 0;
    unsigned int x, y;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            uint8_t expected = (uint8_t) ((x + y + offset) & 0xFF);

            if (pix[y * stride + x] != expected)
                mismatches++;
        }
    }
    return mismatches;
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/video0";
    unsigned long frames = 16;
    struct vpipe_mmap_buffer out_bufs[BUFFER_COUNT] = {0};
    struct vpipe_mmap_buffer cap_bufs[BUFFER_COUNT] = {0};
    struct v4l2_format fmt;
    uint32_t backend = VPIPE_BACKEND_VCAM_DERIVED;
    bool out_streaming = false;
    bool cap_streaming = false;
    size_t sizeimage;
    unsigned int stride;
    void *map = MAP_FAILED;
    __s32 dbuf_fd = -1;
    int fd = -1;
    int rc = 1;
    unsigned int i;

    for (i = 0; i < BUFFER_COUNT; i++) {
        out_bufs[i].dmabuf_fd = -1;
        cap_bufs[i].dmabuf_fd = -1;
    }

    if (argc > 2 && parse_uint_range(argv[2], 1, UINT_MAX, &frames) < 0) {
        fprintf(stderr, "invalid frames: %s\n", argv[2]);
        return 1;
    }

    fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open vpipe");
        return 1;
    }

    if (vpipe_xioctl(fd, VPIPE_IOC_SET_BACKEND, &backend) < 0) {
        perror("VPIPE_IOC_SET_BACKEND vcam-derived");
        goto cleanup;
    }

    if (vpipe_set_format(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT, WIDTH, HEIGHT,
                         V4L2_PIX_FMT_GREY) < 0 ||
        vpipe_set_format(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, WIDTH, HEIGHT,
                         V4L2_PIX_FMT_GREY) < 0) {
        perror("VIDIOC_S_FMT");
        goto cleanup;
    }


    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (vpipe_xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("VIDIOC_G_FMT");
        goto cleanup;
    }
    stride = fmt.fmt.pix.bytesperline;
    sizeimage = fmt.fmt.pix.sizeimage;
    if (stride == 0 || sizeimage == 0) {
        fprintf(stderr, "G_FMT returned zero geometry\n");
        goto cleanup;
    }

    if (vpipe_request_mmap_buffers(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                                   BUFFER_COUNT, out_bufs) < 0 ||
        vpipe_request_mmap_buffers(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                   BUFFER_COUNT, cap_bufs) < 0) {
        perror("REQBUFS/mmap");
        goto cleanup;
    }

    if (set_ctrl(fd, VPIPE_CID_ALGO, VPIPE_ALGO_NONE) < 0) {
        perror("VIDIOC_S_CTRL ALGO");
        goto cleanup;
    }

    for (i = 0; i < BUFFER_COUNT; i++) {
        if (vpipe_queue_mmap_buffer(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                    V4L2_MEMORY_MMAP, i, -1) < 0) {
            perror("initial CAPTURE qbuf");
            goto cleanup;
        }
    }

    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (vpipe_xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("streamon capture");
            goto cleanup;
        }
        cap_streaming = true;
        type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        if (vpipe_xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            perror("streamon output");
            goto cleanup;
        }
        out_streaming = true;
    }

    for (i = 0; i < (unsigned int) frames; i++) {
        struct v4l2_buffer cap_dq;
        struct v4l2_buffer out_dq;

        if (vpipe_queue_mmap_buffer(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                                    V4L2_MEMORY_MMAP, i % BUFFER_COUNT,
                                    -1) < 0) {
            perror("OUTPUT qbuf");
            goto cleanup;
        }

        memset(&cap_dq, 0, sizeof(cap_dq));
        cap_dq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cap_dq.memory = V4L2_MEMORY_MMAP;
        if (vpipe_xioctl(fd, VIDIOC_DQBUF, &cap_dq) < 0) {
            perror("dqbuf capture");
            goto cleanup;
        }

        memset(&out_dq, 0, sizeof(out_dq));
        out_dq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        out_dq.memory = V4L2_MEMORY_MMAP;
        if (vpipe_xioctl(fd, VIDIOC_DQBUF, &out_dq) < 0) {
            perror("dqbuf output");
            goto cleanup;
        }

        if (vpipe_queue_mmap_buffer(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                    V4L2_MEMORY_MMAP, cap_dq.index, -1) < 0) {
            perror("requeue capture");
            goto cleanup;
        }
    }

    if (vpipe_xioctl(fd, VPIPE_IOC_EXPORT_SRC_DMABUF, &dbuf_fd) < 0) {
        perror("VPIPE_IOC_EXPORT_SRC_DMABUF");
        goto cleanup;
    }
    if (dbuf_fd < 0) {
        fprintf(stderr, "export returned invalid fd %d\n", dbuf_fd);
        goto cleanup;
    }

    map = mmap(NULL, sizeimage, PROT_READ, MAP_SHARED, dbuf_fd, 0);
    if (map == MAP_FAILED) {
        perror("mmap dma-buf");
        goto cleanup;
    }

    {
        const uint8_t *pix = map;
        unsigned long mismatches =
            verify_gradient(pix, WIDTH, HEIGHT, stride);
        uint32_t crc = vpipe_crc32(pix, sizeimage);

        printf("frames=%lu offset_low=%u stride=%u sizeimage=%zu crc32=0x%08x\n",
               frames, pix[0], stride, sizeimage, crc);
        if (mismatches) {
            fprintf(stderr,
                    "FAIL: %lu/%zu pixels diverge from the expected gradient\n",
                    mismatches, sizeimage);
            goto cleanup;
        }
        printf("PASS: imported DMA-BUF matches the deterministic producer "
               "gradient\n");
    }

    rc = 0;

cleanup:
    if (map != MAP_FAILED)
        munmap(map, sizeimage);
    if (dbuf_fd >= 0)
        close(dbuf_fd);
    if (out_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        (void) vpipe_xioctl(fd, VIDIOC_STREAMOFF, &type);
    }
    if (cap_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void) vpipe_xioctl(fd, VIDIOC_STREAMOFF, &type);
    }
    vpipe_unmap_buffers(out_bufs, BUFFER_COUNT);
    vpipe_unmap_buffers(cap_bufs, BUFFER_COUNT);
    if (fd >= 0)
        close(fd);
    return rc;
}
