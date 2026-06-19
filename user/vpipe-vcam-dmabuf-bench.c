/* SPDX-License-Identifier: MIT */


#define _POSIX_C_SOURCE 200809L

#include "vpipe-common.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
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

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/video0";
    const char *csv = argc > 2 ? argv[2] : "bench/vcam-dmabuf.csv";
    unsigned long frames = 6000;
    struct vpipe_mmap_buffer cap_bufs[BUFFER_COUNT] = {0};
    uint32_t backend = VPIPE_BACKEND_VCAM_DERIVED;
    size_t out_sizeimage;
    bool out_streaming = false;
    bool cap_streaming = false;
    FILE *fp = NULL;
    __s32 src_fd = -1;
    int fd = -1;
    int rc = 1;
    unsigned int i;

    for (i = 0; i < BUFFER_COUNT; i++)
        cap_bufs[i].dmabuf_fd = -1;

    if (argc > 3 && parse_uint_range(argv[3], 1, UINT_MAX, &frames) < 0) {
        fprintf(stderr, "invalid frames: %s\n", argv[3]);
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
    out_sizeimage = vpipe_get_sizeimage(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT);
    if (out_sizeimage == 0) {
        fprintf(stderr, "VIDIOC_G_FMT OUTPUT: zero sizeimage\n");
        goto cleanup;
    }


    {
        struct v4l2_requestbuffers req;

        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        req.memory = V4L2_MEMORY_DMABUF;
        if (vpipe_xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS OUTPUT DMABUF");
            goto cleanup;
        }
    }
    if (vpipe_request_mmap_buffers(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                   BUFFER_COUNT, cap_bufs) < 0) {
        perror("REQBUFS/mmap CAPTURE");
        goto cleanup;
    }

    if (set_ctrl(fd, VPIPE_CID_ALGO, VPIPE_ALGO_NONE) < 0) {
        perror("VIDIOC_S_CTRL ALGO");
        goto cleanup;
    }

    fp = vpipe_open_csv(csv,
                        "frame,src_sequence,enqueue_monotonic_ns,dequeue_"
                        "monotonic_ns,bytesused,crc32");
    if (!fp) {
        perror("csv");
        goto cleanup;
    }
    setvbuf(fp, NULL, _IOLBF, 0);

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

    if (vpipe_xioctl(fd, VPIPE_IOC_EXPORT_SRC_DMABUF, &src_fd) < 0) {
        perror("VPIPE_IOC_EXPORT_SRC_DMABUF");
        goto cleanup;
    }
    if (src_fd < 0) {
        fprintf(stderr, "export returned invalid fd %d\n", src_fd);
        goto cleanup;
    }

    for (i = 0; i < (unsigned int) frames; i++) {
        struct v4l2_buffer qbuf;
        struct v4l2_buffer cap_dq;
        struct v4l2_buffer out_dq;
        uint64_t enqueue_ns;
        uint64_t dequeue_ns;
        uint32_t crc;

        memset(&qbuf, 0, sizeof(qbuf));
        qbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        qbuf.memory = V4L2_MEMORY_DMABUF;
        qbuf.index = 0;
        qbuf.m.fd = src_fd;
        qbuf.bytesused = (uint32_t) out_sizeimage;
        qbuf.length = (uint32_t) out_sizeimage;

        enqueue_ns = vpipe_now_monotonic_ns();
        if (vpipe_xioctl(fd, VIDIOC_QBUF, &qbuf) < 0) {
            perror("OUTPUT qbuf DMABUF");
            goto cleanup;
        }

        memset(&cap_dq, 0, sizeof(cap_dq));
        cap_dq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cap_dq.memory = V4L2_MEMORY_MMAP;
        if (vpipe_xioctl(fd, VIDIOC_DQBUF, &cap_dq) < 0) {
            perror("dqbuf capture");
            goto cleanup;
        }
        dequeue_ns = vpipe_now_monotonic_ns();
        if (cap_dq.index >= BUFFER_COUNT) {
            fprintf(stderr, "capture index out of range: %u\n", cap_dq.index);
            goto cleanup;
        }

        memset(&out_dq, 0, sizeof(out_dq));
        out_dq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        out_dq.memory = V4L2_MEMORY_DMABUF;
        if (vpipe_xioctl(fd, VIDIOC_DQBUF, &out_dq) < 0) {
            perror("dqbuf output");
            goto cleanup;
        }

        crc = vpipe_crc32(cap_bufs[cap_dq.index].addr, cap_dq.bytesused);
        fprintf(fp, "%u,%u,%" PRIu64 ",%" PRIu64 ",%u,0x%08x\n", i,
                cap_dq.sequence, enqueue_ns, dequeue_ns, cap_dq.bytesused, crc);

        if (vpipe_queue_mmap_buffer(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                    V4L2_MEMORY_MMAP, cap_dq.index, -1) < 0) {
            perror("requeue capture");
            goto cleanup;
        }
    }

    rc = 0;

cleanup:
    if (out_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        (void) vpipe_xioctl(fd, VIDIOC_STREAMOFF, &type);
    }
    if (cap_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void) vpipe_xioctl(fd, VIDIOC_STREAMOFF, &type);
    }
    if (fp)
        fclose(fp);
    if (src_fd >= 0)
        close(src_fd);
    vpipe_unmap_buffers(cap_bufs, BUFFER_COUNT);
    if (fd >= 0)
        close(fd);
    return rc;
}
