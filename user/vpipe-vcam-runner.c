/* SPDX-License-Identifier: MIT */

#define _POSIX_C_SOURCE 200809L

#include "vpipe-common.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>


#define WIDTH 640
#define HEIGHT 480
#define STRIDE WIDTH
#define VPIPE_MAX_QUEUE_DEPTH 32


// Backend ids live in the kernel-internal vpipe-source.h

#define VPIPE_BACKEND_FIXTURE 1
#define VPIPE_BACKEND_VCAM_DERIVED 3


enum source_kind { SRC_FIXTURE, SRC_VCAM };
enum buffer_kind { BUF_MMAP, BUF_DMABUF };


struct config {
    unsigned int frames;
    unsigned int fps;
    unsigned int queue_depth;
    enum source_kind source;
    bool verify_pattern;
    const char *device;
    const char *csv;
};


struct run_stats {
    bool ok;
    enum buffer_kind buffer;
    enum source_kind source;
    unsigned int frames;
    unsigned int fps;
    unsigned int queue_depth;
    unsigned int out_inflight;
    uint64_t lat_median_ns;
    uint64_t lat_p95_ns;
    uint64_t lat_p99_ns;
    bool meta_available;
    double occ_mean;
    uint32_t occ_max;
    unsigned int dropped;
    unsigned int csum_checked;
    unsigned int csum_pass;
    unsigned int verify_checked;
    unsigned int verify_pass;
};


struct pacer {
    uint64_t next_ns;
    uint64_t interval_ns;
};


static const char *source_name(enum source_kind s)
{
    return s == SRC_VCAM ? "vcam" : "fixture";
}


static const char *buffer_name(enum buffer_kind b)
{
    return b == BUF_DMABUF ? "dmabuf" : "mmap";
}


static void usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  --frames N         frames to run (default 300)\n"
            "  --fps N            userspace submission pacing, 0=full speed "
            "(default 0)\n"
            "  --queue-depth N    OUTPUT in-flight depth, 2..%u (default 4)\n"
            "  --buffer mmap|dmabuf   OUTPUT transport (default mmap)\n"
            "  --source fixture|vcam  frame producer (default vcam)\n"
            "  --csv PATH         per-frame CSV output (default out.csv)\n"
            "  --device PATH      vpipe m2m node (default /dev/video0)\n"
            "  --compare          mmap vs dmabuf at in-flight=1 (fair "
            "transport), print delta\n"
            "  --verify-pattern   verify vcam deterministic gradient per frame\n"
            "  --help\n",
            prog, VPIPE_MAX_QUEUE_DEPTH);
}


static int parse_uint(const char *s, unsigned long min, unsigned long max,
                      unsigned int *out)
{
    char *end;
    unsigned long v;


    if (!s || *s == '\0' || *s == '-')
        return -1;
    errno = 0;
    v = strtoul(s, &end, 10);
    if (errno || *end != '\0' || v < min || v > max)
        return -1;
    *out = (unsigned int) v;
    return 0;
}


/*
 * Deterministic gradient identical to the vcam-derived backend
 * (vcam_fill_gradient): pixel(x,y) = (x + y + offset) & 0xFF with pixel[0]
 * carrying the low byte of the offset so the pattern is self-describing.
 */
static void fill_gradient(uint8_t *dst, uint32_t offset)
{
    unsigned int x, y;


    for (y = 0; y < HEIGHT; y++) {
        uint8_t *row = dst + (size_t) y * STRIDE;


        for (x = 0; x < WIDTH; x++)
            row[x] = (uint8_t) ((x + y + offset) & 0xFF);
    }
    dst[0] = (uint8_t) (offset & 0xFF);
}


/* Recover the offset from pixel[0] and confirm the whole frame matches. */
static bool verify_gradient(const uint8_t *cap)
{
    uint32_t offset = cap[0];
    unsigned int x, y;


    for (y = 0; y < HEIGHT; y++) {
        const uint8_t *row = cap + (size_t) y * STRIDE;


        for (x = 0; x < WIDTH; x++) {
            uint8_t expect = (uint8_t) ((x + y + offset) & 0xFF);


            if (row[x] != expect)
                return false;
        }
    }
    return true;
}


static int cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *) a;
    uint64_t y = *(const uint64_t *) b;


    return (x > y) - (x < y);
}


/* Linear-interpolated percentile, matching scripts/summarize-benchmark.py. */
static uint64_t percentile(const uint64_t *sorted, size_t n, double pct)
{
    double rank;
    size_t lo, hi;
    double frac;


    if (n == 0)
        return 0;
    if (n == 1)
        return sorted[0];


    rank = (double) (n - 1) * pct;
    lo = (size_t) rank;
    hi = (rank > (double) lo) ? lo + 1 : lo;
    if (hi >= n)
        hi = n - 1;
    frac = rank - (double) lo;
    return (uint64_t) ((double) sorted[lo] +
                       ((double) sorted[hi] - (double) sorted[lo]) * frac);
}


static void pace(struct pacer *p)
{
    struct timespec ts;


    if (!p->interval_ns)
        return;
    if (!p->next_ns)
        p->next_ns = vpipe_now_monotonic_ns();


    ts.tv_sec = (time_t) (p->next_ns / 1000000000ULL);
    ts.tv_nsec = (long) (p->next_ns % 1000000000ULL);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    p->next_ns += p->interval_ns;
}


static int set_backend(int fd, uint32_t backend)
{
    return vpipe_xioctl(fd, VPIPE_IOC_SET_BACKEND, &backend);
}


static int set_algo_none(int fd)
{
    struct v4l2_control ctrl;


    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = VPIPE_CID_ALGO;
    ctrl.value = VPIPE_ALGO_NONE;
    return vpipe_xioctl(fd, VIDIOC_S_CTRL, &ctrl);
}


static int queue_output_dmabuf(int fd, int dmabuf_fd, size_t size)
{
    struct v4l2_buffer buf;


    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = 0;
    buf.m.fd = dmabuf_fd;
    buf.bytesused = (uint32_t) size;
    buf.length = (uint32_t) size;
    return vpipe_xioctl(fd, VIDIOC_QBUF, &buf);
}


static int dqbuf(int fd, enum v4l2_buf_type type, enum v4l2_memory memory,
                 struct v4l2_buffer *out)
{
    memset(out, 0, sizeof(*out));
    out->type = type;
    out->memory = memory;
    return vpipe_xioctl(fd, VIDIOC_DQBUF, out);
}


static int read_meta(int meta_fd, struct vpipe_meta_entry *m)
{
    ssize_t got;


    if (meta_fd < 0)
        return -1;
    got = read(meta_fd, m, sizeof(*m));
    if (got != (ssize_t) sizeof(*m))
        return -1;
    return 0;
}


/*
 * Submit one OUTPUT frame. For the vcam backend the kernel fills the buffer
 * at queue time, so userspace only fills for the fixture source (a no-op
 * backend). Records the QBUF timestamp keyed by submission order.
 */
static int submit_output(int fd,
                         const struct config *cfg,
                         enum buffer_kind buffer,
                         struct vpipe_mmap_buffer *out_bufs,
                         int src_fd,
                         size_t out_sizeimage,
                         unsigned int frame_no,
                         unsigned int mmap_index,
                         struct pacer *pacer,
                         uint64_t *qbuf_ring)
{
    pace(pacer);
    qbuf_ring[frame_no % cfg->queue_depth] = vpipe_now_monotonic_ns();


    if (buffer == BUF_DMABUF)
        return queue_output_dmabuf(fd, src_fd, out_sizeimage);


    if (cfg->source == SRC_FIXTURE)
        fill_gradient(out_bufs[mmap_index].addr, frame_no);


    return vpipe_queue_mmap_buffer(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                                   V4L2_MEMORY_MMAP, mmap_index, -1);
}


static struct run_stats run_one(const struct config *cfg,
                                enum buffer_kind buffer,
                                const char *csv,
                                unsigned int out_inflight)
{
    struct run_stats st = {0};
    struct vpipe_mmap_buffer cap_bufs[VPIPE_MAX_QUEUE_DEPTH] = {0};
    struct vpipe_mmap_buffer out_bufs[VPIPE_MAX_QUEUE_DEPTH] = {0};
    uint64_t qbuf_ring[VPIPE_MAX_QUEUE_DEPTH] = {0};
    uint64_t *lat = NULL;
    struct pacer pacer = {0};
    size_t out_sizeimage;
    unsigned int out_depth;
    unsigned int submitted = 0;
    unsigned int completed = 0;
    unsigned int i;
    double occ_sum = 0.0;
    bool out_streaming = false;
    bool cap_streaming = false;
    int meta_fd = -1;
    int src_fd = -1;
    int fd = -1;
    FILE *fp = NULL;


    st.buffer = buffer;
    st.source = cfg->source;
    st.frames = cfg->frames;
    st.fps = cfg->fps;
    st.queue_depth = cfg->queue_depth;


    for (i = 0; i < cfg->queue_depth; i++) {
        cap_bufs[i].dmabuf_fd = -1;
        out_bufs[i].dmabuf_fd = -1;
    }


    out_depth = (buffer == BUF_DMABUF) ? 1 : out_inflight;
    if (out_depth > cfg->queue_depth)
        out_depth = cfg->queue_depth;
    if (out_depth < 1)
        out_depth = 1;
    st.out_inflight = out_depth;


    lat = malloc((size_t) cfg->frames * sizeof(*lat));
    if (!lat) {
        fprintf(stderr, "out of memory for %u latency samples\n", cfg->frames);
        return st;
    }


    fd = open(cfg->device, O_RDWR);
    if (fd < 0) {
        perror("open vpipe");
        goto cleanup;
    }


    if (set_backend(fd, cfg->source == SRC_VCAM ? VPIPE_BACKEND_VCAM_DERIVED
                                                : VPIPE_BACKEND_FIXTURE) < 0) {
        perror("VPIPE_IOC_SET_BACKEND");
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


    if (buffer == BUF_DMABUF) {
        struct v4l2_requestbuffers req;


        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        req.memory = V4L2_MEMORY_DMABUF;
        if (vpipe_xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS OUTPUT DMABUF");
            goto cleanup;
        }
    } else if (vpipe_request_mmap_buffers(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                                          cfg->queue_depth, out_bufs) < 0) {
        perror("REQBUFS/mmap OUTPUT");
        goto cleanup;
    }


    if (vpipe_request_mmap_buffers(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                   cfg->queue_depth, cap_bufs) < 0) {
        perror("REQBUFS/mmap CAPTURE");
        goto cleanup;
    }


    if (set_algo_none(fd) < 0) {
        perror("VIDIOC_S_CTRL ALGO");
        goto cleanup;
    }


    fp = vpipe_open_csv(
        csv,
        "frame,buffer_type,us_qbuf_ns,us_dqbuf_ns,us_latency_ns,"
        "meta_seq,source_ts_ns,qbuf_ts_ns,capture_done_ns,dqbuf_ts_ns,"
        "queue_depth,cpu_id,crc32_kernel,crc32_user,dropped,flags");
    if (!fp) {
        perror("csv");
        goto cleanup;
    }


    /* Open the sideband before frames flow so the reader cursor starts at the
     * current head and captures exactly this run's entries.
     */
    meta_fd = open("/dev/vpipe-meta", O_RDONLY);
    if (meta_fd < 0)
        fprintf(stderr,
                "warning: /dev/vpipe-meta: %s; kernel timeline, occupancy, "
                "dropped, and checksum columns disabled\n",
                strerror(errno));
    st.meta_available = meta_fd >= 0;


    for (i = 0; i < cfg->queue_depth; i++) {
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


    if (buffer == BUF_DMABUF) {
        if (vpipe_xioctl(fd, VPIPE_IOC_EXPORT_SRC_DMABUF, &src_fd) < 0) {
            perror("VPIPE_IOC_EXPORT_SRC_DMABUF");
            goto cleanup;
        }
        if (src_fd < 0) {
            fprintf(stderr, "export returned invalid fd %d\n", src_fd);
            goto cleanup;
        }
    }


    pacer.interval_ns = cfg->fps ? 1000000000ULL / cfg->fps : 0;


    /* Fill the OUTPUT pipeline before draining so occupancy can build up. */
    while (submitted < cfg->frames && submitted < out_depth) {
        if (submit_output(fd, cfg, buffer, out_bufs, src_fd, out_sizeimage,
                          submitted, submitted, &pacer, qbuf_ring) < 0) {
            perror("OUTPUT qbuf");
            goto cleanup;
        }
        submitted++;
    }


    while (completed < cfg->frames) {
        unsigned int inflight = submitted - completed;
        struct v4l2_buffer cap_dq;
        struct v4l2_buffer out_dq;
        struct vpipe_meta_entry m;
        bool have_meta;
        uint64_t t_dq;
        uint32_t crc_user;
        bool dropped = false;


        /* Occupancy = frames in flight between QBUF and DQBUF. The kernel
         * queue_depth column reports OUTPUT-ready backlog only, which stays ~0
         * because the m2m scheduler drains each buffer synchronously on QBUF;
         * this userspace gauge captures the true pipeline depth instead.
         */
        occ_sum += (double) inflight;
        if (inflight > st.occ_max)
            st.occ_max = inflight;


        if (dqbuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP,
                  &cap_dq) < 0) {
            perror("dqbuf capture");
            goto cleanup;
        }
        t_dq = vpipe_now_monotonic_ns();
        if (cap_dq.index >= cfg->queue_depth) {
            fprintf(stderr, "capture index out of range: %u\n", cap_dq.index);
            goto cleanup;
        }


        crc_user = vpipe_crc32(cap_bufs[cap_dq.index].addr, cap_dq.bytesused);
        have_meta = read_meta(meta_fd, &m) == 0;


        lat[completed] = t_dq - qbuf_ring[completed % cfg->queue_depth];


        if (have_meta) {
            dropped = (m.flags & VPIPE_META_F_DROPPED) != 0;
            if (dropped)
                st.dropped++;
            st.csum_checked++;
            if (crc_user == m.crc32)
                st.csum_pass++;
        }


        if (cfg->verify_pattern && cfg->source == SRC_VCAM && !dropped) {
            st.verify_checked++;
            if (verify_gradient(cap_bufs[cap_dq.index].addr))
                st.verify_pass++;
        }


        fprintf(fp,
                "%u,%u,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
                ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u,%u,0x%08x,0x%08x,%u,"
                "0x%08x\n",
                completed, have_meta ? m.buffer_type : 0,
                qbuf_ring[completed % cfg->queue_depth], t_dq,
                lat[completed], have_meta ? (uint64_t) m.seq : 0,
                have_meta ? (uint64_t) m.source_timestamp_ns : 0,
                have_meta ? (uint64_t) m.qbuf_timestamp_ns : 0,
                have_meta ? (uint64_t) m.capture_done_ns : 0,
                have_meta ? (uint64_t) m.dqbuf_timestamp_ns : 0,
                have_meta ? m.queue_depth : 0, have_meta ? m.cpu_id : 0,
                have_meta ? m.crc32 : 0, crc_user, dropped ? 1 : 0,
                have_meta ? m.flags : 0);


        completed++;


        if (vpipe_queue_mmap_buffer(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                    V4L2_MEMORY_MMAP, cap_dq.index, -1) < 0) {
            perror("requeue capture");
            goto cleanup;
        }


        if (dqbuf(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                  buffer == BUF_DMABUF ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP,
                  &out_dq) < 0) {
            perror("dqbuf output");
            goto cleanup;
        }


        if (submitted < cfg->frames) {
            if (submit_output(fd, cfg, buffer, out_bufs, src_fd, out_sizeimage,
                              submitted, out_dq.index, &pacer, qbuf_ring) < 0) {
                perror("OUTPUT qbuf");
                goto cleanup;
            }
            submitted++;
        }
    }


    qsort(lat, cfg->frames, sizeof(*lat), cmp_u64);
    st.lat_median_ns = percentile(lat, cfg->frames, 0.5);
    st.lat_p95_ns = percentile(lat, cfg->frames, 0.95);
    st.lat_p99_ns = percentile(lat, cfg->frames, 0.99);
    if (cfg->frames)
        st.occ_mean = occ_sum / (double) cfg->frames;
    st.ok = true;


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
    if (meta_fd >= 0)
        close(meta_fd);
    if (src_fd >= 0)
        close(src_fd);
    if (buffer == BUF_MMAP)
        vpipe_unmap_buffers(out_bufs, cfg->queue_depth);
    vpipe_unmap_buffers(cap_bufs, cfg->queue_depth);
    if (fd >= 0)
        close(fd);
    free(lat);
    return st;
}


static void print_summary(const struct run_stats *st)
{
    printf("frames=%u source=%s buffer=%s fps=%u queue_depth=%u inflight=%u\n",
           st->frames, source_name(st->source), buffer_name(st->buffer),
           st->fps, st->queue_depth, st->out_inflight);
    printf("latency_us   median=%.3f p95=%.3f p99=%.3f\n",
           st->lat_median_ns / 1000.0, st->lat_p95_ns / 1000.0,
           st->lat_p99_ns / 1000.0);
    printf("occupancy    mean=%.2f max=%u (in-flight)\n", st->occ_mean,
           st->occ_max);
    if (st->meta_available) {
        printf("dropped      %u\n", st->dropped);
        printf("checksum     %s (%u/%u match)\n",
               st->csum_checked && st->csum_pass == st->csum_checked ? "PASS"
                                                                     : "FAIL",
               st->csum_pass, st->csum_checked);
    } else {
        printf("dropped      n/a (no /dev/vpipe-meta)\n");
        printf("checksum     n/a (no /dev/vpipe-meta)\n");
    }
    if (st->verify_checked)
        printf("pattern      %s (%u/%u match)\n",
               st->verify_pass == st->verify_checked ? "PASS" : "FAIL",
               st->verify_pass, st->verify_checked);
}


static void print_delta_us(const char *label, uint64_t a_ns, uint64_t b_ns)
{
    double a = a_ns / 1000.0;
    double b = b_ns / 1000.0;


    printf("%-16s %-11.3f %-11.3f ", label, a, b);
    if (a_ns)
        printf("%+.0f%%\n", (b - a) / a * 100.0);
    else
        printf("n/a\n");
}


static void print_comparison(const struct run_stats *mmap_st,
                             const struct run_stats *dmabuf_st)
{
    if (!mmap_st->ok || !dmabuf_st->ok) {
        printf("comparison unavailable: %s run failed\n",
               !mmap_st->ok ? "mmap" : "dmabuf");
        return;
    }


    printf("\nMMAP vs DMA-BUF (source=%s frames=%u fps=%u, lockstep "
           "in-flight=1 for fair transport)\n",
           source_name(mmap_st->source), mmap_st->frames, mmap_st->fps);
    printf("%-16s %-11s %-11s %s\n", "metric", "mmap", "dmabuf", "delta");
    print_delta_us("median_lat_us", mmap_st->lat_median_ns,
                   dmabuf_st->lat_median_ns);
    print_delta_us("p95_lat_us", mmap_st->lat_p95_ns, dmabuf_st->lat_p95_ns);
    print_delta_us("p99_lat_us", mmap_st->lat_p99_ns, dmabuf_st->lat_p99_ns);
    printf("%-16s %-11.2f %-11.2f\n", "occupancy_mean", mmap_st->occ_mean,
           dmabuf_st->occ_mean);
    if (mmap_st->meta_available && dmabuf_st->meta_available) {
        printf("%-16s %-11u %-11u\n", "dropped", mmap_st->dropped,
               dmabuf_st->dropped);
        printf("%-16s %-11s %-11s\n", "checksum",
               mmap_st->csum_pass == mmap_st->csum_checked ? "PASS" : "FAIL",
               dmabuf_st->csum_pass == dmabuf_st->csum_checked ? "PASS"
                                                               : "FAIL");
    }
}


int main(int argc, char **argv)
{
    struct config cfg = {
        .frames = 300,
        .fps = 0,
        .queue_depth = 4,
        .source = SRC_VCAM,
        .verify_pattern = false,
        .device = "/dev/video0",
        .csv = "out.csv",
    };
    enum buffer_kind buffer = BUF_MMAP;
    bool compare = false;
    int opt;


    static const struct option longopts[] = {
        {"frames", required_argument, 0, 'f'},
        {"fps", required_argument, 0, 'r'},
        {"queue-depth", required_argument, 0, 'q'},
        {"buffer", required_argument, 0, 'b'},
        {"source", required_argument, 0, 's'},
        {"csv", required_argument, 0, 'c'},
        {"device", required_argument, 0, 'd'},
        {"compare", no_argument, 0, 'C'},
        {"verify-pattern", no_argument, 0, 'V'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0},
    };


    while ((opt = getopt_long(argc, argv, "f:r:q:b:s:c:d:CVh", longopts,
                              NULL)) != -1) {
        switch (opt) {
        case 'f':
            if (parse_uint(optarg, 1, UINT_MAX, &cfg.frames) < 0) {
                fprintf(stderr, "invalid --frames: %s\n", optarg);
                return 2;
            }
            break;
        case 'r':
            if (parse_uint(optarg, 0, 1000, &cfg.fps) < 0) {
                fprintf(stderr, "invalid --fps: %s\n", optarg);
                return 2;
            }
            break;
        case 'q':
            if (parse_uint(optarg, 2, VPIPE_MAX_QUEUE_DEPTH, &cfg.queue_depth) <
                0) {
                fprintf(stderr, "invalid --queue-depth: %s\n", optarg);
                return 2;
            }
            break;
        case 'b':
            if (!strcmp(optarg, "mmap"))
                buffer = BUF_MMAP;
            else if (!strcmp(optarg, "dmabuf"))
                buffer = BUF_DMABUF;
            else {
                fprintf(stderr, "invalid --buffer: %s\n", optarg);
                return 2;
            }
            break;
        case 's':
            if (!strcmp(optarg, "fixture"))
                cfg.source = SRC_FIXTURE;
            else if (!strcmp(optarg, "vcam"))
                cfg.source = SRC_VCAM;
            else {
                fprintf(stderr, "invalid --source: %s\n", optarg);
                return 2;
            }
            break;
        case 'c':
            cfg.csv = optarg;
            break;
        case 'd':
            cfg.device = optarg;
            break;
        case 'C':
            compare = true;
            break;
        case 'V':
            cfg.verify_pattern = true;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 2;
        }
    }

    if (cfg.source == SRC_FIXTURE &&
        (buffer == BUF_DMABUF || compare)) {
        fprintf(stderr,
                "source=fixture with dmabuf is not supported yet; use "
                "--source vcam for dmabuf/compare runs\n");
        return 2;
    }


    if (compare) {
        struct run_stats mmap_st, dmabuf_st;
        char mmap_csv[PATH_MAX];
        char dmabuf_csv[PATH_MAX];


        snprintf(mmap_csv, sizeof(mmap_csv), "%s.mmap.csv", cfg.csv);
        snprintf(dmabuf_csv, sizeof(dmabuf_csv), "%s.dmabuf.csv", cfg.csv);


        mmap_st = run_one(&cfg, BUF_MMAP, mmap_csv, 1);
        print_summary(&mmap_st);
        printf("\n");
        dmabuf_st = run_one(&cfg, BUF_DMABUF, dmabuf_csv, 1);
        print_summary(&dmabuf_st);
        print_comparison(&mmap_st, &dmabuf_st);
        return (mmap_st.ok && dmabuf_st.ok) ? 0 : 1;
    }

    {
        struct run_stats st = run_one(&cfg, buffer, cfg.csv, cfg.queue_depth);

        print_summary(&st);
        return st.ok ? 0 : 1;
    }
}