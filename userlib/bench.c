/*
 * bench.c — sfc7120 userspace direct-path latency/throughput harness (sfcbench).
 *
 * Measures how fast a packet travels port-to-port (PF0 -> PF1 -> PF0) over the
 * zero-syscall kernel-bypass data path (sfc7120_tx_post / sfc7120_poll /
 * sfc7120_rx_recv). This is the Phase-G benchmark counterpart to test.c's
 * functional smoke test.
 *
 * Topology: SINGLE PROCESS, BOTH PFs. PF0 and PF1 are cabled port-to-port via
 * an SFP+ DAC, so a frame TX'd on PF0 ingresses PF1. One process opens both
 * /dev/sfc7120pol0 and /dev/sfc7120pol1 and bounces a frame across the wire and
 * back, timing the full round trip with a single monotonic clock — no
 * cross-process clock skew, no reflector-process scheduling jitter.
 *
 *   t0 -> tx_post(PF0 -> PF1) -> [arrives PF1] -> reflect: swap MACs,
 *         tx_post(PF1 -> PF0) -> [arrives PF0] -> t1.   rtt = t1 - t0.
 *
 * Two modes, distinguished only by how many packets are kept in flight:
 *   - latency   : window = 1 (one packet bouncing) -> RTT distribution
 *   - throughput: window = W (W packets circulating) -> round-trips/sec
 *
 * Both sweep frame sizes {64, 1024, 1518}, over either data path:
 *   - direct : zero-syscall ef_vi path (tx_post / poll / rx_recv), the
 *              throughput window is TPUT_WINDOW packets in flight
 *   - kernel : the SFC7120_TX / SFC7120_RX ioctl oracle. The TX ioctl blocks
 *              on its own TX completion and consumes every EVQ event it walks
 *              past (including RX_EVs), so the path is synchronous by
 *              construction: throughput runs window=1 — its ceiling IS 1/RTT,
 *              which is exactly the comparison point against the direct path.
 *
 * Both paths print identical tables so results compare row-for-row. When both
 * paths run in one invocation, each gets its own attach session (destroy +
 * re-init in between): direct-path use makes destroy sync the EVQ read ptr /
 * ring heads back to the kernel, and the kernel ioctls need those pointers
 * fresh — mixing paths in one session would leave the kernel polling a stale
 * data_evq_read_ptr.
 *
 * Usage: ./sfcbench [lat|tput|both] [direct|kernel|both]   (default: both both)
 *
 * Requires modmap.ko + sfc7120pol.ko loaded and sfxge unloaded (see ../CLAUDE.md).
 */
#include "sfc7120_user.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DEV_PF0     "/dev/sfc7120pol0"
#define DEV_PF1     "/dev/sfc7120pol1"

#define ETHERTYPE   0x88B5      /* IEEE 802 local experimental */

/* Frame-size sweep (Ethernet payload+header, sans FCS). */
static const size_t g_sizes[] = { 64, 1024, 1518 };
#define N_SIZES (sizeof(g_sizes) / sizeof(g_sizes[0]))

#define LAT_WARMUP    1000      /* discarded warmup round trips */
#define LAT_SAMPLES   20000     /* measured RTT samples per size */

#define TPUT_WINDOW   128       /* packets in flight; must stay < ring depth 512 */
#define TPUT_NS       2000000000ULL  /* 2 s measurement window per size */

/* Per-packet RX wall-clock budget. A bounced frame's real round trip is
 * microseconds, so 1 s is enormous slack; a dropped packet trips it and ends
 * the run with a direction-tagged diagnostic instead of spinning for minutes.
 * Wall-clock (not a fixed try count) because per-poll cost over NIC-coherent
 * memory varies wildly, so a try count gives an unpredictable timeout. */
#define RX_TIMEOUT_NS    1000000000ULL

/* ------------------------------------------------------------------ timing */

static inline uint64_t
now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* ------------------------------------------------------------------ frames */

/*
 * build_frame — DST = peer MAC, SRC = own MAC, ethertype 0x88B5, 32-bit seq at
 * payload offset 0 (frame[14..17]), rest filled. Zero-padded to len.
 */
static void
build_frame(uint8_t *frame, const uint8_t dst[6], const uint8_t src[6],
            size_t len, uint32_t seq)
{
    memset(frame, 0, len);
    memcpy(&frame[0], dst, 6);
    memcpy(&frame[6], src, 6);
    frame[12] = (ETHERTYPE >> 8) & 0xff;
    frame[13] = ETHERTYPE & 0xff;
    memcpy(&frame[14], &seq, sizeof(seq));
    if (len > 18)
        memset(&frame[18], 0xA5, len - 18);
}

/* swap_mac — swap DST<->SRC in place so a received frame echoes back to its
 * sender. The peer's exact-DST_MAC RX filter then matches on the return trip. */
static void
swap_mac(uint8_t *frame)
{
    uint8_t tmp[6];
    memcpy(tmp,        &frame[0], 6);
    memcpy(&frame[0],  &frame[6], 6);
    memcpy(&frame[6],  tmp,       6);
}

/*
 * wait_rx — spin sfc7120_poll until one RX_EV arrives, then read + recycle that
 * slot via sfc7120_rx_recv. TX_EVs (and OTHER) are consumed and ignored. Returns
 * 0 on success with the frame in buf / its length in *len_out, -1 on timeout or
 * poll/recv error.
 */
static int
wait_rx(sfc7120_if_t *sfc, void *buf, size_t *len_out)
{
    sfc7120_ev_t evs[8];
    uint64_t deadline = 0;   /* armed on the first slow-path check below */
    long     tries    = 0;

    for (;;) {
        int n = sfc7120_poll(sfc, evs, 8);
        if (n < 0)
            return -1;
        for (int j = 0; j < n; j++) {
            if (evs[j].type != SFC7120_EV_RX)
                continue;   /* TX completion / driver event — ignore */
            if (sfc7120_rx_recv(sfc, buf, len_out, evs[j].rx_bytes) != 0)
                return -1;
            return 0;
        }
        /* Amortize the clock read: the common case returns above long before
         * 65536 empty polls, so a promptly-arriving frame never reads the
         * clock here — the data path stays zero-syscall. Only a stalled wait
         * arms the deadline and trips the timeout. */
        if ((++tries & 0xffff) == 0) {
            uint64_t t = now_ns();
            if (deadline == 0)
                deadline = t + RX_TIMEOUT_NS;
            else if (t >= deadline)
                return -1;  /* timed out — packet lost */
        }
    }
}

/*
 * diag_evq — dump an interface's data-path state on an RX timeout. The decisive
 * question is whether the NIC wrote an event we're failing to read: we scan the
 * whole mapped EVQ ring for any non-empty slot (poll zeroes consumed slots; the
 * NIC writes a non-zero, non-all-ones event word). A non-empty slot *ahead* of
 * evq_read_ptr ⇒ the read pointer desynced from where the NIC is writing; an
 * entirely empty ring ⇒ the NIC posted nothing (RX buffers exhausted / frame
 * dropped — the unaligned-RX-WPTR / Gotcha 3 hypothesis).
 */
static void
diag_evq(const char *who, sfc7120_if_t *sfc)
{
    volatile uint64_t *evq = (volatile uint64_t *)sfc->evq_ring;
    int      nonempty  = 0;
    int      first_idx = -1;
    uint64_t first_val = 0;

    for (int i = 0; i < SFC7120_NUM_EVQ_ENTRY; i++) {
        uint64_t v = evq[i];
        if (v != 0 && v != 0xffffffffffffffffULL) {
            if (first_idx < 0) { first_idx = i; first_val = v; }
            nonempty++;
        }
    }

    fprintf(stderr,
            "  [%s diag] evq_read_ptr=%u rx_head=%u tx_head=%u | "
            "EVQ non-empty=%d/%d",
            who, sfc->evq_read_ptr, sfc->rx_head, sfc->tx_head,
            nonempty, SFC7120_NUM_EVQ_ENTRY);
    if (first_idx >= 0)
        fprintf(stderr, " first@%d=0x%016llx code=%llu",
                first_idx, (unsigned long long)first_val,
                (unsigned long long)((first_val >> 60) & 0xf));
    fprintf(stderr, "\n");
}

/* ------------------------------------------------------------------ paths */

/*
 * bench_path_t — the two data paths share one latency loop. tx submits one
 * frame; rx blocks until one frame arrives. Direct: tx_post + wait_rx (their
 * signatures match exactly). Kernel: the blocking ioctls — sfc7120_rx's
 * timeout is the kernel's own EVQ spin loop (ETIMEDOUT -> -1), so no
 * userspace deadline is layered on top.
 */
typedef struct {
    const char *name;
    int  (*tx)(sfc7120_if_t *sfc, const void *buf, size_t len);
    int  (*rx)(sfc7120_if_t *sfc, void *buf, size_t *len_out);
    bool direct;    /* diag_evq is only meaningful for the direct path */
} bench_path_t;

static const bench_path_t g_path_direct = {
    .name = "direct", .tx = sfc7120_tx_post, .rx = wait_rx, .direct = true,
};
static const bench_path_t g_path_kernel = {
    .name = "kernel ioctl", .tx = sfc7120_tx, .rx = sfc7120_rx, .direct = false,
};

/* --------------------------------------------------------------- latency */

static int
cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

/*
 * run_latency — window-of-1 ping-pong over either path. For each size, bounce
 * LAT_SAMPLES frames (after LAT_WARMUP discarded) and report the RTT
 * distribution. pf0 is the initiator+timer; pf1 reflects.
 */
static int
run_latency(const bench_path_t *p, sfc7120_if_t *pf0, sfc7120_if_t *pf1)
{
    uint8_t  dst[6], frame[2048], rxbuf[2048];
    uint64_t *samples = malloc(LAT_SAMPLES * sizeof(uint64_t));

    if (samples == NULL) {
        perror("sfcbench: malloc samples");
        return -1;
    }

    /* PF1 MAC = PF0 base MAC + 1 in the low octet. */
    memcpy(dst, pf0->mac_addr, 6);
    dst[5] += 1;

    printf("\n=== Latency [%s] (round-trip ping-pong, window=1, %d samples) ===\n",
           p->name, LAT_SAMPLES);
    printf("%6s  %10s  %10s  %10s  %10s   %10s\n",
           "bytes", "min_ns", "mean_ns", "med_ns", "p99_ns", "1way_ns");

    for (size_t s = 0; s < N_SIZES; s++) {
        size_t   size = g_sizes[s];
        uint32_t seq  = 0;

        for (long i = 0; i < LAT_WARMUP + LAT_SAMPLES; i++) {
            size_t len = 0;

            build_frame(frame, dst, pf0->mac_addr, size, seq++);
            uint64_t t0 = now_ns();

            if (p->tx(pf0, frame, size) != 0) {
                fprintf(stderr, "sfcbench: tx PF0 failed (size %zu)\n", size);
                free(samples);
                return -1;
            }
            /* Frame arrives at PF1 -> reflect it back to PF0. */
            if (p->rx(pf1, rxbuf, &len) != 0) {
                fprintf(stderr, "sfcbench: PF1 RX timeout (size %zu, i=%ld)\n",
                        size, i);
                if (p->direct) {
                    diag_evq("PF1", pf1);
                    diag_evq("PF0", pf0);
                }
                free(samples);
                return -1;
            }
            swap_mac(rxbuf);
            if (p->tx(pf1, rxbuf, len) != 0) {
                fprintf(stderr, "sfcbench: tx PF1 echo failed\n");
                free(samples);
                return -1;
            }
            /* Echo arrives back at PF0 — round trip complete. */
            if (p->rx(pf0, rxbuf, &len) != 0) {
                fprintf(stderr, "sfcbench: PF0 RX timeout (size %zu, i=%ld)\n",
                        size, i);
                if (p->direct) {
                    diag_evq("PF0", pf0);
                    diag_evq("PF1", pf1);
                }
                free(samples);
                return -1;
            }
            uint64_t t1 = now_ns();

            if (i >= LAT_WARMUP)
                samples[i - LAT_WARMUP] = t1 - t0;
        }

        qsort(samples, LAT_SAMPLES, sizeof(uint64_t), cmp_u64);
        uint64_t sum = 0;
        for (long i = 0; i < LAT_SAMPLES; i++)
            sum += samples[i];
        uint64_t mean = sum / LAT_SAMPLES;
        uint64_t med  = samples[LAT_SAMPLES / 2];
        uint64_t p99  = samples[(LAT_SAMPLES * 99) / 100];

        printf("%6zu  %10llu  %10llu  %10llu  %10llu   %10llu\n",
               size,
               (unsigned long long)samples[0],
               (unsigned long long)mean,
               (unsigned long long)med,
               (unsigned long long)p99,
               (unsigned long long)(med / 2));
    }

    free(samples);
    return 0;
}

/* ------------------------------------------------------------- throughput */

/*
 * run_throughput — window-of-W sustained ping-pong. Prime W frames on PF0, then
 * keep ~W packets circulating: reflect every arrival on PF1, and on every echo
 * back at PF0 count one completed round trip and relaunch one frame to refill
 * the window. Counting keys off PF0 RX echoes, so EF10 TX-completion coalescing
 * does not perturb the count. Runs TPUT_NS per size.
 */
static int
run_throughput(sfc7120_if_t *pf0, sfc7120_if_t *pf1)
{
    uint8_t      dst[6], frame[2048], rxbuf[2048];
    sfc7120_ev_t evs[8];

    memcpy(dst, pf0->mac_addr, 6);
    dst[5] += 1;

    printf("\n=== Throughput [direct] (sustained ping-pong, window=%d, %llu ms) ===\n",
           TPUT_WINDOW, (unsigned long long)(TPUT_NS / 1000000ULL));
    printf("%6s  %14s  %14s  %12s\n",
           "bytes", "roundtrips/s", "pkts/s(1way)", "Gbit/s(bidir)");

    for (size_t s = 0; s < N_SIZES; s++) {
        size_t   size = g_sizes[s];
        uint32_t seq  = 0;
        uint64_t round_trips = 0;
        long     iter = 0;

        /* Prime the window: W frames PF0 -> PF1. */
        for (int k = 0; k < TPUT_WINDOW; k++) {
            build_frame(frame, dst, pf0->mac_addr, size, seq++);
            if (sfc7120_tx_post(pf0, frame, size) != 0) {
                fprintf(stderr, "sfcbench: tput prime tx_post failed\n");
                return -1;
            }
        }

        uint64_t t0 = now_ns();
        for (;;) {
            /* Service PF1: reflect each arrival back to PF0. */
            int n = sfc7120_poll(pf1, evs, 8);
            if (n < 0) { fprintf(stderr, "sfcbench: poll PF1\n"); return -1; }
            for (int j = 0; j < n; j++) {
                if (evs[j].type != SFC7120_EV_RX)
                    continue;
                size_t len = 0;
                if (sfc7120_rx_recv(pf1, rxbuf, &len, evs[j].rx_bytes) != 0)
                    return -1;
                swap_mac(rxbuf);
                if (sfc7120_tx_post(pf1, rxbuf, len) != 0)
                    return -1;
            }

            /* Service PF0: count round trips + relaunch to keep window full. */
            n = sfc7120_poll(pf0, evs, 8);
            if (n < 0) { fprintf(stderr, "sfcbench: poll PF0\n"); return -1; }
            for (int j = 0; j < n; j++) {
                if (evs[j].type != SFC7120_EV_RX)
                    continue;
                size_t len = 0;
                if (sfc7120_rx_recv(pf0, rxbuf, &len, evs[j].rx_bytes) != 0)
                    return -1;
                round_trips++;
                build_frame(frame, dst, pf0->mac_addr, size, seq++);
                if (sfc7120_tx_post(pf0, frame, size) != 0)
                    return -1;
            }

            /* Check the clock occasionally to amortize now_ns(). */
            if ((iter++ & 0xff) == 0 && now_ns() - t0 >= TPUT_NS)
                break;
        }
        uint64_t elapsed = now_ns() - t0;

        if (round_trips == 0) {
            printf("%6zu  %14s  (no round trips completed — check the wire / "
                   "reverse-direction RX)\n", size, "0");
            continue;
        }

        double secs   = (double)elapsed / 1e9;
        double rts     = (double)round_trips / secs;
        double pps     = rts;                 /* one-directional packet rate */
        double gbps    = (rts * 2.0 * (double)size * 8.0) / 1e9;  /* both dirs */

        printf("%6zu  %14.0f  %14.0f  %12.3f\n", size, rts, pps, gbps);
    }

    return 0;
}

/*
 * run_throughput_kernel — sustained ping-pong over the ioctl path, window=1.
 * The SFC7120_TX ioctl blocks on its own TX completion and consumes every EVQ
 * event it walks past (including RX_EVs), so a W>1 window would have echo
 * RX events silently eaten while a TX ioctl spins — the circulating window
 * decays until the run times out. With window=1 nothing overlaps, no event is
 * ever eaten, and the measured rate is the path's true ceiling: 1/RTT.
 * Same table format as the direct run so results compare row-for-row.
 */
static int
run_throughput_kernel(sfc7120_if_t *pf0, sfc7120_if_t *pf1)
{
    uint8_t dst[6], frame[2048], rxbuf[2048];

    memcpy(dst, pf0->mac_addr, 6);
    dst[5] += 1;

    printf("\n=== Throughput [kernel ioctl] (sustained ping-pong, window=1 "
           "(path is synchronous), %llu ms) ===\n",
           (unsigned long long)(TPUT_NS / 1000000ULL));
    printf("%6s  %14s  %14s  %12s\n",
           "bytes", "roundtrips/s", "pkts/s(1way)", "Gbit/s(bidir)");

    for (size_t s = 0; s < N_SIZES; s++) {
        size_t   size = g_sizes[s];
        uint32_t seq  = 0;
        uint64_t round_trips = 0;
        long     iter = 0;

        uint64_t t0 = now_ns();
        for (;;) {
            size_t len = 0;

            build_frame(frame, dst, pf0->mac_addr, size, seq++);
            if (sfc7120_tx(pf0, frame, size) != 0) {
                fprintf(stderr, "sfcbench: ktput tx PF0 failed (size %zu)\n",
                        size);
                return -1;
            }
            if (sfc7120_rx(pf1, rxbuf, &len) != 0) {
                fprintf(stderr, "sfcbench: ktput PF1 RX timeout (size %zu)\n",
                        size);
                return -1;
            }
            swap_mac(rxbuf);
            if (sfc7120_tx(pf1, rxbuf, len) != 0) {
                fprintf(stderr, "sfcbench: ktput tx PF1 echo failed\n");
                return -1;
            }
            if (sfc7120_rx(pf0, rxbuf, &len) != 0) {
                fprintf(stderr, "sfcbench: ktput PF0 RX timeout (size %zu)\n",
                        size);
                return -1;
            }
            round_trips++;

            /* Check the clock occasionally to amortize now_ns(). */
            if ((iter++ & 0xff) == 0 && now_ns() - t0 >= TPUT_NS)
                break;
        }
        uint64_t elapsed = now_ns() - t0;

        double secs = (double)elapsed / 1e9;
        double rts  = (double)round_trips / secs;
        double pps  = rts;                 /* one-directional packet rate */
        double gbps = (rts * 2.0 * (double)size * 8.0) / 1e9;  /* both dirs */

        printf("%6zu  %14.0f  %14.0f  %12.3f\n", size, rts, pps, gbps);
    }

    return 0;
}

/* -------------------------------------------------------------------- main */

/*
 * run_path_session — one attach session for one path: init both PFs, run the
 * requested modes, destroy. Each path gets its own session because direct-path
 * use makes destroy sync the EVQ read ptr / ring heads back to the kernel,
 * and the kernel ioctls poll at the kernel's copy of those pointers — mixing
 * paths inside one session would leave one side reading a stale pointer.
 */
static int
run_path_session(const bench_path_t *p, int do_lat, int do_tput)
{
    int rc = 0;

    sfc7120_if_t pf0 = { .dev_path = DEV_PF0 };
    sfc7120_if_t pf1 = { .dev_path = DEV_PF1 };

    if (sfc7120_init(&pf0) != 0) {
        fprintf(stderr, "sfcbench: init PF0 (%s) failed\n", DEV_PF0);
        return -1;
    }
    if (sfc7120_init(&pf1) != 0) {
        fprintf(stderr, "sfcbench: init PF1 (%s) failed — a single process may "
                "not be able to attach both PFs; see the plan's two-process "
                "fallback\n", DEV_PF1);
        sfc7120_destroy(&pf0);
        return -1;
    }

    printf("\nsfcbench: path %s\n", p->name);
    printf("sfcbench: PF0 %s MAC %02x:%02x:%02x:%02x:%02x:%02x\n", DEV_PF0,
           pf0.mac_addr[0], pf0.mac_addr[1], pf0.mac_addr[2],
           pf0.mac_addr[3], pf0.mac_addr[4], pf0.mac_addr[5]);
    printf("sfcbench: PF1 %s MAC %02x:%02x:%02x:%02x:%02x:%02x\n", DEV_PF1,
           pf1.mac_addr[0], pf1.mac_addr[1], pf1.mac_addr[2],
           pf1.mac_addr[3], pf1.mac_addr[4], pf1.mac_addr[5]);

    if (do_lat && run_latency(p, &pf0, &pf1) != 0)
        rc = -1;
    if (rc == 0 && do_tput) {
        if (p->direct)
            rc = run_throughput(&pf0, &pf1);
        else
            rc = run_throughput_kernel(&pf0, &pf1);
    }

    sfc7120_destroy(&pf1);
    sfc7120_destroy(&pf0);
    return rc;
}

int
main(int argc, char **argv)
{
    const char *mode = (argc >= 2) ? argv[1] : "both";
    const char *path = (argc >= 3) ? argv[2] : "both";
    int do_lat    = (strcmp(mode, "lat")    == 0) || (strcmp(mode, "both") == 0);
    int do_tput   = (strcmp(mode, "tput")   == 0) || (strcmp(mode, "both") == 0);
    int do_direct = (strcmp(path, "direct") == 0) || (strcmp(path, "both") == 0);
    int do_kernel = (strcmp(path, "kernel") == 0) || (strcmp(path, "both") == 0);
    int rc = 0;

    if ((!do_lat && !do_tput) || (!do_direct && !do_kernel)) {
        fprintf(stderr, "usage: %s [lat|tput|both] [direct|kernel|both]\n",
                argv[0]);
        return 2;
    }

    /* Kernel first so the direct path's numbers print last, next to the
     * comparison target. Each path runs in its own attach session. */
    if (do_kernel && run_path_session(&g_path_kernel, do_lat, do_tput) != 0)
        rc = 1;
    if (rc == 0 && do_direct &&
        run_path_session(&g_path_direct, do_lat, do_tput) != 0)
        rc = 1;

    printf("\nsfcbench: done (%s)\n", rc == 0 ? "ok" : "FAILED");
    return rc;
}
