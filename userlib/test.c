/*
 * test.c — dual-port sfc7120 userspace TX/RX smoke test (phase 1, ioctl path).
 *
 * PF0 and PF1 are cabled port-to-port via SFP+ DAC, so a frame TX'd on port 0
 * ingresses port 1. This test runs as two cooperating processes:
 *
 *   ./sfctest rx     consumer — opens /dev/sfc7120pol1 (PF1), blocks on RX
 *   ./sfctest tx     producer — opens /dev/sfc7120pol0 (PF0), sends frames
 *
 * Start the consumer first, then the producer. The producer addresses frames
 * to PF1's MAC (its own PF0 base MAC with the low octet +1, per the hardware
 * docs) so the kernel exact-DST_MAC RX filter on PF1 matches → RXQ 0.
 *
 * Not part of the kernel-bypass build — just a sanity check of the userlib API.
 */
#include "sfc7120_user.h"

#include <stdio.h>
#include <string.h>

#define DEV_PF0       "/dev/sfc7120pol0"
#define DEV_PF1       "/dev/sfc7120pol1"

#define TEST_PACKETS  4
#define ETHERTYPE     0x88B5   /* IEEE 802 local experimental */
#define FRAME_LEN     64       /* min Ethernet frame (sans FCS) */

static void
build_frame(uint8_t *frame, const uint8_t dst[6], const uint8_t src[6], int seq)
{
    memset(frame, 0, FRAME_LEN);
    memcpy(&frame[0], dst, 6);
    memcpy(&frame[6], src, 6);
    frame[12] = (ETHERTYPE >> 8) & 0xff;
    frame[13] = ETHERTYPE & 0xff;
    frame[14] = (uint8_t)seq;             /* payload marker */
    memset(&frame[15], 0xA5, FRAME_LEN - 15);
}

/*
 * dump_vi_state — print the Phase C mappings so hardware verification is
 * observable: VI geometry from GET_VI_INFO, the mapped ring addresses, and
 * a liveness read of HW_REV_ID through its bounded slice capability.
 */
static void
dump_vi_state(const sfc7120_if_t *sfc)
{
    const sfc7120_vi_info_req_t *vi = &sfc->vi_info;

    printf("test: vi_info: tx_paddr=0x%016lx rx_paddr=0x%016lx\n",
           (unsigned long)vi->tx_buffer_paddr,
           (unsigned long)vi->rx_buffer_paddr);
    printf("test: vi_info: vi_base=%u evq=%u rxq=%u txq=%u "
           "ntx=%u nrx=%u nevq=%u tx_head=%u rx_head=%u evq_rptr=%u\n",
           vi->vi_base, vi->evq_instance, vi->rxq_instance, vi->txq_instance,
           vi->num_tx_desc, vi->num_rx_desc, vi->num_evq_entry,
           vi->tx_head, vi->rx_head, vi->evq_read_ptr);
    printf("test: rings: tx_desc=%p rx_desc=%p evq=%p (slices=%zu)\n",
           sfc->tx_desc_ring, sfc->rx_desc_ring, sfc->evq_ring,
           sfc->mmio_slices_len);
    /* The MMIO base is a munmap token: full-region bounds, but LOAD/STORE
     * stripped — %#p shows the perms so that's verifiable on hardware. */
    printf("test: mmio base cap (munmap token) = %#p\n",
           sfc->region_maps[SFC7120_MMIO_REGION].base);

    if (sfc->mmio_slices_len > SFC7120_SLICE_HW_REV_ID) {
        uint32_t rev = *(volatile uint32_t * __capability)
            sfc->mmio_slices[SFC7120_SLICE_HW_REV_ID].addr;
        printf("test: HW_REV_ID via slice cap = 0x%08x (%s)\n",
               rev, rev != 0 ? "ok" : "BAD — expected non-zero");
    }
}

static int
run_producer(void)
{
    sfc7120_if_t sfc = { .dev_path = DEV_PF0 };
    uint8_t      dst[6], frame[FRAME_LEN];

    if (sfc7120_init(&sfc) != 0) {
        fprintf(stderr, "test: producer init failed\n");
        return 1;
    }
    printf("test: producer up on %s, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
           DEV_PF0, sfc.mac_addr[0], sfc.mac_addr[1], sfc.mac_addr[2],
           sfc.mac_addr[3], sfc.mac_addr[4], sfc.mac_addr[5]);
    dump_vi_state(&sfc);

    /* PF1 MAC = PF0 base MAC + 1 in the low octet. */
    memcpy(dst, sfc.mac_addr, 6);
    dst[5] += 1;

    for (int i = 0; i < TEST_PACKETS; i++) {
        build_frame(frame, dst, sfc.mac_addr, i);
        if (sfc7120_tx(&sfc, frame, FRAME_LEN) != 0) {
            fprintf(stderr, "test: TX %d failed\n", i);
            sfc7120_destroy(&sfc);
            return 1;
        }
        printf("test: TX %d ok (%d bytes)\n", i, FRAME_LEN);
    }

    sfc7120_destroy(&sfc);
    printf("test: producer done\n");
    return 0;
}

/*
 * run_producer_direct — Phase E producer: transmits via sfc7120_tx_direct
 * (userspace posts the descriptor + rings the doorbell + polls the TX_EV,
 * kernel out of the data path) instead of the SFC7120_TX ioctl. Pair with
 * `sfctest rxd` on PF1 for a full direct-path dual-port test.
 */
static int
run_producer_direct(void)
{
    sfc7120_if_t sfc = { .dev_path = DEV_PF0 };
    uint8_t      dst[6], frame[FRAME_LEN];

    if (sfc7120_init(&sfc) != 0) {
        fprintf(stderr, "test: direct producer init failed\n");
        return 1;
    }
    printf("test: direct producer up on %s, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
           DEV_PF0, sfc.mac_addr[0], sfc.mac_addr[1], sfc.mac_addr[2],
           sfc.mac_addr[3], sfc.mac_addr[4], sfc.mac_addr[5]);
    dump_vi_state(&sfc);
    printf("test: direct TX from tx_head=%u\n", sfc.tx_head);

    /* PF1 MAC = PF0 base MAC + 1 in the low octet. */
    memcpy(dst, sfc.mac_addr, 6);
    dst[5] += 1;

    for (int i = 0; i < TEST_PACKETS; i++) {
        build_frame(frame, dst, sfc.mac_addr, i);
        if (sfc7120_tx_direct(&sfc, frame, FRAME_LEN) != 0) {
            fprintf(stderr, "test: direct TX %d failed\n", i);
            sfc7120_destroy(&sfc);
            return 1;
        }
        printf("test: direct TX %d ok (%d bytes)\n", i, FRAME_LEN);
    }

    sfc7120_destroy(&sfc);
    printf("test: direct producer done\n");
    return 0;
}

static int
run_consumer(void)
{
    sfc7120_if_t sfc = { .dev_path = DEV_PF1 };
    uint8_t      frame[SFC7120_RX_BUFFER_SIZE];

    if (sfc7120_init(&sfc) != 0) {
        fprintf(stderr, "test: consumer init failed\n");
        return 1;
    }
    printf("test: consumer up on %s, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
           DEV_PF1, sfc.mac_addr[0], sfc.mac_addr[1], sfc.mac_addr[2],
           sfc.mac_addr[3], sfc.mac_addr[4], sfc.mac_addr[5]);
    dump_vi_state(&sfc);

    for (int i = 0; i < TEST_PACKETS; i++) {
        size_t len = 0;
        if (sfc7120_rx(&sfc, frame, &len) != 0) {
            fprintf(stderr, "test: RX %d failed\n", i);
            break;
        }
        printf("test: RX %d ok (%zu bytes, payload marker 0x%02x)\n",
               i, len, len > 14 ? frame[14] : 0);
    }

    sfc7120_destroy(&sfc);
    printf("test: consumer done\n");
    return 0;
}

/*
 * run_consumer_direct — Phase D consumer: receives via sfc7120_rx_direct
 * (userspace consumes + re-posts the RX ring, kernel out of the data path)
 * instead of the SFC7120_RX ioctl. Verify against the kernel TX path:
 * run `sfctest tx` on PF0 as the producer; each frame should arrive here.
 */
static int
run_consumer_direct(void)
{
    sfc7120_if_t sfc = { .dev_path = DEV_PF1 };
    uint8_t      frame[SFC7120_RX_BUFFER_SIZE];

    if (sfc7120_init(&sfc) != 0) {
        fprintf(stderr, "test: direct consumer init failed\n");
        return 1;
    }
    printf("test: direct consumer up on %s, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
           DEV_PF1, sfc.mac_addr[0], sfc.mac_addr[1], sfc.mac_addr[2],
           sfc.mac_addr[3], sfc.mac_addr[4], sfc.mac_addr[5]);
    dump_vi_state(&sfc);
    printf("test: direct RX from rx_head=%u (start `sfctest tx` on PF0)\n",
           sfc.rx_head);

    for (int i = 0; i < TEST_PACKETS; i++) {
        size_t len = 0;
        if (sfc7120_rx_direct(&sfc, frame, &len) != 0) {
            fprintf(stderr, "test: direct RX %d failed\n", i);
            break;
        }
        printf("test: direct RX %d ok (%zu bytes, payload marker 0x%02x)\n",
               i, len, len > 14 ? frame[14] : 0);
    }

    sfc7120_destroy(&sfc);
    printf("test: direct consumer done\n");
    return 0;
}

/*
 * run_poller — Phase C consumer: drains the data EVQ directly via
 * sfc7120_poll (zero syscalls in the poll loop) instead of the SFC7120_RX
 * ioctl. The kernel still owns descriptor posting; we only observe + ack
 * events. Run `sfctest tx` on PF0 as the producer — each 64-byte frame
 * should appear here as one RX_EV with rx_bytes = 64 + 14 (EF10 prefix).
 */
static int
run_poller(void)
{
    sfc7120_if_t sfc = { .dev_path = DEV_PF1 };
    sfc7120_ev_t evs[8];
    int          rx_seen = 0;
    long         tries;

    if (sfc7120_init(&sfc) != 0) {
        fprintf(stderr, "test: poller init failed\n");
        return 1;
    }
    printf("test: poller up on %s, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
           DEV_PF1, sfc.mac_addr[0], sfc.mac_addr[1], sfc.mac_addr[2],
           sfc.mac_addr[3], sfc.mac_addr[4], sfc.mac_addr[5]);
    dump_vi_state(&sfc);
    printf("test: polling data EVQ from read_ptr=%u (start `sfctest tx` "
           "on PF0)\n", sfc.evq_read_ptr);

    /* Generous budget — the oracle spins 100000 tries per packet; we wait
     * for a human to launch the producer in another window. */
    for (tries = 0; tries < 2000000000L && rx_seen < TEST_PACKETS; tries++) {
        int n = sfc7120_poll(&sfc, evs, 8);
        if (n < 0) {
            fprintf(stderr, "test: sfc7120_poll failed\n");
            break;
        }
        for (int i = 0; i < n; i++) {
            const char *name =
                evs[i].type == SFC7120_EV_RX ? "RX_EV" :
                evs[i].type == SFC7120_EV_TX ? "TX_EV" : "OTHER";
            printf("test: ev %s raw=0x%016lx rx_bytes=%u tx_idx=%u "
                   "read_ptr now %u\n",
                   name, (unsigned long)evs[i].raw, evs[i].rx_bytes,
                   evs[i].tx_desc_idx, sfc.evq_read_ptr);
            if (evs[i].type == SFC7120_EV_RX)
                rx_seen++;
        }
    }

    printf("test: poller saw %d/%d RX events (final read_ptr=%u)\n",
           rx_seen, TEST_PACKETS, sfc.evq_read_ptr);
    sfc7120_destroy(&sfc);
    printf("test: poller done\n");
    return rx_seen == TEST_PACKETS ? 0 : 1;
}

int
main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s tx|txd|rx|rxd|poll\n", argv[0]);
        return 2;
    }
    if (strcmp(argv[1], "tx") == 0)
        return run_producer();
    if (strcmp(argv[1], "txd") == 0)
        return run_producer_direct();
    if (strcmp(argv[1], "rx") == 0)
        return run_consumer();
    if (strcmp(argv[1], "rxd") == 0)
        return run_consumer_direct();
    if (strcmp(argv[1], "poll") == 0)
        return run_poller();

    fprintf(stderr, "usage: %s tx|txd|rx|rxd|poll\n", argv[0]);
    return 2;
}
