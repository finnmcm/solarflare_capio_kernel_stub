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

int
main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s tx|rx\n", argv[0]);
        return 2;
    }
    if (strcmp(argv[1], "tx") == 0)
        return run_producer();
    if (strcmp(argv[1], "rx") == 0)
        return run_consumer();

    fprintf(stderr, "usage: %s tx|rx\n", argv[0]);
    return 2;
}
