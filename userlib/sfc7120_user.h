#ifndef SFC7120_USER_H
#define SFC7120_USER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/ioccom.h>

#include "../sfc7120_uapi.h"
#include "modmap.h"
#include "capio.h"   /* CAPIO_ATTACH / CAPIO_GOODBYE */

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif

#define DEVSFC7120 "/dev/sfc7120pol1" // our actual module in tree
#define DEVMODMAP  "/dev/modmap"

/* EVQ_RPTR doorbell: bits 0..14 are the read pointer (mod 2^15).
 * Mirrors SFC7120_EVQ_RPTR_MASK in ../sfc7120_mmio.h (kernel-only header). */
#define SFC7120_EVQ_RPTR_MASK 0x7fffu

/* EF10 low-latency firmware prepends a 14-byte metadata prefix to each RX
 * packet in the buffer; frame length is the LE uint16 at prefix offset +8.
 * Mirrors SFC7120_EF10_RX_PREFIX_LEN in ../sfc7120.c (kernel-only). */
#define SFC7120_EF10_RX_PREFIX_LEN 14

/*
 * sfc7120_poll event types — decoded from the EV_CODE nibble (bits 63:60)
 * exactly as the kernel ioctl handlers do: RX_EV=0, TX_EV=2; everything
 * else (DRIVER_EV=5, MCDI_EV=12, ...) is consumed but reported as OTHER.
 */
typedef enum {
    SFC7120_EV_RX,
    SFC7120_EV_TX,
    SFC7120_EV_OTHER,
} sfc7120_ev_type_t;

typedef struct sfc7120_ev {
    sfc7120_ev_type_t type;
    uint16_t tx_desc_idx;  /* TX_EV: completed descriptor index (ev & 0xffff) */
    uint16_t rx_bytes;     /* RX_EV: byte count incl. 14-byte prefix (ev & 0x3fff) */
    uint64_t raw;          /* raw 8-byte event word */
} sfc7120_ev_t;

typedef struct sfc7120_if { // state struct, everything we need from kernel stub
    const char *dev_path;   /* device node to open; NULL → DEVSFC7120 default */

    int     fd;
    int     modmap_fd;

    cap_req_t cap_req;

    uint8_t mac_addr[6];

    void   *tx_buffer;
    void   *rx_buffer;

    /* Phase C: direct data-path resources */
    void   *tx_desc_ring;          /* 4 KB mapped TX descriptor ring */
    void   *rx_desc_ring;          /* 4 KB mapped RX descriptor ring */
    void   *evq_ring;              /* 4 KB mapped data-EVQ (instance 1) ring */

    user_slice_def_t *mmio_slices; /* bounded caps, indexed by sfc7120_mmio_slice_idx_t */
    size_t            mmio_slices_len;

    sfc7120_vi_info_req_t vi_info; /* paddrs, vi_base, instances, counts, heads */
    bool     vi_info_valid;        /* GET_VI_INFO succeeded — gates rptr sync */
    uint32_t evq_read_ptr;         /* our data-EVQ read ptr (seeded from vi_info) */
    bool     used_poll;            /* sfc7120_poll ran this session — only then
                                    * is evq_read_ptr ours to sync back. In the
                                    * ioctl-only path the kernel owns the EVQ
                                    * pointer and must NOT be clobbered. */
    uint32_t rx_head;              /* our RX slot to consume + re-post next;
                                    * seeded from vi_info.rx_head (direct RX) */
    uint32_t tx_head;              /* our TX producer slot; seeded from
                                    * vi_info.tx_head (direct TX) */

    /* Per-region mapping record for munmap at destroy; indexed by
     * sfc7120_vm_map_type_t. For the sliced MMIO region, base is the
     * perm-stripped (no LOAD/STORE) full-region cap — a munmap token only. */
    struct {
        void   *base;
        size_t  len;
    } region_maps[SFC7120_REGION_COUNT];

    void   *cap_token;  /* malloc'd page — raw material for CAPIO_ATTACH seal */
} sfc7120_if_t;

int  sfc7120_init(sfc7120_if_t *sfc);
void sfc7120_destroy(sfc7120_if_t *sfc);
int  sfc7120_tx(sfc7120_if_t *sfc, const void *buf, size_t len);
int  sfc7120_tx_direct(sfc7120_if_t *sfc, const void *buf, size_t len);
int  sfc7120_rx(sfc7120_if_t *sfc, void *buf, size_t *len_out);
int  sfc7120_rx_direct(sfc7120_if_t *sfc, void *buf, size_t *len_out);
int  sfc7120_poll(sfc7120_if_t *sfc, sfc7120_ev_t *evs, int max_evs);

#endif /* SFC7120_USER_H */
