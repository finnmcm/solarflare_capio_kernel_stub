#ifndef SFC7120_UAPI_H
#define SFC7120_UAPI_H

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif
#include <sys/ioccom.h>

/* Queue geometry — must match the kernel ring allocations. */
#define SFC7120_NUM_TX_DESC    512
#define SFC7120_NUM_RX_DESC    512
#define SFC7120_NUM_EVQ_ENTRY  512
#define SFC7120_TX_BUFFER_SIZE 2048
#define SFC7120_RX_BUFFER_SIZE 2048

#define SFC7120_EVQ_ENTRY_SIZE 8
#define SFC7120_TX_DESC_SIZE   8
#define SFC7120_RX_DESC_SIZE   8

/* Memory region indices — must match sfc7120_vm_map_type_t in sfc7120.h. */
typedef enum {
    SFC7120_TX_BUFFER,        /* 0 — DMA TX packet buffer (1 MB) */
    SFC7120_RX_BUFFER,        /* 1 — DMA RX packet buffer (1 MB) */
    SFC7120_MMIO_REGION,      /* 2 — BAR2, sliced per-register */
    SFC7120_TX_DESC_RING,     /* 3 — TX descriptor ring (4 KB) */
    SFC7120_RX_DESC_RING,     /* 4 — RX descriptor ring (4 KB) */
    SFC7120_EVQ_RING,         /* 5 — data EVQ ring, instance 1 (4 KB) */
    SFC7120_REGION_COUNT
} sfc7120_vm_map_type_t;

/*
 * MMIO slice indices — must match the order of sfc7120_reg_slices[] in
 * sfc7120_tables.c. Userspace receives the slice capabilities as an array
 * in manifest order; the name field copied out is a kernel pointer and
 * cannot be dereferenced, so index is the lookup key.
 */
typedef enum sfc7120_mmio_slice_idx {
    SFC7120_SLICE_MC_DOORBELL = 0,     /* 0x0200 — MCDI kick (kernel use) */
    SFC7120_SLICE_DATA_EVQ_RPTR_DBL,   /* 0x2400 — data EVQ (instance 1) ack */
    SFC7120_SLICE_RX_DESC_DBL,         /* 0x0830 — RX producer push */
    SFC7120_SLICE_TX_DESC_DBL,         /* 0x0a10, 12 B (incl. +8 WPTR push) */
    SFC7120_SLICE_HW_REV_ID,           /* 0x0000 — RO liveness check */
    SFC7120_SLICE_COUNT
} sfc7120_mmio_slice_idx_t;

/* Every CAPIO IOCTL struct must begin with user_cap + sealed_cap. */

typedef struct sfc7120_mac_req {
    void * __capability user_cap;
    void * __capability sealed_cap;

    uint8_t mac_addr[6];
} sfc7120_mac_req_t;

typedef struct sfc7120_tx_req {
    void * __capability    user_cap;
    void * __capability    sealed_cap;

    uint8_t * __capability tx_buf_addr;
    size_t                 length;
    uint8_t                status;
} sfc7120_tx_req_t;

/*
 * sfc7120_rx_req_t — kept deliberately small.
 *
 * Do NOT embed a large inline array here. _IOWR encodes struct size in a
 * 13-bit field (IOCPARM_MAX = 8192 bytes). A struct over that limit causes
 * kern_ioctl to copy fewer bytes than the driver writes back, producing a
 * kernel stack overflow or silent data corruption. raw_buffer is a pointer
 * to a userspace-allocated buffer — the received frame is copyout'd there
 * directly.
 */
typedef struct sfc7120_rx_req {
    void * __capability user_cap;
    void * __capability sealed_cap;

    uint8_t *raw_buffer;
    size_t   length_received;

    uint8_t status;
    uint8_t error;
} sfc7120_rx_req_t;

/*
 * sfc7120_vi_info_req_t — VI geometry handed to userspace for the direct
 * (phase C+) data path, following ef_vi's resource-manager model. The kernel
 * fills the DMA bus addresses, VI base, queue instance numbers, ring counts,
 * and current head pointers so the process can drive the rings itself. All
 * plain scalars — no inline arrays — so it stays well under IOCPARM_MAX.
 */
typedef struct sfc7120_vi_info_req {
    void * __capability user_cap;
    void * __capability sealed_cap;

    uint64_t tx_buffer_paddr;   /* bus addr of TX packet buffer slot 0 */
    uint64_t rx_buffer_paddr;   /* bus addr of RX packet buffer slot 0 */

    uint32_t vi_base;           /* absolute base VI for this function */
    uint32_t evq_instance;      /* data EVQ instance (1) */
    uint32_t rxq_instance;      /* RXQ instance (0) */
    uint32_t txq_instance;      /* TXQ instance (0) */

    uint32_t num_tx_desc;       /* 512 */
    uint32_t num_rx_desc;       /* 512 */
    uint32_t num_evq_entry;     /* 512 */

    uint32_t tx_head;           /* current kernel TX producer index */
    uint32_t rx_head;           /* current kernel RX post index */
    uint32_t evq_read_ptr;      /* current data-EVQ read pointer */
} sfc7120_vi_info_req_t;

/*
 * sfc7120_evq_sync_req_t — userspace reports its final data-path pointers back
 * to the kernel at teardown (phase C+). The direct TX/RX/poll paths advance
 * their own copies of the EVQ read pointer and the TX/RX ring heads, which the
 * kernel never sees; without this sync the next run's GET_VI_INFO hands out
 * stale seeds and the data path desyncs from the NIC (whose queue state
 * persists across opens — INIT_*Q runs once per module load, not per open).
 */
typedef struct sfc7120_evq_sync_req {
    void * __capability user_cap;
    void * __capability sealed_cap;

    uint32_t evq_read_ptr;      /* userspace's final data-EVQ read pointer */
    uint32_t tx_head;           /* userspace's final TX producer index */
    uint32_t rx_head;           /* userspace's final RX consume/post index */
} sfc7120_evq_sync_req_t;

/* IOCTLs — 'S' group matches the kernel definition. */
#define SFC7120_RX           _IOWR('S', 1, sfc7120_rx_req_t)
#define SFC7120_TX           _IOWR('S', 2, sfc7120_tx_req_t)
#define SFC7120_GET_MAC      _IOWR('S', 3, sfc7120_mac_req_t)
#define SFC7120_GET_VI_INFO  _IOWR('S', 4, sfc7120_vi_info_req_t)
#define SFC7120_SET_EVQ_RPTR _IOWR('S', 5, sfc7120_evq_sync_req_t)

#endif /* SFC7120_UAPI_H */
