#ifndef SFC7120_POL_H
#define SFC7120_POL_H

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <sys/bus.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/ioccom.h>
#include <sys/conf.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/ucred.h>
#include <sys/rwlock.h>
#include <sys/taskqueue.h>
#include <sys/selinfo.h>
#include <sys/poll.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/param.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <cheri/cheric.h>
#include <cheri/cheri.h>

#include "capio.h"
#include "sfc7120_uapi.h"

/*
 * Solarflare SFN7000-series (Huntington / EF10) is event-driven, not
 * descriptor-DD-driven. The classic queue layout:
 *
 *   - One Event Queue (EVQ) per channel: hardware writes events here
 *   - One TX queue per channel: descriptors point at TX packet buffers
 *   - One RX queue per channel: descriptors point at RX packet buffers
 *
 * For a first stub we expose a single channel (q0) with one EVQ + TX + RX,
 * mirroring the e1000 stub's 3-region split. Multi-queue (mlx5pol-style) can
 * be added later by extending the region enum and softc.
 */

/* Per-buffer metadata, mirrors the e1000 pattern. */
typedef struct sfc7120_rx_buf_info {
    uint8_t   *buf;
    bus_addr_t paddr;
} sfc7120_rx_buf_info_t;

typedef struct sfc7120_tx_buf_info {
    uint8_t   *buf;
    bus_addr_t paddr;
} sfc7120_tx_buf_info_t;

/*
 * Per-instance software context.
 *
 * MANDATORY: dev MUST be field 0 and capio_sc MUST be field 1.
 * capio.c casts (void *)sc -> (capio_softc_header_t *) and indexes
 * directly. Anything before dev or between dev and capio_sc breaks every
 * cast and panics the kernel.
 */
typedef struct sfc7120_softc {
    device_t        dev;        /* MUST be first */
    capio_softc_t   capio_sc;   /* MUST be second */

    shared_mem_region_t smem[SFC7120_REGION_COUNT];

    /* PCI / BAR resources */
    struct resource    *mem_resource;       /* BAR0 — function MMIO window */
    int                 mem_res_id;
    bus_space_tag_t     mem_bus_tag;
    bus_space_handle_t  mem_bsh;

    uint8_t             mac_addr[6];

    /* Interrupt resources (MSI-X is the EF10 norm; legacy fallback omitted) */
    struct resource    *irq_resource;
    int                 irq_res_id;
    void               *irq_handle;

    /* Event queue (EVQ) — DMA buffer of 64-bit events */
    void               *evq_ring;
    bus_addr_t          evq_ring_paddr;
    bus_dma_tag_t       evq_dtag;
    bus_dmamap_t        evq_dmamap;
    int                 evq_read_ptr;

    /* TX resources */
    void               *tx_desc_ring;
    bus_addr_t          tx_desc_ring_paddr;
    bus_dma_tag_t       tx_desc_dtag;
    bus_dmamap_t        tx_desc_dmamap;
    sfc7120_tx_buf_info_t *tx_buf_infos;
    void               *tx_buffer;
    bus_addr_t          tx_buffer_paddr;
    bus_dma_tag_t       tx_buffer_dtag;
    bus_dmamap_t        tx_buffer_dmamap;
    bool                tx_buffer_mapped;
    int                 tx_head;
    int                 tx_tail;
    int                 tx_descriptors_free;

    /* RX resources */
    volatile bool       rx_received;
    volatile bool       rx_teardown;
    struct task         rx_task;
    struct taskqueue   *rx_taskqueue;
    struct mtx          rx_mtx;
    struct selinfo      selinfo;

    void               *rx_desc_ring;
    bus_addr_t          rx_desc_ring_paddr;
    bus_dma_tag_t       rx_desc_dtag;
    bus_dmamap_t        rx_desc_dmamap;

    sfc7120_rx_buf_info_t *rx_buf_infos;
    void               *rx_buffer;
    bus_addr_t          rx_buffer_paddr;
    bus_dma_tag_t       rx_buffer_dtag;
    bus_dmamap_t        rx_buffer_dmamap;
    bool                rx_buffer_mapped;
    int                 rx_head;

    /* Lifecycle */
    bool                device_attached;
    struct cdev        *cdev;
    struct mtx          sc_mtx;
    bool                dying;
    bool                mapped;

    /* MCDI: DMA-resident mailbox + bookkeeping for the MC firmware command
     * channel. Populated by sfc7120_mcdi_init() during hw_init. The mailbox
     * MUST be 256-byte aligned (EF10 doorbell recovery requirement, sfxge
     * bug24769) — sfc7120_mcdi.c enforces that via bus_dma_tag_create. */
    void               *mcdi_buf;
    bus_addr_t          mcdi_buf_paddr;
    bus_dma_tag_t       mcdi_dtag;
    bus_dmamap_t        mcdi_dmamap;
    struct mtx          mcdi_mtx;
    uint8_t             mcdi_seq;        /* 4-bit sequence, increments per request */
    bool                mcdi_new_epoch;  /* next request starts a new epoch */
    bool                mcdi_initialized;

    //finn: this is for failure preventions later
    uint32_t mcdi_prev_reboot_status;

    /* DRV_ATTACH / ALLOC_VIS results, populated during hw_init. */
    uint32_t            mcdi_func_flags;  /* MC_CMD_DRV_ATTACH_EXT_OUT_FUNC_FLAGS */
    uint32_t            vi_base;          /* base absolute VI for this function */
    uint32_t            vi_count;         /* VIs allocated to this function */

    /* GET_CAPABILITIES FLAGS1 cache. Populated by sfc7120_mcdi_dump_func_info.
     * Used downstream to gate optional MCDI flags (e.g. the PERMIT_SET_MAC_*
     * bit in VADAPTOR_ALLOC) that older firmware variants reject with EINVAL
     * when the cap isn't advertised. Mirrors sfxge's enc_* capability cache.*/
    uint32_t            mcdi_cap_flags1;
    bool                mcdi_caps_valid;

    /* GET_VERSION result (firmware build identifier). */
    uint32_t            fw_version[2];    /* hi/lo pair */
    bool                drv_attached;
    bool                vis_allocated;
    bool                evq_initialized;

    /* vadaptor, tx/rx init flags */
    bool vadaptor_allocated;
    bool rxq_initialized;
    bool txq_initialized;

    /* PHY capability cache — populated by sfc7120_mcdi_get_phy_cfg. */
    uint32_t            phy_supported_cap_mask;
    uint32_t            phy_adv_cap_mask;
    uint32_t            phy_media_type;

    /* Link state — updated by sfc7120_mcdi_get_link. */
    bool                link_up;
    bool                full_duplex;
    uint32_t            link_speed_mbps;
    uint32_t            link_fcntl;

    /* Bringup completion flags. */
    bool                mac_configured;
    bool                link_configured;

    /* RX filter (MC_CMD_FILTER_OP). Inserted after INIT_RXQ so matching RX
     * frames are steered to RX queue 0. The firmware-returned 64-bit handle
     * is needed to remove the filter on teardown. */
    uint64_t            rx_filter_handle;
    bool                rx_filter_inserted;

    bool                debug_reg_ops;
} sfc7120_softc_t;

/* IOCTLs defined in sfc7120_uapi.h (SFC7120_RX, SFC7120_TX, SFC7120_GET_MAC). */

#endif /* SFC7120_POL_H */
