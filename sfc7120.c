/*
 * sfc7120.c — out-of-tree CAPIO kernel driver stub for the Solarflare
 * SFN7000-series (Huntington / EF10) NIC.
 *
 * This is a SKELETON. It compiles to a kernel module that registers a PCI
 * driver, exposes /dev/sfc7120pol, and wires up the CAPIO framework. The
 * real hardware bringup (MCDI, EVQ/RXQ/TXQ init, link config) is left as
 * TODO blocks. Use ~/CheriBsdE1000/e1000pol.c and
 * ~/cheri/cheribsd/sys/modules/mlx5pol/mlx5pol.c as reference for filling
 * in the bodies.
 */

#include "capio.h"
#include "sys/_stdint.h"
#include "sys/bus.h"
#include "sys/rman.h"
#include "sys/types.h"
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "sfc7120.h"
#include "sfc7120_mcdi.h"
#include "sfc7120_mmio.h"

#if CROSS_COMPILE
#include "../CheriModmap/modmap_api.h"
#else
#include "modmap_api.h"
#endif

MALLOC_DECLARE(M_DEVBUF);
MALLOC_DEFINE(M_DEVBUF, "sfc7120pol", "Solarflare 7120 Device Buffer");

static d_open_t      sfc7120_open;
static d_close_t     sfc7120_close;
static d_ioctl_t     sfc7120_ioctl;
static d_poll_t      sfc7120_poll;
static is_dying_t    sfc7120_is_dying;
static get_buffer_size_t sfc7120_get_buffer_size;

static struct cdevsw sfc7120_cdevsw = {
    .d_name    = "sfc7120pol",
    .d_version = D_VERSION,
    .d_open    = sfc7120_open,
    .d_close   = sfc7120_close,
    .d_ioctl   = sfc7120_ioctl,    /* overwritten by make_dev_capio */
    .d_poll    = sfc7120_poll,
};

static capio_ops_t sfc7120_capio_ops = {
    .ioctl           = sfc7120_ioctl,
    .get_buffer_size = sfc7120_get_buffer_size,
    .is_dying        = sfc7120_is_dying,
};

static int  sfc7120_fbsd_probe(device_t dev);
static int  sfc7120_fbsd_attach(device_t dev);
static int  sfc7120_fbsd_detach(device_t dev);
static void sfc7120_interrupt_handler(void *arg);
static int  sfc7120_intr_setup(sfc7120_softc_t *sc);
static void sfc7120_intr_teardown(sfc7120_softc_t *sc);
static void sfc7120_rx_task_handler(void *context, int pending);

/* EF10 RX prefix: 14-byte pseudo-header prepended when INIT_RXQ FLAG_PREFIX
 * is set (our flags=0x300 includes bit 8). Layout per ef10_rx.c:737:
 *   +0x00  hash32 (LE)
 *   +0x04  outer VLAN (BE)
 *   +0x06  inner VLAN (BE)
 *   +0x08  packet length (LE uint16, excl. prefix; 0 = cut-through mode)
 *   +0x0a  MAC timestamp (LE) */
#define SFC7120_EF10_RX_PREFIX_LEN  14

#define SFC7120_LOCK_INIT(sc) \
    mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), \
             "sfc7120 softc lock", MTX_DEF)
#define SFC7120_LOCK(sc)         mtx_lock(&(sc)->sc_mtx)
#define SFC7120_UNLOCK(sc)       mtx_unlock(&(sc)->sc_mtx)
#define SFC7120_LOCK_DESTROY(sc) mtx_destroy(&(sc)->sc_mtx)

#define SFC7120_RX_LOCK_INIT(sc) \
    mtx_init(&(sc)->rx_mtx, device_get_nameunit((sc)->dev), \
             "sfc7120 rx lock", MTX_DEF)
#define SFC7120_RX_LOCK(sc)         mtx_lock(&(sc)->rx_mtx)
#define SFC7120_RX_UNLOCK(sc)       mtx_unlock(&(sc)->rx_mtx)
#define SFC7120_RX_LOCK_DESTROY(sc) mtx_destroy(&(sc)->rx_mtx)

/*
 * Solarflare PCI device IDs.
 *
 * Vendor 0x1924 = Solarflare Communications (later acquired by Xilinx /
 * AMD). The SFN7xxx series uses the Huntington EF10 controller and reports
 * 0x0903 (PF) / 0x1903 (VF). Adjust this table to whatever board you have
 * physically in front of you — confirm with `pciconf -lv` on CheriBSD.
 */
static const struct {
    uint16_t    vendor;
    uint16_t    device;
    const char *desc;
} sfc7120_fbsd_devs[] = {
    { 0x1924, 0x0903, "AMD Solarflare SFC9120 10G Ethernet Controller" },
    { 0x1924, 0x0903, "AMD Solarflare SFC9120 10G Ethernet Controller" },
};
 
static const size_t sfc7120_dev_count =
    sizeof(sfc7120_fbsd_devs) / sizeof(sfc7120_fbsd_devs[0]);

/* ---------------------------------------------------------------------- */
/* CAPIO vtable                                                           */
/* ---------------------------------------------------------------------- */

static size_t
sfc7120_get_buffer_size(void *arg, int type)
{
    sfc7120_softc_t *sc = (sfc7120_softc_t *)arg;
    sfc7120_vm_map_type_t map_type = (sfc7120_vm_map_type_t)type;
    size_t buffer_size = 0;

    switch (map_type) {
    case SFC7120_TX_BUFFER:
        return SFC7120_NUM_TX_DESC * SFC7120_TX_BUFFER_SIZE;
    case SFC7120_RX_BUFFER:
        return SFC7120_NUM_RX_DESC * SFC7120_RX_BUFFER_SIZE;
    case SFC7120_MMIO_REGION:
        SFC7120_LOCK(sc);
        buffer_size = rman_get_size(sc->mem_resource);
        SFC7120_UNLOCK(sc);
        return buffer_size;
    case SFC7120_TX_DESC_RING:
        return SFC7120_NUM_TX_DESC * SFC7120_TX_DESC_SIZE;
    case SFC7120_RX_DESC_RING:
        return SFC7120_NUM_RX_DESC * SFC7120_RX_DESC_SIZE;
    case SFC7120_EVQ_RING:
        return SFC7120_NUM_EVQ_ENTRY * SFC7120_EVQ_ENTRY_SIZE;
    default:
        return 0;
    }
}

static bool
sfc7120_is_dying(void *void_sc)
{
    sfc7120_softc_t *sc = (sfc7120_softc_t *)void_sc;
    bool res;

    SFC7120_LOCK(sc);
    res = sc->dying;
    SFC7120_UNLOCK(sc);
    return res;
}

/* ---------------------------------------------------------------------- */
/* DMA helpers                                                            */
/* ---------------------------------------------------------------------- */

static void
sfc7120_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
    bus_addr_t *out = arg;
    *out = (error != 0 || nseg < 1) ? 0 : segs[0].ds_addr;
}

/* Allocate one DMA-coherent buffer: tag → alloc → load → return paddr.
 * On failure, cleans up whatever it managed to allocate and returns errno. */
static int
sfc7120_alloc_dmabuf(device_t dev,
                     bus_dma_tag_t *dtag, bus_dmamap_t *dmamap,
                     void **vaddr, bus_addr_t *paddr,
                     bus_size_t size, bus_size_t align,
                     const char *name)
{
    int error;

    error = bus_dma_tag_create(bus_get_dma_tag(dev),
                               align, 0,
                               BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
                               NULL, NULL,
                               size, 1, size,
                               0, NULL, NULL, dtag);
    if (error != 0) {
        device_printf(dev, "%s: bus_dma_tag_create failed: %d\n", name, error);
        return error;
    }
  //arthur:  stores virtual addresses that we can actually use for the buffers using a freebsd macro, in the softc
    error = bus_dmamem_alloc(*dtag, vaddr,
                             BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO,
                             dmamap);
    if (error != 0) {
        device_printf(dev, "%s: bus_dmamem_alloc failed: %d\n", name, error);
        bus_dma_tag_destroy(*dtag);
        *dtag = NULL;
        return error;
    }

    *paddr = 0;
      // arthur: this does a translation from the virtual addresses to physical ones the NIC can DMA to/from
    error = bus_dmamap_load(*dtag, *dmamap, *vaddr, size,
                            sfc7120_dma_cb, paddr, BUS_DMA_NOWAIT);
    if (error != 0 || *paddr == 0) {
        device_printf(dev, "%s: bus_dmamap_load failed: %d\n", name, error);
        bus_dmamem_free(*dtag, *vaddr, *dmamap);
        bus_dma_tag_destroy(*dtag);
        *vaddr = NULL;
        *dtag  = NULL;
        return (error != 0) ? error : ENOMEM;
    }

    device_printf(dev, "DMA alloc: %-16s  vaddr=%p  paddr=0x%lx  size=0x%lx\n",
                  name, *vaddr, (unsigned long)*paddr, (unsigned long)size);
    return 0;
}

static void
sfc7120_free_dmabuf(bus_dma_tag_t *dtag, bus_dmamap_t *dmamap,
                    void **vaddr, bus_addr_t *paddr)
{
    if (*paddr != 0) {
        bus_dmamap_unload(*dtag, *dmamap);
        *paddr = 0;
    }
    if (*vaddr != NULL) {
        bus_dmamem_free(*dtag, *vaddr, *dmamap);
        *vaddr = NULL;
    }
    if (*dtag != NULL) {
        bus_dma_tag_destroy(*dtag);
        *dtag = NULL;
    }
}

static int
sfc7120_alloc_dma_resources(sfc7120_softc_t *sc)
{
    int        error;
    bus_size_t evq_size = SFC7120_NUM_EVQ_ENTRY * SFC7120_EVQ_ENTRY_SIZE;
    bus_size_t txd_size = SFC7120_NUM_TX_DESC   * SFC7120_TX_DESC_SIZE;
    bus_size_t rxd_size = SFC7120_NUM_RX_DESC   * SFC7120_RX_DESC_SIZE;
    bus_size_t txb_size = SFC7120_NUM_TX_DESC   * SFC7120_TX_BUFFER_SIZE;
    bus_size_t rxb_size = SFC7120_NUM_RX_DESC   * SFC7120_RX_BUFFER_SIZE;

    /* EVQ ring: NIC DMA-writes 64-bit events here. 4KB aligned (EF10 req). */
    error = sfc7120_alloc_dmabuf(sc->dev,
                                 &sc->evq_dtag, &sc->evq_dmamap,
                                 &sc->evq_ring, &sc->evq_ring_paddr,
                                 evq_size, 4096, "EVQ ring");
    if (error != 0)
        return error;

    /* Prime ring to 0xff. EF10 events use bit 63 as the "valid" marker; an
     * all-1s ring lets the driver tell "no event yet" from "stale event
     * with garbage payload" when polling. PREWRITE sync flushes the CPU
     * caches so the NIC sees the pattern before it starts producing. */
    memset(sc->evq_ring, 0xff, evq_size);
    bus_dmamap_sync(sc->evq_dtag, sc->evq_dmamap, BUS_DMASYNC_PREWRITE);

    /* Data EVQ ring (instance 1): same geometry as the control EVQ. Primed
     * to 0xff for the same "valid event" detection as EVQ 0. */
    error = sfc7120_alloc_dmabuf(sc->dev,
                                 &sc->data_evq_dtag, &sc->data_evq_dmamap,
                                 &sc->data_evq_ring, &sc->data_evq_ring_paddr,
                                 evq_size, 4096, "data EVQ ring");
    if (error != 0)
        goto fail_evq;
    memset(sc->data_evq_ring, 0xff, evq_size);
    bus_dmamap_sync(sc->data_evq_dtag, sc->data_evq_dmamap,
                    BUS_DMASYNC_PREWRITE);

    /* TX descriptor ring: driver writes outgoing packet descriptors here. */
    error = sfc7120_alloc_dmabuf(sc->dev,
                                 &sc->tx_desc_dtag, &sc->tx_desc_dmamap,
                                 &sc->tx_desc_ring, &sc->tx_desc_ring_paddr,
                                 txd_size, 4096, "TX desc ring");
    if (error != 0)
        goto fail_data_evq;

    /* RX descriptor ring: driver writes free buffer addresses here. */
    error = sfc7120_alloc_dmabuf(sc->dev,
                                 &sc->rx_desc_dtag, &sc->rx_desc_dmamap,
                                 &sc->rx_desc_ring, &sc->rx_desc_ring_paddr,
                                 rxd_size, 4096, "RX desc ring");
    if (error != 0)
        goto fail_txd;

    /* TX packet buffer: 128KB region userspace mmaps to write TX packets. */
    error = sfc7120_alloc_dmabuf(sc->dev,
                                 &sc->tx_buffer_dtag, &sc->tx_buffer_dmamap,
                                 &sc->tx_buffer, &sc->tx_buffer_paddr,
                                 txb_size, PAGE_SIZE, "TX buffer");
    if (error != 0)
        goto fail_rxd;

    /* RX packet buffer: 128KB region NIC DMAs received packets into. */
    error = sfc7120_alloc_dmabuf(sc->dev,
                                 &sc->rx_buffer_dtag, &sc->rx_buffer_dmamap,
                                 &sc->rx_buffer, &sc->rx_buffer_paddr,
                                 rxb_size, PAGE_SIZE, "RX buffer");
    if (error != 0)
        goto fail_txb;

    return 0;

fail_txb:
    sfc7120_free_dmabuf(&sc->tx_buffer_dtag, &sc->tx_buffer_dmamap,
                        &sc->tx_buffer, &sc->tx_buffer_paddr);
fail_rxd:
    sfc7120_free_dmabuf(&sc->rx_desc_dtag, &sc->rx_desc_dmamap,
                        &sc->rx_desc_ring, &sc->rx_desc_ring_paddr);
fail_txd:
    sfc7120_free_dmabuf(&sc->tx_desc_dtag, &sc->tx_desc_dmamap,
                        &sc->tx_desc_ring, &sc->tx_desc_ring_paddr);
fail_data_evq:
    sfc7120_free_dmabuf(&sc->data_evq_dtag, &sc->data_evq_dmamap,
                        &sc->data_evq_ring, &sc->data_evq_ring_paddr);
fail_evq:
    sfc7120_free_dmabuf(&sc->evq_dtag, &sc->evq_dmamap,
                        &sc->evq_ring, &sc->evq_ring_paddr);
    return error;
}

static void
sfc7120_free_dma_resources(sfc7120_softc_t *sc)
{
    sfc7120_free_dmabuf(&sc->rx_buffer_dtag, &sc->rx_buffer_dmamap,
                        &sc->rx_buffer, &sc->rx_buffer_paddr);
    sfc7120_free_dmabuf(&sc->tx_buffer_dtag, &sc->tx_buffer_dmamap,
                        &sc->tx_buffer, &sc->tx_buffer_paddr);
    sfc7120_free_dmabuf(&sc->rx_desc_dtag, &sc->rx_desc_dmamap,
                        &sc->rx_desc_ring, &sc->rx_desc_ring_paddr);
    sfc7120_free_dmabuf(&sc->tx_desc_dtag, &sc->tx_desc_dmamap,
                        &sc->tx_desc_ring, &sc->tx_desc_ring_paddr);
    sfc7120_free_dmabuf(&sc->data_evq_dtag, &sc->data_evq_dmamap,
                        &sc->data_evq_ring, &sc->data_evq_ring_paddr);
    sfc7120_free_dmabuf(&sc->evq_dtag, &sc->evq_dmamap,
                        &sc->evq_ring, &sc->evq_ring_paddr);
}

/* ---------------------------------------------------------------------- */
/* Hardware bringup. Stops here at MCDI + identity queries — EVQ/RXQ/TXQ  */
/* init needs DMA rings, so it lives in a future sfc7120_hw_start() that  */
/* runs after sfc7120_alloc_dma_resources(). The MCDI plumbing itself is  */
/* in sfc7120_mcdi.c.                                                     */
/* ---------------------------------------------------------------------- */

/*
 * sfc7120_post_rx_buffers — seed the RX descriptor ring at attach time.
 *
 * After INIT_RXQ the firmware knows the ring exists but the ring itself is
 * all zeros — the NIC has nowhere to put incoming packets. This function
 * writes one 8-byte descriptor per slot telling the NIC the physical address
 * and capacity of each buffer, then writes the RX doorbell to hand those
 * slots to the hardware.
 *
 * We post NUM_RX_DESC-1 (511) slots rather than all 512. Posting all 512
 * would wrap the write pointer back to 0, which the hardware reads as
 * "ring empty." Leaving one slot unposted keeps the pointer at 511 which
 * is unambiguous.
 *
 * After attach, sc->rx_head = 0. Each time the RX ioctl consumes a packet
 * it will re-post that descriptor and advance rx_head so the ring stays
 * topped up.
 */
static void
sfc7120_post_rx_buffers(sfc7120_softc_t *sc)
{
    volatile uint64_t *ring = (volatile uint64_t *)sc->rx_desc_ring;

    /*
     * Fill the RX descriptor ring so the NIC knows where to DMA incoming
     * packets. Each 8-byte descriptor carries:
     *   bits[61:48] — buffer capacity (BYTE_CNT), always SFC7120_RX_BUFFER_SIZE
     *   bits[47:32] — upper 16 bits of the slot's physical address
     *   bits[31:0]  — lower 32 bits of the slot's physical address
     *
     * We post NUM_RX_DESC-1 slots, leaving one empty. If we filled all 512
     * the write pointer would wrap back to 0, which looks identical to an
     * empty ring. Leaving one slot unposted keeps the pointer unambiguous.
     */
    int npost = SFC7120_NUM_RX_DESC - 1;
    for (int i = 0; i < npost; i++) {
        bus_addr_t paddr = sc->rx_buffer_paddr + i * SFC7120_RX_BUFFER_SIZE;
        uint64_t desc =
            ((uint64_t)(SFC7120_RX_BUFFER_SIZE & 0x3fff) << 48) |
            ((uint64_t)((paddr >> 32) & 0xffff)           << 32) |
            ((uint64_t)(paddr & 0xffffffff));
        ring[i] = desc;
    }

    /* Flush the descriptor writes to DMA memory before poking the doorbell. */
    bus_dmamap_sync(sc->rx_desc_dtag, sc->rx_desc_dmamap,
                    BUS_DMASYNC_PREWRITE);

    /* Tell the NIC: slots 0..(npost-1) are ready. */
    SFC7120_WRITE_REG(sc, SFC7120_REG_DATA_RX_DESC_DBL, (uint32_t)npost);
    sc->rx_head = 0;

    device_printf(sc->dev, "RX: posted %d buffer descriptors\n", npost);
}

static int
sfc7120_hw_init(sfc7120_softc_t *sc)
{
    int error;


    error = sfc7120_mcdi_init(sc);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: mcdi_init failed: %d\n", error);
        goto fail;
    }


    error = sfc7120_mcdi_get_version(sc);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: GET_VERSION failed: %d\n", error);
        goto fail_mcdi;
    }


    error = sfc7120_mcdi_drv_attach(sc);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: DRV_ATTACH failed: %d\n", error);
        goto fail_mcdi;
    }


    /* Drain/clear any pending MC firmware assertion. Mirrors sfxge
     * ef10_nic.c:1981-1991. */
    error = sfc7120_mcdi_clear_assertions(sc);
    if (error != 0) {
        device_printf(sc->dev,
            "hw_init: clear_assertions failed: %d\n", error);
        goto fail_mcdi;
    }

    /* Per-function resource reset. Releases any VIs / queues / filters
     * still tied to this PCI function from a previous load that didn't
     * detach cleanly. Issued AFTER DRV_ATTACH because ENTITY_RESET checks
     * SRIOV_CTG_GENERAL privilege which the function gains via TRUSTED in
     * the DRV_ATTACH response. */
    error = sfc7120_mcdi_entity_reset(sc);
    if (error != 0) {
        device_printf(sc->dev,
            "hw_init: ENTITY_RESET failed: %d\n", error);
        goto fail_attach;
    }


    error = sfc7120_mcdi_get_mac(sc);
    if (error != 0) {
        device_printf(sc->dev,
            "hw_init: GET_MAC_ADDRESSES failed: %d\n", error);
        goto fail_attach;
    }

    /* Pull GET_CAPABILITIES (plus PORT_ASSIGNMENT / FUNCTION_INFO) so the
     * cap_flags1 cache is populated before any later command that needs to
     * gate optional flag bits on advertised capabilities (e.g. the PERMIT_
     * SET_MAC_* bit in VADAPTOR_ALLOC). Non-fatal — it only logs. */
    (void)sfc7120_mcdi_dump_func_info(sc);

    /* min=1, max=32. Sfxge defaults to MIN(128, MAX(rxq_limit,txq_limit))
     * (ef10_nic.c:2003-2004). Some firmware variants reject a max=1 request
     * outright; widening the range lets the MC pick a number it's willing
     * to grant. We only actually use one VI for now. */
    error = sfc7120_mcdi_alloc_vis(sc, 1, 32);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: ALLOC_VIS failed: %d\n", error);
        sfc7120_mcdi_log_mc_state(sc, "after ALLOC_VIS fail");
        goto fail_attach;
    }

    /* Allocate the vAdaptor on the function's auto-assigned upstream port.
     * Without this, INIT_RXQ / INIT_TXQ will reject our PORT_ID. */
    error = sfc7120_mcdi_vadaptor_alloc(sc);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: VADAPTOR_ALLOC failed: %d\n", error);
        sfc7120_mcdi_log_mc_state(sc, "after VADAPTOR_ALLOC fail");
        goto fail_attach;
    }

    error = sfc7120_mcdi_get_phy_cfg(sc);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: GET_PHY_CFG failed: %d\n", error);
        goto fail_attach;
    }

    error = sfc7120_mcdi_set_mac(sc);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: SET_MAC failed: %d\n", error);
        goto fail_attach;
    }

    error = sfc7120_mcdi_set_link(sc);
    if (error != 0) {
        device_printf(sc->dev, "hw_init: SET_LINK failed: %d\n", error);
        goto fail_attach;
    }

    /* Poll for link-up, up to 3 s (30 × 100 ms). Not fatal if it stays
     * down — cable may be unplugged; we continue and let TX/RX fail later. */
    for (int i = 0; i < 30; i++) {
        error = sfc7120_mcdi_get_link(sc);
        if (error != 0) {
            device_printf(sc->dev, "hw_init: GET_LINK failed: %d\n", error);
            goto fail_attach;
        }
        if (sc->link_up)
            break;
        pause("sfc7120_lnk", hz / 10);
    }
    if (!sc->link_up)
        device_printf(sc->dev, "hw_init: link not up after 3s (cable disconnected?)\n");

    return 0;

fail_attach:
    (void)sfc7120_mcdi_drv_detach(sc);
fail_mcdi:
    sfc7120_mcdi_fini(sc);
fail:
    return error;
}

static void
sfc7120_hw_teardown(sfc7120_softc_t *sc)
{
    /* Reverse of hw_init + the queue inits done in attach. Each helper is a
     * no-op if the corresponding stage never ran, so it's safe to call from
     * partial-attach failure paths.
     *
     * Order matters: FINI_TXQ / FINI_RXQ must complete before FINI_EVQ
     * (firmware returns EBUSY otherwise — RX/TX queues hold a reference to
     * their target EVQ). VADAPTOR_FREE must happen before FREE_VIS. */
    (void)sfc7120_mcdi_filter_remove(sc);
    (void)sfc7120_mcdi_fini_txq(sc, 0);
    (void)sfc7120_mcdi_fini_rxq(sc, 0);
    (void)sfc7120_mcdi_fini_evq(sc, 1);   /* data EVQ first (reverse of init) */
    (void)sfc7120_mcdi_fini_evq(sc, 0);   /* control EVQ — the wakeup anchor */
    (void)sfc7120_mcdi_vadaptor_free(sc);
    (void)sfc7120_mcdi_free_vis(sc);
    (void)sfc7120_mcdi_drv_detach(sc);
    sfc7120_mcdi_fini(sc);
}

/* ---------------------------------------------------------------------- */
/* PCI probe / attach / detach                                            */
/* ---------------------------------------------------------------------- */

static int
sfc7120_fbsd_probe(device_t dev)
{
    uint16_t vendor = pci_get_vendor(dev);
    uint16_t devid  = pci_get_device(dev);

    for (size_t i = 0; i < sfc7120_dev_count; i++) {
        if (vendor == sfc7120_fbsd_devs[i].vendor &&
            devid  == sfc7120_fbsd_devs[i].device) {
            device_set_desc(dev, sfc7120_fbsd_devs[i].desc);
            return BUS_PROBE_DEFAULT;
        }
    }
    return ENXIO;
}

static int
sfc7120_fbsd_attach(device_t dev)
{
	//finn: dev = opaque handle representing the PCI device instance (the solarflare). This gets us a pointer to a zero'd block of mem alloc'd by the kernel
	//sc contains BAR handles, DMA buffers, ring pointers, MAC, CAPIO state
    sfc7120_softc_t *sc = device_get_softc(dev);
    int error;
	
    
    sc->dev = dev;
    sc->dying = false;
    sc->device_attached = false;
    //finn: sc_mtx : main device lock, protects fields touched by multiple contexts. mutex taken by any thread that wants to use shared fields
    SFC7120_LOCK_INIT(sc);
    SFC7120_RX_LOCK_INIT(sc);

    /* 1. Allocate BAR2 (function MMIO window). */
    device_printf(dev, "TRACE: about to alloc BAR2\n");
    sc->mem_res_id = PCIR_BAR(2); // by the user guide, BARO is I/O on EF10, BAR2 is the register window
    sc->mem_resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
        &sc->mem_res_id, RF_ACTIVE); // freebsd returns null when you ask for a shared memory resource -> must be active
    if (sc->mem_resource == NULL) {
        device_printf(dev, "could not allocate BAR2\n");
        error = ENXIO;
        goto fail_lock;
    }
    sc->mem_bus_tag = rman_get_bustag(sc->mem_resource);
    sc->mem_bsh     = rman_get_bushandle(sc->mem_resource);
    device_printf(dev, "TRACE: BAR2 mapped, paddr=0x%lx size=0x%lx\n",
        rman_get_start(sc->mem_resource), rman_get_size(sc->mem_resource));

    /* CAPIO is_physical regions hand userspace rman_get_start() as if it were a
     * CPU physical address (capio.c feeds it straight to vm_page_getfake), with
     * no host-bridge translation. The kernel's own reads go through mem_bsh,
     * which IS translated. If these two disagree, userspace maps the wrong
     * physical page and reads RAZ (0) with no CHERI trap — exactly the
     * HW_REV_ID-via-slice symptom. MATCH => address is fine, look elsewhere. */
    {
        vm_paddr_t bus_start = (vm_paddr_t)rman_get_start(sc->mem_resource);
        vm_paddr_t true_pa   = pmap_kextract((vm_offset_t)sc->mem_bsh);
        device_printf(dev,
            "BAR2 PA check: rman_get_start=0x%lx  pmap_kextract(mem_bsh)=0x%lx (%s)\n",
            (u_long)bus_start, (u_long)true_pa,
            bus_start == true_pa ? "MATCH" : "MISMATCH — capio maps wrong PA");
    }
    pci_enable_busmaster(dev);

    /* 2. Hardware bringup (MCDI handshake, queue init). */
    device_printf(dev, "TRACE: calling hw_init\n");
    error = sfc7120_hw_init(sc);
    if (error != 0) {
        device_printf(dev, "hw_init failed: %d\n", error);
        goto fail_bar;
    }
    device_printf(dev, "TRACE: hw_init done\n");


    /* 3. DMA buffer allocation. */
    device_printf(dev, "TRACE: calling alloc_dma_resources\n");
    error = sfc7120_alloc_dma_resources(sc);
    if (error != 0) {
        device_printf(dev, "alloc_dma_resources failed: %d\n", error);
        goto fail_hw;
    }
    device_printf(dev, "TRACE: alloc_dma_resources done\n");

    /* MSI-X allocation. Runs before INIT_EVQ so the interrupting control EVQ
     * (0) has its vector ready. A spurious interrupt before
     * evq_initialized=true is harmless — the handler's latch bails. */
    device_printf(dev, "TRACE: calling intr_setup\n");
    error = sfc7120_intr_setup(sc);
    if (error != 0) {
        device_printf(dev, "intr_setup failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: intr_setup done\n");

    /* INITIALIZING EVENT QUEUE */
    device_printf(dev, "TRACE: calling init_evq\n");
    /* INSTANCE is the *function-local* queue index, not the absolute VI
     * number (ALLOC_VIS-returned vi_base). EF10 firmware additionally
     * requires the first EVQ (function-local 0) to be interrupting —
     * init_evq sets the INTERRUPTING flag and IRQ_NUM=0 accordingly. */
    error = sfc7120_mcdi_init_evq(sc, 0, sc->evq_ring_paddr, SFC7120_NUM_EVQ_ENTRY);
    if (error != 0) {
        device_printf(dev, "init_evq failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: init_evq done (control, EVQ 0)\n");

    /* Control EVQ is up: track its init state (for symmetric fini) and arm
     * the ISR. The handler only reads EVQ 0, so arming here — before EVQ 1
     * exists — is safe. */
    sc->evq_initialized = true;

    /* Data EVQ (instance 1): non-interrupting, carries TX/RX completions.
     * Created AFTER EVQ 0 — a non-interrupting EVQ must reference an
     * already-existing interrupting EVQ (EVQ 0) for wake-ups. */
    error = sfc7120_mcdi_init_evq(sc, 1, sc->data_evq_ring_paddr,
                                  SFC7120_NUM_EVQ_ENTRY);
    if (error != 0) {
        device_printf(dev, "init_evq(data) failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: init_evq done (data, EVQ 1)\n");

    /* Track data EVQ init state for symmetric fini. */
    sc->data_evq_initialized = true;

    /* INITIALIZING RX QUEUE. Args: (instance, target_evq, ...). RXQ 0's
     * completion events are steered to the DATA EVQ (instance 1), not the
     * control EVQ — this retarget is what frees EVQ 0 to become control-only. */
    device_printf(dev, "TRACE: calling init_rxq\n");
    error = sfc7120_mcdi_init_rxq(sc, 0, 1, sc->rx_desc_ring_paddr,
                                  SFC7120_NUM_RX_DESC);
    if (error != 0) {
        device_printf(dev, "init_rxq failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: init_rxq done\n");

    sfc7120_post_rx_buffers(sc);

    /* INITIALIZING TX QUEUE. TXQ 0's completion events likewise go to the
     * DATA EVQ (instance 1). */
    device_printf(dev, "TRACE: calling init_txq\n");
    error = sfc7120_mcdi_init_txq(sc, 0, 1, sc->tx_desc_ring_paddr,
                                  SFC7120_NUM_TX_DESC);
    if (error != 0) {
        device_printf(dev, "init_txq failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: init_txq done\n");

    /* Insert RX filter so the MC steers incoming frames to RXQ 0.
     * Must come after INIT_RXQ since the filter names RX queue 0 as its
     * destination. Without this, the firmware drops all RX frames. */
    error = sfc7120_mcdi_filter_insert(sc);
    if (error != 0) {
        device_printf(dev, "filter_insert failed: %d\n", error);
        goto fail_dma;
    }

    /* 4. Create cdev. make_dev_capio overwrites d_ioctl with
     *    capio_ioctl_handler and registers the modmap callbacks. */
    device_printf(dev, "TRACE: calling make_dev_capio\n");
    /* unit and name must be unique per device instance — two cards both
     * attaching as "sfc7120pol" unit 0 causes make_dev to block on a
     * duplicate devfs entry, hanging the kernel */
    sc->cdev = make_dev_capio(&sfc7120_cdevsw, sc, device_get_unit(dev),
                              UID_ROOT, GID_WHEEL, 0600, "sfc7120pol%d",
                              device_get_unit(dev));
    if (sc->cdev == NULL) {
        error = ENOMEM;
        goto fail_dma;
    }
    sc->cdev->si_drv1 = sc;
    device_printf(dev, "TRACE: cdev created\n");

    /* 5. Populate smem[]. The MMIO entry uses the BAR's physical address;
     *    DMA entries use the kernel virtual address of each allocation.
     *    Order MUST match the sfc7120_vm_map_type_t enum. */
    sc->smem[SFC7120_TX_BUFFER].type        = SFC7120_TX_BUFFER;
    sc->smem[SFC7120_TX_BUFFER].is_physical = false;
    sc->smem[SFC7120_TX_BUFFER].addr        = sc->tx_buffer;
    sc->smem[SFC7120_TX_BUFFER].len         =
        SFC7120_NUM_TX_DESC * SFC7120_TX_BUFFER_SIZE;
    sc->smem[SFC7120_TX_BUFFER].is_sliced   = false;

    sc->smem[SFC7120_RX_BUFFER].type        = SFC7120_RX_BUFFER;
    sc->smem[SFC7120_RX_BUFFER].is_physical = false;
    sc->smem[SFC7120_RX_BUFFER].addr        = sc->rx_buffer;
    sc->smem[SFC7120_RX_BUFFER].len         =
        SFC7120_NUM_RX_DESC * SFC7120_RX_BUFFER_SIZE;
    sc->smem[SFC7120_RX_BUFFER].is_sliced   = false;

    sc->smem[SFC7120_MMIO_REGION].type        = SFC7120_MMIO_REGION;
    sc->smem[SFC7120_MMIO_REGION].is_physical = true;
    sc->smem[SFC7120_MMIO_REGION].paddr       = rman_get_start(sc->mem_resource);
    sc->smem[SFC7120_MMIO_REGION].len         = rman_get_size(sc->mem_resource);
    sc->smem[SFC7120_MMIO_REGION].is_sliced   = true;
    sc->smem[SFC7120_MMIO_REGION].slice_definitions = sfc7120_reg_slices;
    sc->smem[SFC7120_MMIO_REGION].slice_def_len     = SFC7120_MMIO_SLICE_COUNT;

    /* Data-path rings (phase A): expose the TX/RX descriptor rings and the
     * data EVQ ring (instance 1) so userspace can poll and post directly in
     * later phases. All three are kernel-VA DMA allocations, not sliced. The
     * EVQ region is sc->data_evq_ring (the non-interrupting data EVQ), NOT the
     * kernel-owned control EVQ 0. Userspace still drives TX/RX through the
     * SFC7120_TX / SFC7120_RX ioctls until the direct path lands; mapping these
     * regions does not by itself remove the kernel from the data path. */
    sc->smem[SFC7120_TX_DESC_RING].type        = SFC7120_TX_DESC_RING;
    sc->smem[SFC7120_TX_DESC_RING].is_physical = false;
    sc->smem[SFC7120_TX_DESC_RING].addr        = sc->tx_desc_ring;
    sc->smem[SFC7120_TX_DESC_RING].len         =
        SFC7120_NUM_TX_DESC * SFC7120_TX_DESC_SIZE;
    sc->smem[SFC7120_TX_DESC_RING].is_sliced   = false;

    sc->smem[SFC7120_RX_DESC_RING].type        = SFC7120_RX_DESC_RING;
    sc->smem[SFC7120_RX_DESC_RING].is_physical = false;
    sc->smem[SFC7120_RX_DESC_RING].addr        = sc->rx_desc_ring;
    sc->smem[SFC7120_RX_DESC_RING].len         =
        SFC7120_NUM_RX_DESC * SFC7120_RX_DESC_SIZE;
    sc->smem[SFC7120_RX_DESC_RING].is_sliced   = false;

    sc->smem[SFC7120_EVQ_RING].type        = SFC7120_EVQ_RING;
    sc->smem[SFC7120_EVQ_RING].is_physical = false;
    sc->smem[SFC7120_EVQ_RING].addr        = sc->data_evq_ring;
    sc->smem[SFC7120_EVQ_RING].len         =
        SFC7120_NUM_EVQ_ENTRY * SFC7120_EVQ_ENTRY_SIZE;
    sc->smem[SFC7120_EVQ_RING].is_sliced   = false;

    device_printf(dev, "TRACE: smem populated\n");

    /* 6. init_capio_sc. MUST come AFTER make_dev_capio and AFTER smem[]
     *    is fully populated — capio.c stores a pointer to the array, not
     *    a copy, and registers the sealing key + cap_mtx. */
    device_printf(dev, "TRACE: calling init_capio_sc\n");
    init_capio_sc(sc, &sfc7120_capio_ops, SFC7120_REGION_COUNT, sc->smem);
    device_printf(dev, "TRACE: init_capio_sc done\n");

    /* 7. Allocate vm_object_handle_t for each region. -- THIS IS WRONG AS CAPIO.C:395 HANDLES THIS ON MMAP CALLS */
    // for (int i = 0; i < SFC7120_REGION_COUNT; i++) {
    //     vm_object_handle_t *h = malloc(sizeof(*h), M_DEVBUF,
    //                                    M_WAITOK | M_ZERO);
    //     h->sc   = sc;
    //     h->type = sc->smem[i].type;
    //     sc->smem[i].vm_object_handle = h;
    // }

    sc->device_attached = true;
    // device_printf(dev, "TRACE: calling dump_regs\n"); -> apparently firmware needs to initialize the hardware registers first 
    sfc7120_dump_regs(sc);
    device_printf(dev, "sfc7120pol attached\n");
    return 0;

fail_dma:
    /* Stop interrupts first — the ISR reads the EVQ DMA we're about to free.
     * Then FINI the queues (inside hw_teardown) BEFORE freeing the DMA rings
     * the firmware is writing into — otherwise the NIC keeps DMAing into freed
     * memory. Can't fall through to fail_hw or we'd call hw_teardown twice. */
    sfc7120_intr_teardown(sc);
    sfc7120_hw_teardown(sc);
    sfc7120_free_dma_resources(sc);
    goto fail_bar;
fail_hw:
    sfc7120_hw_teardown(sc);
fail_bar:
    pci_disable_busmaster(dev);
    bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_res_id, sc->mem_resource);
fail_lock:
    SFC7120_RX_LOCK_DESTROY(sc);
    SFC7120_LOCK_DESTROY(sc);
    return error;
}

static int
sfc7120_fbsd_detach(device_t dev)
{
    sfc7120_softc_t *sc = device_get_softc(dev);

    SFC7120_LOCK(sc);
    sc->device_attached = false;
    sc->dying = true;
    SFC7120_UNLOCK(sc);

    
    // from what i can tell capio handles this, thus the vm_object_handle fields don't exist yet 

    /* Free vm_object_handle allocations BEFORE destroy_dev — the pager may
    //  * still reference them during teardown. */
    // for (int i = 0; i < SFC7120_REGION_COUNT; i++) {
    //     if (sc->smem[i].vm_object_handle != NULL) {
    //         free(sc->smem[i].vm_object_handle, M_DEVBUF);
    //         sc->smem[i].vm_object_handle = NULL;
    //     }
    // }

    if (sc->cdev != NULL) {
        modmap_unregister_vaddr_callback(sc->cdev);
        destroy_dev(sc->cdev);
        sc->cdev = NULL;
    }

    /* Stop interrupts before fini'ing queues / freeing the EVQ DMA the ISR
     * reads. (dying=true was set above, so the handler already no-ops; this
     * guarantees the vector is gone before the memory is.) */
    sfc7120_intr_teardown(sc);

    /* Firmware teardown (FINI_TXQ/RXQ/EVQ, VADAPTOR_FREE, FREE_VIS, ...)
     * MUST happen before freeing the DMA rings — otherwise the NIC keeps
     * DMA-writing events/completions into freed memory. */
    sfc7120_hw_teardown(sc);
    sfc7120_free_dma_resources(sc);

    if (sc->mem_resource != NULL) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_res_id,
                             sc->mem_resource);
        sc->mem_resource = NULL;
    }
    pci_disable_busmaster(dev);

    /* MUST be last: capio_destroy tears down the cap_mtx that was
     * initialized by init_capio_sc. */
    capio_destroy(sc);

    SFC7120_RX_LOCK_DESTROY(sc);
    SFC7120_LOCK_DESTROY(sc);
    return 0;
}

/* ---------------------------------------------------------------------- */
/* cdev callbacks                                                         */
/* ---------------------------------------------------------------------- */

static int
sfc7120_open(struct cdev *dev, int flags, int devtype, struct thread *td)
{
    return 0;
}

static int
sfc7120_close(struct cdev *dev, int flags, int devtype, struct thread *td)
{
    sfc7120_softc_t *sc = dev->si_drv1;
    if (sc != NULL)
        revoke_access(sc);
    return 0;
}

static int
sfc7120_poll(struct cdev *dev, int events, struct thread *td)
{
    sfc7120_softc_t *sc = dev->si_drv1;
    int revents = 0;

    if (sc == NULL)
        return POLLERR;

    SFC7120_RX_LOCK(sc);
    if (sc->rx_received)
        revents |= events & (POLLIN | POLLRDNORM);
    else
        selrecord(td, &sc->selinfo);
    SFC7120_RX_UNLOCK(sc);
    return revents;
}

static int
sfc7120_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag,
              struct thread *td)
{
    sfc7120_softc_t *sc = dev->si_drv1;

    if (sc == NULL)
        return EINVAL;

    /* CAPIO_ATTACH, CAPIO_GOODBYE, CAPIO_PRINT_PTE, CAPIO_PRINT_PTE_VA are
     * intercepted by capio_ioctl_handler before this function is called.
     * Anything that lands here is driver-specific. */
    switch (cmd) {
    case SFC7120_GET_MAC: {
        sfc7120_mac_req_t *req = (sfc7120_mac_req_t *)data;
        SFC7120_LOCK(sc);
        memcpy(req->mac_addr, sc->mac_addr, sizeof(req->mac_addr));
        SFC7120_UNLOCK(sc);
        return 0;
    }
    case SFC7120_GET_VI_INFO: {
        /*
         * Hand userspace the VI geometry it needs to drive the rings
         * directly (phase C+). The CAPIO framework has already validated
         * user_cap/sealed_cap before dispatch, so no extra token check is
         * needed here (same as GET_MAC). Instance numbers are
         * function-relative and match the init_evq/rxq/txq calls in
         * sfc7120_hw_init: data EVQ = 1, RXQ = 0, TXQ = 0.
         */
        sfc7120_vi_info_req_t *req = (sfc7120_vi_info_req_t *)data;
        SFC7120_LOCK(sc);
        req->tx_buffer_paddr = sc->tx_buffer_paddr;
        req->rx_buffer_paddr = sc->rx_buffer_paddr;
        req->vi_base         = sc->vi_base;
        req->evq_instance    = SFC7120_DATA_EVQ_INSTANCE;
        req->rxq_instance    = SFC7120_RXQ_INSTANCE;
        req->txq_instance    = SFC7120_TXQ_INSTANCE;
        req->num_tx_desc     = SFC7120_NUM_TX_DESC;
        req->num_rx_desc     = SFC7120_NUM_RX_DESC;
        req->num_evq_entry   = SFC7120_NUM_EVQ_ENTRY;
        req->tx_head         = sc->tx_head;
        req->rx_head         = sc->rx_head;
        req->evq_read_ptr    = sc->data_evq_read_ptr;
        SFC7120_UNLOCK(sc);
        return 0;
    }
    case SFC7120_TX: {
        /*
         * Phase 1 TX ioctl handler.
         *
         * Userspace passes a CHERI capability (tx_buf_addr) pointing at the
         * packet to send, and a byte length. We:
         *   1. Validate the capability — must be tagged, must be readable,
         *      must cover at least `length` bytes.
         *   2. Copy the packet from userspace into the next slot in the TX
         *      DMA buffer (sc->tx_buffer), which is system RAM the NIC can
         *      reach over PCIe.
         *   3. Write an 8-byte TX descriptor into sc->tx_desc_ring at
         *      tx_head, telling the NIC the physical address and length.
         *   4. Ring the TX doorbell so the NIC goes and sends it.
         *   5. Poll the EVQ until a TX completion event arrives confirming
         *      the NIC is done with that descriptor.
         *   6. Advance tx_head and clean up the EVQ slot.
         */
        sfc7120_tx_req_t *req = (sfc7120_tx_req_t *)data;

        /* --- Step 1: validate CHERI capability --- */
        if (!cheri_gettag(req->tx_buf_addr))
            return EINVAL;
        if (!(cheri_getperm(req->tx_buf_addr) & CHERI_PERM_LOAD))
            return EPERM;
        if (req->length == 0 || req->length > SFC7120_TX_BUFFER_SIZE)
            return EINVAL;
        if (cheri_getlen(req->tx_buf_addr) < req->length)
            return EINVAL;

        /* --- Step 2: copy packet into TX DMA buffer slot --- */
        SFC7120_LOCK(sc);

        uint8_t *slot_va = (uint8_t *)sc->tx_buffer +
                           sc->tx_head * SFC7120_TX_BUFFER_SIZE;
        bus_addr_t slot_pa = sc->tx_buffer_paddr +
                             sc->tx_head * SFC7120_TX_BUFFER_SIZE;

        int copy_err = copyin(req->tx_buf_addr, slot_va, req->length);
        if (copy_err != 0) {
            SFC7120_UNLOCK(sc);
            return copy_err;
        }

        /* Flush the packet bytes to DMA memory before writing the
         * descriptor — the NIC must see the data before it sees the
         * descriptor that points to it. */
        bus_dmamap_sync(sc->tx_buffer_dtag, sc->tx_buffer_dmamap,
                        BUS_DMASYNC_PREWRITE);

        /* --- Step 3: write TX descriptor ---
         * 8-byte EF10 TX kernel descriptor layout (efx_regs_ef10.h):
         *   bit[63]      TYPE  = 0  (kernel descriptor, not PIO)
         *   bit[62]      CONT  = 0  (last/only segment — EOP)
         *   bits[61:48]  BYTE_CNT   packet length (14 bits)
         *   bits[47:32]  BUF_ADDR upper 16 bits of 48-bit physical address
         *   bits[31:0]   BUF_ADDR lower 32 bits of physical address
         */
        uint64_t tx_desc =
            ((uint64_t)(req->length & 0x3fff)        << 48) |
            ((uint64_t)((slot_pa >> 32) & 0xffff)    << 32) |
            ((uint64_t)(slot_pa & 0xffffffff));

        volatile uint64_t *tx_ring = (volatile uint64_t *)sc->tx_desc_ring;
        tx_ring[sc->tx_head] = tx_desc;

        bus_dmamap_sync(sc->tx_desc_dtag, sc->tx_desc_dmamap,
                        BUS_DMASYNC_PREWRITE);

        /* --- Step 4: ring TX doorbell ---
         * Write the new producer index (tx_head + 1) so the NIC knows one
         * new descriptor is ready. The NIC reads the descriptor, fetches the
         * packet from DMA memory, and transmits it. */
        int posted_idx = sc->tx_head;
        /* Push TX write pointer via the WRITED2 path.
         * ERF_DZ_TX_DESC_WPTR lives at dword[2] (LBN=64) of TX_DESC_UPD.
         * Write to TX_WPTR_DBL = TX_DESC_DBL + 8 = 0x0a18. */
        uint32_t wptr = (uint32_t)(sc->tx_head + 1) & (SFC7120_NUM_TX_DESC - 1);
        SFC7120_WRITE_REG(sc, SFC7120_REG_DATA_TX_WPTR_DBL, wptr);

        /* --- Step 5: poll EVQ for TX completion ---
         * The NIC posts an 8-byte TX_EV event to the EVQ when it finishes
         * sending. Event layout (efx_regs_ef10.h):
         *   bits[63:60]  EV_CODE = 2 (TX_EV)
         *   bits[15:0]   TX_DESCR_INDX — index of completed descriptor
         *
         * We spin up to 10000 iterations waiting for it. */
        /* Since 1c, TXQ 0 completions land in the DATA EVQ (instance 1), so
         * we poll data_evq_ring — not the control EVQ. */
        volatile uint64_t *evq = (volatile uint64_t *)sc->data_evq_ring;
        int tx_done = 0;
        int consumed = 0;
        for (int tries = 0; tries < 10000 && !tx_done; tries++) {
            bus_dmamap_sync(sc->data_evq_dtag, sc->data_evq_dmamap,
                            BUS_DMASYNC_POSTREAD);
            uint64_t ev = evq[sc->data_evq_read_ptr];

            /* All-ones or all-zeros means no event written yet. */
            if (ev == 0xffffffffffffffffULL || ev == 0)
                continue;

            uint32_t code = (uint32_t)(ev >> 60) & 0xf;
            if (code == 2) { /* TX_EV */
                uint32_t comp_idx = (uint32_t)(ev & 0xffff);
                if (comp_idx == (uint32_t)posted_idx) {
                    tx_done = 1;
                }
            }

            /* Consume the event regardless — advance past it. */
            evq[sc->data_evq_read_ptr] = 0;
            sc->data_evq_read_ptr =
                (sc->data_evq_read_ptr + 1) & (SFC7120_NUM_EVQ_ENTRY - 1);
            consumed++;
        }

        /* Ack consumed events on the DATA EVQ's own RPTR doorbell — at the
         * instance-1 window offset (0x2400), NOT EVQ 0's 0x0400. Mirrors the
         * control-EVQ ack in the ISR. Without this the NIC eventually sees
         * the EVQ as full and stops posting completions. */
        if (consumed > 0) {
            bus_dmamap_sync(sc->data_evq_dtag, sc->data_evq_dmamap,
                            BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
            SFC7120_WRITE_REG(sc, SFC7120_REG_DATA_EVQ_RPTR_DBL,
                (uint32_t)sc->data_evq_read_ptr & SFC7120_EVQ_RPTR_MASK);
        }

        /* --- Step 6: advance tx_head --- */
        sc->tx_head = (sc->tx_head + 1) & (SFC7120_NUM_TX_DESC - 1);
        req->status = tx_done ? 0 : ETIMEDOUT;

        SFC7120_UNLOCK(sc);
        return tx_done ? 0 : ETIMEDOUT;
    }
    case SFC7120_RX: {
        /*
         * Phase 1 RX ioctl handler.
         *
         * The NIC DMAs incoming packets into the RX buffer slots we
         * pre-posted in sfc7120_post_rx_buffers. When it finishes filling
         * a slot it posts an 8-byte RX_EV event on the EVQ. We:
         *   1. Poll the EVQ until an RX_EV (code=0) arrives.
         *   2. Extract the byte count from the event.
         *   3. Verify the low 4 bits of the descriptor index match rx_head
         *      so we know we are reading the right slot.
         *   4. Copy the packet out of the DMA buffer slot to userspace.
         *   5. Re-post that descriptor so the NIC can reuse the slot.
         *   6. Ring the RX doorbell and advance rx_head.
         *
         * RX_EV layout (efx_regs_ef10.h):
         *   bits[63:60]  EV_CODE = 0
         *   bits[51:48]  RX_DSC_PTR_LBITS — low 4 bits of descriptor index
         *   bits[13:0]   RX_BYTES — byte count of received packet
         */
        sfc7120_rx_req_t *req = (sfc7120_rx_req_t *)data;

        SFC7120_LOCK(sc);

        /* --- Step 1: poll the DATA EVQ for an RX event ---
         * Since 1c, RXQ 0 completions land in the data EVQ (instance 1). */
        volatile uint64_t *evq = (volatile uint64_t *)sc->data_evq_ring;
        int    rx_found = 0;
        int    consumed = 0;
        uint32_t rx_bytes = 0;

        for (int tries = 0; tries < 100000 && !rx_found; tries++) {
            bus_dmamap_sync(sc->data_evq_dtag, sc->data_evq_dmamap,
                            BUS_DMASYNC_POSTREAD);
            uint64_t ev = evq[sc->data_evq_read_ptr];

            if (ev == 0xffffffffffffffffULL || ev == 0)
                continue;

            uint32_t code = (uint32_t)(ev >> 60) & 0xf;
            if (code == 0) { /* RX_EV */
                /* Extract byte count (includes 14-byte EF10 prefix). Trust
                 * rx_head as the slot — same approach as Finn's ISR path. */
                rx_bytes = (uint32_t)(ev & 0x3fff);
                rx_found = 1;
            }

            /* Consume event regardless of type. */
            evq[sc->data_evq_read_ptr] = 0;
            sc->data_evq_read_ptr = (sc->data_evq_read_ptr + 1) &
                               (SFC7120_NUM_EVQ_ENTRY - 1);
            consumed++;
        }

        /* Ack consumed events on the data EVQ's instance-1 RPTR doorbell
         * (0x2400) — see the TX path note above. */
        if (consumed > 0) {
            bus_dmamap_sync(sc->data_evq_dtag, sc->data_evq_dmamap,
                            BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
            SFC7120_WRITE_REG(sc, SFC7120_REG_DATA_EVQ_RPTR_DBL,
                (uint32_t)sc->data_evq_read_ptr & SFC7120_EVQ_RPTR_MASK);
        }

        if (!rx_found) {
            SFC7120_UNLOCK(sc);
            return ETIMEDOUT;
        }

        /* --- Step 4: copy packet from DMA slot to userspace ---
         * sc->rx_buffer is the virtual address the CPU uses to read the
         * DMA memory. We sync first to ensure the NIC's DMA writes are
         * visible to the CPU, then strip the 14-byte EF10 prefix before
         * copying out to userspace. */
        uint8_t *slot_va = (uint8_t *)sc->rx_buffer +
                           sc->rx_head * SFC7120_RX_BUFFER_SIZE;

        bus_dmamap_sync(sc->rx_buffer_dtag, sc->rx_buffer_dmamap,
                        BUS_DMASYNC_POSTREAD);

        /* Read frame length from prefix offset +8 (LE uint16, excl. prefix).
         * If 0, firmware was in cut-through mode — derive from event bytes. */
        uint16_t plen = (uint16_t)slot_va[8] | ((uint16_t)slot_va[9] << 8);
        if (plen == 0) {
            plen = (rx_bytes > SFC7120_EF10_RX_PREFIX_LEN)
                   ? (uint16_t)(rx_bytes - SFC7120_EF10_RX_PREFIX_LEN)
                   : 0;
        }

        int copy_err = copyout(slot_va + SFC7120_EF10_RX_PREFIX_LEN,
                               req->raw_buffer, plen);
        if (copy_err != 0) {
            SFC7120_UNLOCK(sc);
            return copy_err;
        }
        req->length_received = plen;

        /* --- Step 5: re-post the descriptor for this slot ---
         * We just consumed the slot — write the descriptor back into the
         * ring so the NIC knows it can reuse this buffer for the next
         * incoming packet. Same format as sfc7120_post_rx_buffers. */
        volatile uint64_t *rx_ring = (volatile uint64_t *)sc->rx_desc_ring;
        bus_addr_t slot_pa = sc->rx_buffer_paddr +
                             sc->rx_head * SFC7120_RX_BUFFER_SIZE;
        uint64_t desc =
            ((uint64_t)(SFC7120_RX_BUFFER_SIZE & 0x3fff)    << 48) |
            ((uint64_t)((slot_pa >> 32) & 0xffff)            << 32) |
            ((uint64_t)(slot_pa & 0xffffffff));
        rx_ring[sc->rx_head] = desc;

        bus_dmamap_sync(sc->rx_desc_dtag, sc->rx_desc_dmamap,
                        BUS_DMASYNC_PREWRITE);

        /* --- Step 6: ring RX doorbell and advance rx_head ---
         * Write the current rx_head as the new producer pointer. This
         * hands the re-posted slot back to the NIC. Then advance rx_head
         * to the next expected slot. */
        SFC7120_WRITE_REG(sc, SFC7120_REG_RX_DESC_DBL,
                          (uint32_t)sc->rx_head);
        sc->rx_head = (sc->rx_head + 1) & (SFC7120_NUM_RX_DESC - 1);

        req->status = 0;
        SFC7120_UNLOCK(sc);
        return 0;
    }
    default:
        return ENOTTY;
    }
}

/* ---------------------------------------------------------------------- */
/* Interrupt handling                                                     */
/* ---------------------------------------------------------------------- */

/* EF10 event layout (64-bit qword). Only the fields the control ISR parses
 * are defined here; extend as more event types are wired up. */
#define SFC7120_EV_CODE_LBN          60     /* top 4 bits of the qword */
#define SFC7120_EV_CODE_RX           0
#define SFC7120_EV_CODE_TX           2
#define SFC7120_EV_CODE_DRIVER       5
#define SFC7120_EV_CODE_MCDI         12

/* MCDI sub-event layout (bits inside the qword). */
#define SFC7120_MCDI_EVENT_CODE_LBN  44     /* MCDI subcode field, 8 bits */
#define SFC7120_MCDI_EV_LINKCHANGE   0x4

/* LINKCHANGE payload layout (low 32 bits of the qword). */
#define SFC7120_LC_SPEED_SHIFT       16     /* 4 bits */
#define SFC7120_LC_FCNTL_SHIFT       20     /* 4 bits */
#define SFC7120_LC_FLAGS_SHIFT       24     /* 8 bits */
#define SFC7120_LC_FLAG_LINK_UP      (1u << 0)
#define SFC7120_LC_FLAG_FDX          (1u << 1)

/* SFC7120_EVQ_RPTR_MASK lives in sfc7120_mmio.h next to the register. */

static uint32_t
sfc7120_linkchange_speed_mbps(uint32_t code)
{
    switch (code) {
    case 1: return 100;
    case 2: return 1000;
    case 3: return 10000;
    case 4: return 40000;
    case 5: return 25000;
    case 6: return 50000;
    case 7: return 100000;
    default: return 0;
    }
}

static void
sfc7120_interrupt_handler(void *arg)
{
    sfc7120_softc_t   *sc = (sfc7120_softc_t *)arg;
    volatile uint64_t *ring;
    int                processed = 0;

    /* Safe-to-run latch: bail if the control EVQ isn't up yet or we're
     * tearing down. An MSI-X vector can fire the instant it's wired (before
     * INIT_EVQ) or during detach; these checks keep those harmless. */
    if (sc == NULL || sc->evq_ring == NULL)
        return;
    if (sc->dying || !sc->evq_initialized)
        return;

    ring = (volatile uint64_t *)sc->evq_ring;   /* EVQ 0 = control queue */

    SFC7120_LOCK(sc);

    /* NIC writes events via DMA; sync for CPU reads (BUS_DMA_COHERENT, so
     * this is a barrier hint rather than a cache flush). */
    bus_dmamap_sync(sc->evq_dtag, sc->evq_dmamap,
                    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

    for (;;) {
        uint64_t ev = ring[sc->evq_read_ptr];

        /* Slots are 0xff-primed at alloc and zeroed on consume; either
         * pattern means "no event present". */
        if (ev == 0xffffffffffffffffULL || ev == 0)
            break;

        uint32_t lo   = (uint32_t)(ev & 0xffffffffu);
        uint32_t hi   = (uint32_t)(ev >> 32);
        uint32_t code = (hi >> (SFC7120_EV_CODE_LBN - 32)) & 0xfu;

        switch (code) {
        case SFC7120_EV_CODE_MCDI: {
            uint32_t mcdi_code =
                (hi >> (SFC7120_MCDI_EVENT_CODE_LBN - 32)) & 0xffu;
            if (mcdi_code == SFC7120_MCDI_EV_LINKCHANGE) {
                uint32_t flags = (lo >> SFC7120_LC_FLAGS_SHIFT) & 0xffu;
                uint32_t scode = (lo >> SFC7120_LC_SPEED_SHIFT) & 0xfu;
                uint32_t fcntl = (lo >> SFC7120_LC_FCNTL_SHIFT) & 0xfu;
                bool up  = (flags & SFC7120_LC_FLAG_LINK_UP) != 0;
                bool fdx = (flags & SFC7120_LC_FLAG_FDX) != 0;

                sc->link_up         = up;
                sc->full_duplex     = fdx;
                sc->link_speed_mbps = sfc7120_linkchange_speed_mbps(scode);
                sc->link_fcntl      = fcntl;

                device_printf(sc->dev,
                    "EVQ LINKCHANGE: link=%s speed=%u Mbps %s fcntl=%u "
                    "flags=%#x\n", up ? "UP" : "DOWN", sc->link_speed_mbps,
                    fdx ? "FDX" : "HDX", fcntl, flags);
            } else {
                device_printf(sc->dev,
                    "EVQ MCDI_EV: subcode=%#x ev=%#016jx\n",
                    mcdi_code, (uintmax_t)ev);
            }
            break;
        }
        case SFC7120_EV_CODE_DRIVER:
            device_printf(sc->dev, "EVQ DRIVER_EV: %#016jx\n",
                          (uintmax_t)ev);
            break;
        default:
            /* RX(0)/TX(2) must never land here — they target the data EVQ.
             * If one does, the 1c retarget regressed. */
            device_printf(sc->dev, "EVQ0 event: code=%#x raw=%#016jx\n",
                          code, (uintmax_t)ev);
            break;
        }

        ring[sc->evq_read_ptr] = 0;
        sc->evq_read_ptr =
            (sc->evq_read_ptr + 1) & (SFC7120_NUM_EVQ_ENTRY - 1);
        if (++processed >= SFC7120_NUM_EVQ_ENTRY) {
            device_printf(sc->dev,
                "EVQ0: processed full ring without empty marker\n");
            break;
        }
    }

    if (processed > 0) {
        bus_dmamap_sync(sc->evq_dtag, sc->evq_dmamap,
                        BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
        /* EVQ 0's RPTR doorbell IS the hardcoded 0x0400 — correct for the
         * control queue (the per-VI offset issue only affects EVQ 1). */
        SFC7120_WRITE_REG(sc, SFC7120_REG_EVQ_RPTR_DBL,
                          (uint32_t)sc->evq_read_ptr & SFC7120_EVQ_RPTR_MASK);
    }

    SFC7120_UNLOCK(sc);
}

static void
sfc7120_rx_task_handler(void *context, int pending)
{
    sfc7120_softc_t *sc = (sfc7120_softc_t *)context;
    SFC7120_RX_LOCK(sc);
    sc->rx_received = true;
    selwakeuppri(&sc->selinfo, PZERO + 1);
    SFC7120_RX_UNLOCK(sc);
}

/* ---------------------------------------------------------------------- */
/* MSI-X interrupt setup / teardown (control EVQ 0)                       */
/* ---------------------------------------------------------------------- */

static int
sfc7120_intr_setup(sfc7120_softc_t *sc)
{
    device_t dev = sc->dev;
    int      count;
    int      error;

    /* MSI-X table + PBA live in BAR4 on EF10 (16 KB on SFN7322F-R2,
     * "Table in map 0x20[0x0], PBA in map 0x20[0x2000]" per pciconf -lbv).
     * Mirror sfxge_intr_setup_msix: allocate the BAR, then ask for vectors. */
    count = pci_msix_count(dev);
    if (count <= 0) {
        device_printf(dev, "intr_setup: MSI-X unavailable\n");
        return ENXIO;
    }

    sc->msix_bar_rid = PCIR_BAR(4);
    sc->msix_bar_resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
        &sc->msix_bar_rid, RF_ACTIVE);
    if (sc->msix_bar_resource == NULL) {
        device_printf(dev, "intr_setup: BAR4 alloc failed\n");
        return ENOMEM;
    }

    /* One vector for the single interrupting (control) EVQ. Multi-vector
     * arrives when multi-EVQ does. */
    count = 1;
    error = pci_alloc_msix(dev, &count);
    if (error != 0) {
        device_printf(dev, "intr_setup: pci_alloc_msix failed: %d\n", error);
        goto fail_bar;
    }
    if (count != 1) {
        device_printf(dev,
            "intr_setup: pci_alloc_msix granted %d, expected 1\n", count);
        error = ENXIO;
        goto fail_msi;
    }
    sc->msix_nvec = count;

    /* RID 1 = first MSI-X vector (RID 0 is reserved for legacy INTx). */
    sc->irq_res_id = 1;
    sc->irq_resource = bus_alloc_resource_any(dev, SYS_RES_IRQ,
        &sc->irq_res_id, RF_ACTIVE);
    if (sc->irq_resource == NULL) {
        device_printf(dev, "intr_setup: IRQ resource alloc failed\n");
        error = ENOMEM;
        goto fail_msi;
    }

    /* NULL filter: run the handler directly in ithread context, as sfxge
     * does for MSI-X. */
    error = bus_setup_intr(dev, sc->irq_resource,
        INTR_TYPE_NET | INTR_MPSAFE, NULL,
        sfc7120_interrupt_handler, sc, &sc->irq_handle);
    if (error != 0) {
        device_printf(dev, "intr_setup: bus_setup_intr failed: %d\n", error);
        goto fail_irq;
    }

    sc->intr_initialized = true;
    device_printf(dev, "MSI-X: allocated %d vector(s) on BAR4 RID %d\n",
        sc->msix_nvec, sc->irq_res_id);
    return 0;

fail_irq:
    bus_release_resource(dev, SYS_RES_IRQ, sc->irq_res_id, sc->irq_resource);
    sc->irq_resource = NULL;
fail_msi:
    pci_release_msi(dev);
    sc->msix_nvec = 0;
fail_bar:
    bus_release_resource(dev, SYS_RES_MEMORY, sc->msix_bar_rid,
                         sc->msix_bar_resource);
    sc->msix_bar_resource = NULL;
    return error;
}

static void
sfc7120_intr_teardown(sfc7120_softc_t *sc)
{
    device_t dev = sc->dev;

    /* Tear down the ISR first (bus_teardown_intr waits for any in-flight
     * handler and guarantees no further fires), then release the IRQ,
     * MSI-X vectors, and BAR4. Reverse of sfc7120_intr_setup. */
    if (sc->intr_initialized && sc->irq_resource != NULL &&
        sc->irq_handle != NULL) {
        bus_teardown_intr(dev, sc->irq_resource, sc->irq_handle);
        sc->irq_handle = NULL;
    }
    sc->intr_initialized = false;

    if (sc->irq_resource != NULL) {
        bus_release_resource(dev, SYS_RES_IRQ, sc->irq_res_id,
                             sc->irq_resource);
        sc->irq_resource = NULL;
    }

    if (sc->msix_nvec > 0) {
        pci_release_msi(dev);
        sc->msix_nvec = 0;
    }

    if (sc->msix_bar_resource != NULL) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->msix_bar_rid,
                             sc->msix_bar_resource);
        sc->msix_bar_resource = NULL;
    }
}

/* ---------------------------------------------------------------------- */
/* PCI driver registration                                                */
/* ---------------------------------------------------------------------- */

static device_method_t sfc7120_methods[] = {
    DEVMETHOD(device_probe,  sfc7120_fbsd_probe),
    DEVMETHOD(device_attach, sfc7120_fbsd_attach),
    DEVMETHOD(device_detach, sfc7120_fbsd_detach),
    DEVMETHOD_END
};

static driver_t sfc7120_driver = {
    "sfc7120pol",
    sfc7120_methods,
    sizeof(sfc7120_softc_t)
};

DRIVER_MODULE(sfc7120pol, pci, sfc7120_driver, 0, 0);
MODULE_DEPEND(sfc7120pol, pci, 1, 1, 1);
MODULE_DEPEND(sfc7120pol, modmap, 1, 1, 1);
MODULE_VERSION(sfc7120pol, 1);
