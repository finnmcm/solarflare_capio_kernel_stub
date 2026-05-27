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
static void sfc7120_rx_task_handler(void *context, int pending);
static int  sfc7120_intr_setup(sfc7120_softc_t *sc);
static void sfc7120_intr_teardown(sfc7120_softc_t *sc);
static void sfc7120_rx_post_initial(sfc7120_softc_t *sc);

/* EF10 RX prefix: 14-byte pseudo-header prepended when INIT_RXQ FLAG_PREFIX
 * is set (our flags=0x300 includes bit 8).  Layout per ef10_rx.c:737:
 *   +0x00  hash32 (LE)
 *   +0x04  outer VLAN (BE)
 *   +0x06  inner VLAN (BE)
 *   +0x08  packet length (LE, excl. prefix; 0 = cut-through mode)
 *   +0x0a  MAC timestamp (LE) */
#define SFC7120_EF10_RX_PREFIX_LEN  14

/* EF10 RX write-pointer alignment requirement (EF10_RX_WPTR_ALIGN in
 * ef10_impl.h).  The RX doorbell wptr must be a multiple of this. */
#define SFC7120_RX_WPTR_ALIGN       8

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

    /* TX descriptor ring: driver writes outgoing packet descriptors here. */
    error = sfc7120_alloc_dmabuf(sc->dev,
                                 &sc->tx_desc_dtag, &sc->tx_desc_dmamap,
                                 &sc->tx_desc_ring, &sc->tx_desc_ring_paddr,
                                 txd_size, 4096, "TX desc ring");
    if (error != 0)
        goto fail_evq;

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
    sfc7120_free_dmabuf(&sc->evq_dtag, &sc->evq_dmamap,
                        &sc->evq_ring, &sc->evq_ring_paddr);
}

/* ---------------------------------------------------------------------- */
/* RX descriptor ring initialisation                                       */
/* ---------------------------------------------------------------------- */

/*
 * Post (SFC7120_NUM_RX_DESC - SFC7120_RX_WPTR_ALIGN) = 504 descriptors so
 * the NIC has a deep supply of buffers on first attach.  We intentionally stop
 * one alignment batch short of a full ring so the initial wptr is 504 (not 0)
 * — a wptr of 0 after INIT_RXQ could be ambiguous.
 *
 * Each descriptor is an 8-byte little-endian qword:
 *   bits[47:0]  = physical buffer address (48-bit)
 *   bits[61:48] = buffer byte count (14-bit; BYTE_CNT field)
 *   bits[63:62] = RESERVED (0)
 * (ESF_DZ_RX_KER_* in efx_regs_ef10.h; ef10_rx.c:ef10_rx_qpost.)
 *
 * The wptr is written to SFC7120_REG_RX_DESC_DBL (ER_DZ_RX_DESC_UPD_REG_OFST
 * = 0x0830, function-local VI 0), value = produced & (ring_size - 1).
 * EF10 requires this to be aligned to EF10_RX_WPTR_ALIGN (8).
 */
static void
sfc7120_rx_post_initial(sfc7120_softc_t *sc)
{
    uint64_t *ring = (uint64_t *)sc->rx_desc_ring;
    int n = SFC7120_NUM_RX_DESC - SFC7120_RX_WPTR_ALIGN; /* 504 */

    for (int i = 0; i < n; i++) {
        bus_addr_t paddr = sc->rx_buffer_paddr +
                           (bus_addr_t)i * SFC7120_RX_BUFFER_SIZE;
        uint64_t desc = (paddr & 0x0000ffffffffffffULL) |
                        ((uint64_t)(SFC7120_RX_BUFFER_SIZE & 0x3fff) << 48);
        ring[i] = htole64(desc);
    }
    bus_dmamap_sync(sc->rx_desc_dtag, sc->rx_desc_dmamap, BUS_DMASYNC_PREWRITE);

    sc->rx_pushed = n;
    /* wptr must be aligned to SFC7120_RX_WPTR_ALIGN; n=504 is already aligned. */
    uint32_t wptr = (uint32_t)(n & (SFC7120_NUM_RX_DESC - 1)); /* 504 */
    SFC7120_WRITE_REG(sc, SFC7120_REG_RX_DESC_DBL, wptr);
    device_printf(sc->dev,
        "RX: posted %d initial descriptors, wptr=%u\n", n, wptr);
}

/* ---------------------------------------------------------------------- */
/* Hardware bringup. Stops here at MCDI + identity queries — EVQ/RXQ/TXQ  */
/* init needs DMA rings, so it lives in a future sfc7120_hw_start() that  */
/* runs after sfc7120_alloc_dma_resources(). The MCDI plumbing itself is  */
/* in sfc7120_mcdi.c.                                                     */
/* ---------------------------------------------------------------------- */

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

    /* MAC + PHY bring-up. Without these the MAC stays drained and the PHY
     * never negotiates a link, so the host-side queues we program in
     * sfc7120_fbsd_attach would never see traffic. Mirrors sfxge
     * efx_port_init (efx_port.c:37-99). */
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

    /* Seed link state with a single GET_LINK so any consumer that reads
     * sc->link_* before the first LINKCHANGE event arrives sees something
     * coherent. Subsequent state transitions are delivered as LINKCHANGE
     * events on the EVQ and decoded by sfc7120_interrupt_handler. A link-
     * down outcome here is non-fatal — an unwired cage stays down until
     * a peer comes up and the firmware fires the event. */
    (void)sfc7120_mcdi_get_link(sc);
    if (!sc->link_up) {
        device_printf(sc->dev,
            "hw_init: initial link DOWN; waiting for LINKCHANGE\n");
    }

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
    (void)sfc7120_mcdi_fini_txq(sc, 0);
    /* Remove the RX filter before tearing down RXQ 0 it points at — the
     * filter references the RX queue, so it must go first. Self-guards on
     * sc->rx_filter_inserted, so this is a no-op if insert never ran. */
    (void)sfc7120_mcdi_filter_remove(sc);
    (void)sfc7120_mcdi_fini_rxq(sc, 0);
    (void)sfc7120_mcdi_fini_evq(sc, 0);
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
    pci_enable_busmaster(dev);

    /* 2. Hardware bringup (MCDI handshake, queue init). */
    device_printf(dev, "TRACE: calling hw_init\n");
    error = sfc7120_hw_init(sc);
    if (error != 0) {
        device_printf(dev, "hw_init failed: %d\n", error);
        goto fail_bar;
    }
    device_printf(dev, "TRACE: hw_init done\n");

    /* 2a. MSI-X allocation. Must run before INIT_EVQ so the firmware can
     *     deliver events into a valid vector the moment the queue is armed.
     *     A spurious interrupt before evq_initialized=true is harmless — the
     *     ISR bails on the !evq_initialized guard. */
    device_printf(dev, "TRACE: calling intr_setup\n");
    error = sfc7120_intr_setup(sc);
    if (error != 0) {
        device_printf(dev, "intr_setup failed: %d\n", error);
        goto fail_hw;
    }
    device_printf(dev, "TRACE: intr_setup done\n");

    /* 3. DMA buffer allocation. */
    device_printf(dev, "TRACE: calling alloc_dma_resources\n");
    error = sfc7120_alloc_dma_resources(sc);
    if (error != 0) {
        device_printf(dev, "alloc_dma_resources failed: %d\n", error);
        goto fail_intr;
    }
    device_printf(dev, "TRACE: alloc_dma_resources done\n");

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
    device_printf(dev, "TRACE: init_evq done\n");

    /* INITIALIZING RX QUEUE. instance and target_evq are function-local
     * indices (0..vi_count-1), matching EVQ instance 0 above. */
    device_printf(dev, "TRACE: calling init_rxq\n");
    error = sfc7120_mcdi_init_rxq(sc, 0, 0, sc->rx_desc_ring_paddr,
                                  SFC7120_NUM_RX_DESC);
    if (error != 0) {
        device_printf(dev, "init_rxq failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: init_rxq done\n");

    /* INITIALIZING TX QUEUE */
    device_printf(dev, "TRACE: calling init_txq\n");
    error = sfc7120_mcdi_init_txq(sc, 0, 0, sc->tx_desc_ring_paddr,
                                  SFC7120_NUM_TX_DESC);
    if (error != 0) {
        device_printf(dev, "init_txq failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: init_txq done\n");

    /* INSERT RX FILTER. RXQ 0 must already exist (above) — the filter names
     * RX queue 0 as its destination. Without a filter the MC drops every RX
     * frame before it reaches the ring, so this is the last blocker for RX
     * delivery. Treated like the other queue-init steps: log and bail to
     * fail_dma on error rather than half-attaching. */
    device_printf(dev, "TRACE: calling filter_insert\n");
    error = sfc7120_mcdi_filter_insert(sc);
    if (error != 0) {
        device_printf(dev, "filter_insert failed: %d\n", error);
        goto fail_dma;
    }
    device_printf(dev, "TRACE: filter_insert done\n");

    /* Initialize TX ring counters before posting any descriptors.
     * tx_descriptors_free MUST be set; it defaults to 0 from the zero-init
     * softc which would block all transmits. */
    sc->tx_head             = 0;
    sc->tx_tail             = 0;
    sc->tx_descriptors_free = SFC7120_NUM_TX_DESC;
    sc->rx_head             = 0;
    sc->rx_pending_slot     = 0;
    sc->rx_event_bytes      = 0;

    /* Post the initial RX descriptor batch so the NIC has buffers ready.
     * Sets sc->rx_pushed — must come AFTER the zeroing above. */
    sfc7120_rx_post_initial(sc);

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
    /* FINI the queues (inside hw_teardown) BEFORE freeing the DMA rings the
     * firmware is writing into — otherwise the NIC keeps DMAing into freed
     * memory. Then tear down the ISR (drain taskqueue + bus_teardown_intr).
     * Can't fall through to fail_intr/fail_hw or we'd call those twice. */
    sfc7120_hw_teardown(sc);
    sfc7120_free_dma_resources(sc);
    sfc7120_intr_teardown(sc);
    goto fail_bar;
fail_intr:
    sfc7120_intr_teardown(sc);
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

    /* Firmware teardown (FINI_TXQ/RXQ/EVQ, VADAPTOR_FREE, FREE_VIS, ...)
     * MUST happen before tearing down the ISR or freeing the DMA rings —
     * otherwise the NIC keeps DMA-writing events into freed memory and
     * keeps firing the vector at a torn-down handler. After hw_teardown
     * the firmware is silent, so the ISR can be safely removed; only
     * then is it safe to free the EVQ DMA buffer the ISR was reading. */
    sfc7120_hw_teardown(sc);
    sfc7120_intr_teardown(sc);
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
    case SFC7120_TX: {
        sfc7120_tx_req_t *req = (sfc7120_tx_req_t *)data;

        /* Validate the user CHERI capability (tag + LOAD permission + bounds).
         * Mirrors e1000pol.c:1314-1326. */
        if (!cheri_gettag(req->tx_buf_addr)) {
            device_printf(sc->dev, "TX: invalid capability tag\n");
            return EINVAL;
        }
        if (!(cheri_getperm(req->tx_buf_addr) & CHERI_PERM_LOAD)) {
            device_printf(sc->dev, "TX: no LOAD permission on cap\n");
            return EACCES;
        }
        if (req->length == 0 || req->length > SFC7120_TX_BUFFER_SIZE) {
            device_printf(sc->dev, "TX: bad length %zu\n", req->length);
            return EINVAL;
        }

        SFC7120_LOCK(sc);

        if (sc->tx_descriptors_free == 0) {
            SFC7120_UNLOCK(sc);
            return ENOBUFS;
        }

        int tail = sc->tx_tail;
        uint8_t *kbuf = (uint8_t *)sc->tx_buffer +
                        (size_t)tail * SFC7120_TX_BUFFER_SIZE;
        bus_addr_t paddr = sc->tx_buffer_paddr +
                           (bus_addr_t)tail * SFC7120_TX_BUFFER_SIZE;

        int error = copyin(req->tx_buf_addr, kbuf, req->length);
        if (error != 0) {
            SFC7120_UNLOCK(sc);
            device_printf(sc->dev, "TX: copyin failed: %d\n", error);
            return error;
        }
        bus_dmamap_sync(sc->tx_buffer_dtag, sc->tx_buffer_dmamap,
                        BUS_DMASYNC_PREWRITE);

        /* Build the 8-byte EF10 TX kernel descriptor.
         *   bits[47:0]  = BUF_ADDR  (48-bit physical address)
         *   bits[61:48] = BYTE_CNT  (14-bit byte count)
         *   bit [62]    = CONT=0    (last/only segment)
         *   bit [63]    = TYPE=0    (kernel descriptor, not option)
         * (ESF_DZ_TX_KER_* in efx_regs_ef10.h.) */
        uint64_t *ring = (uint64_t *)sc->tx_desc_ring;
        uint64_t desc = (paddr & 0x0000ffffffffffffULL) |
                        ((uint64_t)(req->length & 0x3fff) << 48);
        ring[tail] = htole64(desc);
        bus_dmamap_sync(sc->tx_desc_dtag, sc->tx_desc_dmamap,
                        BUS_DMASYNC_PREWRITE);

        sc->tx_descriptors_free--;
        sc->tx_tail = (tail + 1) & (SFC7120_NUM_TX_DESC - 1);

        /* Push TX write pointer via the WRITED2 path.
         * ERF_DZ_TX_DESC_WPTR lives at LBN=64 (dword[2]) of TX_DESC_UPD.
         * EFX_BAR_VI_WRITED2 adds 2*sizeof(dword)=8 to the base offset, so
         * we write to SFC7120_REG_TX_WPTR_DBL = 0x0a10 + 8 = 0x0a18.
         * Value = new wptr (12-bit), no VLD bit. */
        uint32_t wptr = (uint32_t)sc->tx_tail & 0xfffu;
        SFC7120_WRITE_REG(sc, SFC7120_REG_TX_WPTR_DBL, wptr);

        device_printf(sc->dev,
            "TX: slot=%d len=%zu paddr=0x%lx wptr=%u free_left=%d\n",
            tail, req->length, (unsigned long)paddr, wptr,
            sc->tx_descriptors_free);

        req->status = 0;
        SFC7120_UNLOCK(sc);
        return 0;
    }

    case SFC7120_RX: {
        sfc7120_rx_req_t *req = (sfc7120_rx_req_t *)data;

        /* Wait for an RX frame.  The ISR (via rx_task_handler) sets
         * rx_received and calls wakeup(&sc->rx_received).  Block up to 5 s.
         * Lock order: rx_mtx only here; sc_mtx acquired separately below for
         * the descriptor refill to avoid rx_mtx -> sc_mtx inversion. */
        SFC7120_RX_LOCK(sc);
        for (int i = 0; i < 5 && !sc->rx_received && !sc->dying; i++) {
            int ret = msleep(sc, &sc->rx_mtx,
                             PZERO | PCATCH, "sfc7120rx", hz);
            if (ret == EINTR || ret == ERESTART) {
                SFC7120_RX_UNLOCK(sc);
                return EINTR;
            }
        }
        if (!sc->rx_received) {
            SFC7120_RX_UNLOCK(sc);
            return ETIMEDOUT;
        }

        /* Snapshot the pending-frame state and clear the flag. */
        int slot         = sc->rx_pending_slot;
        uint32_t rbytes  = sc->rx_event_bytes;
        sc->rx_received  = false;
        SFC7120_RX_UNLOCK(sc);

        /* Sync the RX buffer for CPU reads. */
        bus_dmamap_sync(sc->rx_buffer_dtag, sc->rx_buffer_dmamap,
                        BUS_DMASYNC_POSTREAD);

        /* Decode the 14-byte EF10 prefix (ef10_rx.c:737).
         * Prefix offset +8 holds the frame length (LE uint16, excl. prefix).
         * If 0, firmware was in cut-through mode — use event bytes instead. */
        uint8_t *rxbuf = (uint8_t *)sc->rx_buffer +
                         (size_t)slot * SFC7120_RX_BUFFER_SIZE;
        uint16_t plen = (uint16_t)rxbuf[8] | ((uint16_t)rxbuf[9] << 8);
        if (plen == 0) {
            /* Cut-through mode: derive length from event bytes minus prefix. */
            plen = (rbytes > SFC7120_EF10_RX_PREFIX_LEN)
                   ? (uint16_t)(rbytes - SFC7120_EF10_RX_PREFIX_LEN)
                   : 0;
        }

        device_printf(sc->dev,
            "RX: slot=%d rbytes=%u plen=%u\n", slot, rbytes, (unsigned)plen);

        if (plen == 0 ||
            plen > SFC7120_RX_BUFFER_SIZE - SFC7120_EF10_RX_PREFIX_LEN) {
            req->status = 1;
            req->error  = 1;
            req->length_received = 0;
            return EINVAL;
        }

        /* Copyout the frame data (past the prefix) into the caller's buffer. */
        uint8_t *frame = rxbuf + SFC7120_EF10_RX_PREFIX_LEN;
        int error = copyout(frame, req->raw_buffer, plen);
        if (error != 0) {
            device_printf(sc->dev, "RX: copyout failed: %d\n", error);
            req->status = 1;
            req->error  = 1;
            return error;
        }

        req->descriptors[0].start_ptr  = req->raw_buffer;
        req->descriptors[0].length     = plen;
        req->descriptor_length         = 1;
        req->length_received           = plen;
        req->status                    = 0;
        req->error                     = 0;

        /* Refill the consumed slot and advance the producer pointer.
         * We post the next ring slot (rx_pushed % ring_size) with the
         * corresponding physical buffer.  The wptr doorbell is written only
         * when the new producer value is aligned to SFC7120_RX_WPTR_ALIGN. */
        SFC7120_LOCK(sc);
        int slot_to_post = sc->rx_pushed & (SFC7120_NUM_RX_DESC - 1);
        bus_addr_t rpaddr = sc->rx_buffer_paddr +
                            (bus_addr_t)slot_to_post * SFC7120_RX_BUFFER_SIZE;
        uint64_t *rring = (uint64_t *)sc->rx_desc_ring;
        uint64_t rdesc = (rpaddr & 0x0000ffffffffffffULL) |
                         ((uint64_t)(SFC7120_RX_BUFFER_SIZE & 0x3fff) << 48);
        rring[slot_to_post] = htole64(rdesc);
        bus_dmamap_sync(sc->rx_desc_dtag, sc->rx_desc_dmamap,
                        BUS_DMASYNC_PREWRITE);

        sc->rx_pushed++;
        /* Push doorbell only on aligned boundaries to satisfy EF10 requirement.
         * Unaligned pushes are silently ignored; aligning avoids hardware-side
         * surprises but wastes at most (SFC7120_RX_WPTR_ALIGN - 1) slots. */
        if ((sc->rx_pushed & (SFC7120_RX_WPTR_ALIGN - 1)) == 0) {
            uint32_t rwptr = (uint32_t)(sc->rx_pushed &
                                        (SFC7120_NUM_RX_DESC - 1));
            SFC7120_WRITE_REG(sc, SFC7120_REG_RX_DESC_DBL, rwptr);
            device_printf(sc->dev,
                "RX refill: slot_posted=%d rx_pushed=%d wptr=%u\n",
                slot_to_post, sc->rx_pushed, rwptr);
        }
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

/* EF10 event format constants. Vendored from efx_regs_ef10.h / efx_regs_mcdi.h
 * — only the fields we actually parse in the ISR. Extend as we wire up more
 * event types. */
#define SFC7120_EV_CODE_LBN          60     /* top 4 bits of the 64-bit qword */
#define SFC7120_EV_CODE_RX           0
#define SFC7120_EV_CODE_TX           2
#define SFC7120_EV_CODE_DRIVER       5
#define SFC7120_EV_CODE_MCDI         12

/* MCDI sub-event layout (bits inside the 64-bit qword). */
#define SFC7120_MCDI_EVENT_CODE_LBN  44     /* MCDI subcode field, 8 bits */
#define SFC7120_MCDI_EV_LINKCHANGE   0x4

/* LINKCHANGE payload layout (low 32 bits of the qword). */
#define SFC7120_LC_SPEED_SHIFT       16     /* 4 bits */
#define SFC7120_LC_FCNTL_SHIFT       20     /* 4 bits */
#define SFC7120_LC_FLAGS_SHIFT       24     /* 8 bits */
#define SFC7120_LC_FLAG_LINK_UP      (1u << 0)
#define SFC7120_LC_FLAG_FDX          (1u << 1)

/* EVQ_RPTR doorbell: bits 0..14 are the read-pointer (mod 2^15). sfxge
 * writes `count & ee_mask` (i.e. just the masked rptr, VLD bit clear) —
 * mirror that exactly. */
#define SFC7120_EVQ_RPTR_MASK        0x7fffu

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
    sfc7120_softc_t *sc = (sfc7120_softc_t *)arg;
    volatile uint64_t *ring;
    int processed = 0;
    bool wake_rx_task = false;

    if (sc == NULL || sc->evq_ring == NULL)
        return;
    if (sc->dying || !sc->evq_initialized)
        return;

    ring = (volatile uint64_t *)sc->evq_ring;

    SFC7120_LOCK(sc);

    /* Sync the EVQ for CPU reads — NIC writes events via DMA. The buffer
     * is BUS_DMA_COHERENT so this is a barrier hint, not a cache flush. */
    bus_dmamap_sync(sc->evq_dtag, sc->evq_dmamap,
                    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

    for (;;) {
        uint64_t ev = ring[sc->evq_read_ptr];
        uint32_t lo = (uint32_t)(ev & 0xffffffffu);
        uint32_t hi = (uint32_t)(ev >> 32);

        /* Presence: hardware fills slots with all-ones at init; we clear
         * to all-zeros on consume. Both patterns mean "not a real event"
         * — see EFX_EV_PRESENT in efx_ev.c. */
        if ((lo == 0xffffffffu && hi == 0xffffffffu) ||
            (lo == 0 && hi == 0))
            break;

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
                uint32_t mbps = sfc7120_linkchange_speed_mbps(scode);

                sc->link_up         = up;
                sc->full_duplex     = fdx;
                sc->link_speed_mbps = mbps;
                sc->link_fcntl      = fcntl;

                device_printf(sc->dev,
                    "EVQ LINKCHANGE: link=%s speed=%u Mbps %s fcntl=%u "
                    "flags=%#x\n",
                    up ? "UP" : "DOWN", mbps, fdx ? "FDX" : "HDX",
                    fcntl, flags);
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
        case SFC7120_EV_CODE_RX: {
            /* ESF_DZ_RX_BYTES: LBN=0 WIDTH=14 — byte count including the
             * 14-byte EF10 prefix (FLAG_PREFIX was set in INIT_RXQ flags). */
            uint32_t rbytes = lo & 0x3fffu;
            device_printf(sc->dev,
                "EVQ RX_EV: bytes=%u slot=%d\n", rbytes, sc->rx_head);
            sc->rx_event_bytes  = rbytes;
            sc->rx_pending_slot = sc->rx_head;
            sc->rx_head = (sc->rx_head + 1) & (SFC7120_NUM_RX_DESC - 1);
            wake_rx_task = true;
            break;
        }
        case SFC7120_EV_CODE_TX: {
            /* ESF_DZ_TX_DESCR_INDX: LBN=0 WIDTH=16 — index of the last TX
             * descriptor completed by the NIC. */
            uint32_t descr_indx = lo & 0xffffu;
            int new_head = ((int)descr_indx + 1) & (SFC7120_NUM_TX_DESC - 1);
            /* Credit back descriptors between tx_head and new_head (wrap-safe). */
            int freed = (new_head - sc->tx_head + SFC7120_NUM_TX_DESC)
                        & (SFC7120_NUM_TX_DESC - 1);
            if (freed == 0)
                freed = 1; /* single-descriptor batch always frees one */
            sc->tx_head = new_head;
            sc->tx_descriptors_free += freed;
            if (sc->tx_descriptors_free > SFC7120_NUM_TX_DESC)
                sc->tx_descriptors_free = SFC7120_NUM_TX_DESC;
            device_printf(sc->dev,
                "EVQ TX_EV: descr_indx=%u new_head=%d freed=%d free_now=%d\n",
                descr_indx, new_head, freed, sc->tx_descriptors_free);
            break;
        }
        default:
            device_printf(sc->dev,
                "EVQ unknown code=%#x ev=%#016jx\n",
                code, (uintmax_t)ev);
            break;
        }

        /* Mark slot consumed. Next pass sees all-zeros = "not present"
         * until the NIC overwrites with a fresh event. */
        ring[sc->evq_read_ptr] = 0;
        sc->evq_read_ptr =
            (sc->evq_read_ptr + 1) & (SFC7120_NUM_EVQ_ENTRY - 1);
        processed++;

        if (processed >= SFC7120_NUM_EVQ_ENTRY) {
            /* Defensive: if a bug ever produces a wraparound full of
             * "present" markers we don't want to spin forever in the
             * ISR. */
            device_printf(sc->dev,
                "EVQ: processed full ring without empty marker\n");
            break;
        }
    }

    if (processed > 0) {
        bus_dmamap_sync(sc->evq_dtag, sc->evq_dmamap,
                        BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
        SFC7120_WRITE_REG(sc, SFC7120_REG_EVQ_RPTR_DBL,
            (uint32_t)sc->evq_read_ptr & SFC7120_EVQ_RPTR_MASK);
    }

    SFC7120_UNLOCK(sc);

    if (wake_rx_task && sc->rx_taskqueue != NULL)
        taskqueue_enqueue(sc->rx_taskqueue, &sc->rx_task);
}

static void
sfc7120_rx_task_handler(void *context, int pending)
{
    sfc7120_softc_t *sc = (sfc7120_softc_t *)context;
    SFC7120_RX_LOCK(sc);
    sc->rx_received = true;
    wakeup(sc);                   /* wake SFC7120_RX msleep() callers */
    selwakeuppri(&sc->selinfo, PZERO + 1);
    SFC7120_RX_UNLOCK(sc);
}

/* ---------------------------------------------------------------------- */
/* MSI-X allocation / teardown                                            */
/* ---------------------------------------------------------------------- */

static int
sfc7120_intr_setup(sfc7120_softc_t *sc)
{
    device_t dev = sc->dev;
    int count;
    int error;

    /* Initialize the RX taskqueue + task. The ISR enqueues rx_task on
     * RX_EV; rx_task_handler signals waiting selrecord/poll consumers. */
    TASK_INIT(&sc->rx_task, 0, sfc7120_rx_task_handler, sc);
    sc->rx_taskqueue = taskqueue_create("sfc7120_rx", M_WAITOK,
        taskqueue_thread_enqueue, &sc->rx_taskqueue);
    if (sc->rx_taskqueue == NULL) {
        device_printf(dev, "intr_setup: taskqueue_create failed\n");
        return ENOMEM;
    }
    error = taskqueue_start_threads(&sc->rx_taskqueue, 1, PI_NET,
        "%s rx", device_get_nameunit(dev));
    if (error != 0) {
        device_printf(dev,
            "intr_setup: taskqueue_start_threads failed: %d\n", error);
        goto fail_tq;
    }

    /* MSI-X table + PBA live in BAR4 on EF10 (16 KB on SFN7322F-R2,
     * `Table in map 0x20[0x0], PBA in map 0x20[0x2000]` per pciconf -lbv).
     * Mirror sfxge_intr_setup_msix: allocate the BAR, then ask for vectors. */
    count = pci_msix_count(dev);
    if (count <= 0) {
        device_printf(dev, "intr_setup: MSI-X unavailable\n");
        error = ENXIO;
        goto fail_tq;
    }

    sc->msix_bar_rid = PCIR_BAR(4);
    sc->msix_bar_resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
        &sc->msix_bar_rid, RF_ACTIVE);
    if (sc->msix_bar_resource == NULL) {
        device_printf(dev, "intr_setup: BAR4 alloc failed\n");
        error = ENOMEM;
        goto fail_tq;
    }

    /* Single vector for the single EVQ we currently program. Multi-vector
     * lands when multi-EVQ does. */
    count = 1;
    error = pci_alloc_msix(dev, &count);
    if (error != 0) {
        device_printf(dev,
            "intr_setup: pci_alloc_msix failed: %d\n", error);
        goto fail_bar;
    }
    if (count != 1) {
        device_printf(dev,
            "intr_setup: pci_alloc_msix granted %d, expected 1\n", count);
        error = ENXIO;
        goto fail_msi;
    }
    sc->msix_nvec = count;

    /* RID 1 = first MSI-X vector (RID 0 is reserved for INTx). Not
     * RF_SHAREABLE — MSI-X vectors are per-handler. */
    sc->irq_res_id = 1;
    sc->irq_resource = bus_alloc_resource_any(dev, SYS_RES_IRQ,
        &sc->irq_res_id, RF_ACTIVE);
    if (sc->irq_resource == NULL) {
        device_printf(dev, "intr_setup: IRQ resource alloc failed\n");
        error = ENOMEM;
        goto fail_msi;
    }

    /* No filter — sfxge runs MSI-X in ithread context directly. */
    error = bus_setup_intr(dev, sc->irq_resource,
        INTR_TYPE_NET | INTR_MPSAFE, NULL,
        sfc7120_interrupt_handler, sc, &sc->irq_handle);
    if (error != 0) {
        device_printf(dev,
            "intr_setup: bus_setup_intr failed: %d\n", error);
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
fail_tq:
    if (sc->rx_taskqueue != NULL) {
        taskqueue_free(sc->rx_taskqueue);
        sc->rx_taskqueue = NULL;
    }
    return error;
}

static void
sfc7120_intr_teardown(sfc7120_softc_t *sc)
{
    device_t dev = sc->dev;

    /* Order: drain taskqueue first (any RX_EV-enqueued task could still
     * be in-flight), then tear down the ISR (guarantees no further
     * enqueues), then release MSI/BAR. Mirror of mlx5pol's detach. */
    if (sc->rx_taskqueue != NULL) {
        taskqueue_drain(sc->rx_taskqueue, &sc->rx_task);
        taskqueue_free(sc->rx_taskqueue);
        sc->rx_taskqueue = NULL;
    }

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
