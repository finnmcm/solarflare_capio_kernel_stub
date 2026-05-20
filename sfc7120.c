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

    /* Poll GET_LINK until the link comes up, with a 3-second cap. sfxge
     * doesn't poll — it waits for a firmware-generated LINKCHANGE EVQ
     * event — but our interrupt handler is still TODO, so this poll gives
     * us a usable bringup log line. A link-down outcome is non-fatal
     * (cable could simply be unplugged).
     * TODO: remove this poll once the EVQ event handler is wired. */
    {
        int i;
        for (i = 0; i < 30; i++) {
            (void)sfc7120_mcdi_get_link(sc);
            if (sc->link_up)
                break;
            pause("sfclnk", hz / 10); /* 100 ms */
        }
        if (!sc->link_up) {
            device_printf(sc->dev,
                "hw_init: link still DOWN after %d ms; continuing\n",
                i * 100);
        }
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


    /* 3. DMA buffer allocation. */
    device_printf(dev, "TRACE: calling alloc_dma_resources\n");
    error = sfc7120_alloc_dma_resources(sc);
    if (error != 0) {
        device_printf(dev, "alloc_dma_resources failed: %d\n", error);
        goto fail_hw;
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
     * memory. Can't fall through to fail_hw or we'd call hw_teardown twice. */
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
    case SFC7120_TX:
        /* TODO: validate user cap (tag/perm/length) and copyin into the
         * TX descriptor ring. See e1000_fbsd_raw_transmit() for the
         * cheri_gettag / cheri_getperm / cheri_getlen pattern. */
        return ENOSYS;
    case SFC7120_RX:
        /* TODO: copyout received packets. */
        return ENOSYS;
    default:
        return ENOTTY;
    }
}

/* ---------------------------------------------------------------------- */
/* Interrupt handling                                                     */
/* ---------------------------------------------------------------------- */

static void
sfc7120_interrupt_handler(void *arg)
{
    /* TODO: read EVQ, deferred-process events into rx_task. The EF10
     * event queue is a circular DMA buffer of 64-bit events; software
     * advances a read pointer and writes a doorbell to acknowledge. */
    sfc7120_softc_t *sc = (sfc7120_softc_t *)arg;
    (void)sc;
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
