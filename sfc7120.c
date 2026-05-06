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
/* DMA helpers — TODO: implement EVQ/TXQ/RXQ ring + buffer allocations.   */
/*                                                                        */
/* The e1000 driver has working bus_dma_tag_create / bus_dmamem_alloc /   */
/* bus_dmamap_load chains for TX and RX in e1000_alloc_tx_dma() and       */
/* e1000_alloc_rx_dma(). The mlx5pol driver wraps the same calls in a     */
/* single mlx5_alloc_dmabuf() helper; that pattern is preferable for any  */
/* driver with more than two DMA buffers (sfc7120 has at least three:     */
/* EVQ, TX desc ring, RX desc ring, plus packet buffers).                 */
/* ---------------------------------------------------------------------- */

static int
sfc7120_alloc_dma_resources(sfc7120_softc_t *sc)
{
    /* TODO: allocate evq_ring, tx_desc_ring, rx_desc_ring, tx_buffer,
     * rx_buffer. Mirror e1000_alloc_tx_dma / e1000_alloc_rx_dma. */
    (void)sc;
    return 0;
}

static void
sfc7120_free_dma_resources(sfc7120_softc_t *sc)
{
    /* TODO: free what alloc allocated. */
    (void)sc;
}

/* ---------------------------------------------------------------------- */
/* Hardware bringup — TODO: MCDI handshake, EVQ_INIT, RXQ_INIT, TXQ_INIT,  */
/* MAC config, link config. EF10 talks to the MC firmware through a       */
/* doorbell at BAR offset SFC7120_REG_MCDB and a DMA-resident message     */
/* buffer; see the FreeBSD `sfxge` driver in the CheriBSD tree for a      */
/* reference implementation.                                              */
/* ---------------------------------------------------------------------- */

static int
sfc7120_hw_init(sfc7120_softc_t *sc)
{
    /* TODO: PCIe FLR, MCDI version handshake, GET_BOARD_CFG, GET_MAC_ADDR,
     * INIT_EVQ, INIT_RXQ, INIT_TXQ, MAC_RECONFIG, link up. */
    (void)sc;
    return 0;
}

static void
sfc7120_hw_teardown(sfc7120_softc_t *sc)
{
    /* TODO: link down, FINI_TXQ, FINI_RXQ, FINI_EVQ, MCDI shutdown. */
    (void)sc;
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
    sfc7120_softc_t *sc = device_get_softc(dev);
    int error;

    sc->dev = dev;
    sc->dying = false;
    sc->device_attached = false;
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
    sfc7120_free_dma_resources(sc);
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

    sfc7120_free_dma_resources(sc);
    sfc7120_hw_teardown(sc);

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
