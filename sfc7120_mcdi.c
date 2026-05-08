/*
 * sfc7120_mcdi.c — kernel-side MCDI plumbing for the Solarflare EF10
 * (Huntington / SFN7xxx) NIC.
 *
 * Mirrors the synchronous-poll MCDI transport used by FreeBSD sfxge
 * (sfxge_mcdi.c -> ef10_mcdi.c). We carry one DMA-coherent mailbox per
 * device and serialize all exchanges behind sc->mcdi_mtx. EF10 has no
 * separate "send" register: writing the high half of the mailbox physical
 * address to MC_DB_HWRD is what posts the command.
 *
 * The wire format and command catalogue (MC_CMD_*) live in
 * ref_efx_regs_mcdi.h. We avoid pulling in the full EFX abstraction layer
 * because this driver doesn't share its softc / efsys layout.
 */

#include "capio.h"
#include "sfc7120.h"
#include "sfc7120_mcdi.h"
#include "sfc7120_mmio.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/malloc.h>
#include <sys/endian.h>
#include <machine/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

/* ---------------------------------------------------------------------- */
/* EF10 register offsets used by the kernel-only MCDI transport.          */
/*                                                                        */
/* These come from cheri/cheribsd/sys/dev/sfxge/common/efx_regs_ef10.h    */
/* and are deliberately defined here (rather than in sfc7120_mmio.h)      */
/* because the MC doorbell pair MUST NOT be exposed to userspace through  */
/* the slice manifest — issuing arbitrary MCDI commands from userspace   */
/* would defeat the CAPIO safety boundary.                                */
/* ---------------------------------------------------------------------- */

#define EF10_REG_BIU_HW_REV_ID          0x00000000  /* read 0xeb14face */
#define EF10_REG_BIU_MC_SFT_STATUS      0x00000010  /* MC reboot status */
#define EF10_REG_MC_DB_LWRD             0x00000200  /* doorbell pair, low offset  */
#define EF10_REG_MC_DB_HWRD             0x00000204  /* doorbell pair, high offset */
#define EF10_REG_BIU_HW_REV_ID_RESET    0xeb14faceu /* expected reset value */

/* ---------------------------------------------------------------------- */
/* MCDI v1 wire-format helpers.                                           */
/* ---------------------------------------------------------------------- */

/*
 * MCDI v1 vs v2 framing:
 *   - v1 has a single 32-bit header word with a 7-bit cmd field and 8-bit
 *     datalen — opcodes 0..0x7E and payloads up to 252 bytes.
 *   - v2 escape: v1 header carries cmd=MC_CMD_V2_EXTN (0x7F) and datalen=0.
 *     A second 32-bit "extended header" follows at offset 4 carrying the
 *     real 15-bit cmd and 10-bit length. The actual payload starts at
 *     offset 8. Used for opcodes >= 0x7F or payloads > 252 bytes.
 *   sfxge: efx_mcdi.c:296-327 picks the framing based on
 *   MC_CMD_CMD_SPACE_ESCAPE_7 (0x7F) and MCDI_CTL_SDU_LEN_MAX_V1 (252).
 */
#define MCDI_HEADER_LEN              4u   /* v1 header word size */
#define MCDI_V2_HEADER_LEN           8u   /* v1 + v2 ext header */
#define MCDI_PAYLOAD_LEN_MAX_V1      0xfcu
#define MC_CMD_V2_EXTN               0x7fu

#define MCDI_HDR_CODE_SHIFT          0
#define MCDI_HDR_CODE_MASK           0x7fu
#define MCDI_HDR_RESYNC_SHIFT        7
#define MCDI_HDR_DATALEN_SHIFT       8
#define MCDI_HDR_DATALEN_MASK        0xffu
#define MCDI_HDR_SEQ_SHIFT           16
#define MCDI_HDR_SEQ_MASK            0x0fu
#define MCDI_HDR_NOT_EPOCH_SHIFT     21
#define MCDI_HDR_ERROR_SHIFT         22
#define MCDI_HDR_RESPONSE_SHIFT      23

/* v2 extended header (offset 4): cmd[14:0], len[25:16] */
#define MCDI_V2_EXT_CMD_SHIFT        0
#define MCDI_V2_EXT_CMD_MASK         0x7fffu
#define MCDI_V2_EXT_LEN_SHIFT        16
#define MCDI_V2_EXT_LEN_MASK         0x3ffu

/* MC_CMD opcodes / message lengths used here. Values come from
 * ref_efx_regs_mcdi.h. */
#define MC_CMD_GET_VERSION                0x08
#define MC_CMD_GET_VERSION_OUT_LEN        32
#define MC_CMD_GET_VERSION_OUT_VERSION_OFST 24

#define MC_CMD_DRV_ATTACH                 0x1c
#define MC_CMD_DRV_ATTACH_IN_LEN          12
#define MC_CMD_DRV_ATTACH_IN_NEW_STATE_OFST  0
#define MC_CMD_DRV_ATTACH_IN_UPDATE_OFST     4
#define MC_CMD_DRV_ATTACH_IN_FIRMWARE_ID_OFST 8
#define MC_CMD_DRV_ATTACH_EXT_OUT_LEN     8
#define MC_CMD_DRV_ATTACH_OUT_LEN         4
#define MC_CMD_DRV_ATTACH_EXT_OUT_FUNC_FLAGS_OFST 4
#define MC_CMD_FW_DONT_CARE               0xffffffff

#define MC_CMD_GET_MAC_ADDRESSES          0x55
#define MC_CMD_GET_MAC_ADDRESSES_OUT_LEN  16
#define MC_CMD_GET_MAC_ADDRESSES_OUT_BASE_OFST 0

#define MC_CMD_ALLOC_VIS                  0x8b
#define MC_CMD_ALLOC_VIS_IN_LEN           8
#define MC_CMD_ALLOC_VIS_IN_MIN_OFST      0
#define MC_CMD_ALLOC_VIS_IN_MAX_OFST      4
#define MC_CMD_ALLOC_VIS_OUT_LEN          8
#define MC_CMD_ALLOC_VIS_OUT_VI_COUNT_OFST 0
#define MC_CMD_ALLOC_VIS_OUT_VI_BASE_OFST  4

#define MC_CMD_FREE_VIS                   0x8c

#define MC_CMD_GET_ASSERTS                0x06
#define MC_CMD_GET_ASSERTS_IN_LEN         4
#define MC_CMD_GET_ASSERTS_IN_CLEAR_OFST  0
#define MC_CMD_GET_ASSERTS_OUT_LEN        140
#define MC_CMD_GET_ASSERTS_OUT_GLOBAL_FLAGS_OFST 0
#define MC_CMD_GET_ASSERTS_FLAGS_NO_FAILS 0x1

#define MC_CMD_REBOOT                     0x3d
#define MC_CMD_REBOOT_IN_LEN              4
#define MC_CMD_REBOOT_IN_FLAGS_OFST       0
#define MC_CMD_REBOOT_FLAGS_AFTER_ASSERTION 0x1

#define MC_CMD_ENTITY_RESET               0x20
#define MC_CMD_ENTITY_RESET_IN_LEN        4
#define MC_CMD_ENTITY_RESET_IN_FLAG_OFST  0
#define MC_CMD_ENTITY_RESET_IN_FUNCTION_RESOURCE_RESET 0x1u  /* bit 0 */

#define MC_CMD_GET_PORT_ASSIGNMENT        0xb8
#define MC_CMD_GET_PORT_ASSIGNMENT_OUT_LEN 4
#define MC_CMD_GET_PORT_ASSIGNMENT_OUT_PORT_OFST 0

#define MC_CMD_GET_FUNCTION_INFO          0xec
#define MC_CMD_GET_FUNCTION_INFO_OUT_LEN  8
#define MC_CMD_GET_FUNCTION_INFO_OUT_PF_OFST 0
#define MC_CMD_GET_FUNCTION_INFO_OUT_VF_OFST 4

#define MC_CMD_GET_CAPABILITIES           0xbe
#define MC_CMD_GET_CAPABILITIES_OUT_LEN   20
#define MC_CMD_GET_CAPABILITIES_OUT_FLAGS1_OFST 0
#define MC_CMD_GET_CAPABILITIES_OUT_RX_DPCPU_FW_ID_OFST 4 /* 2 bytes */
#define MC_CMD_GET_CAPABILITIES_OUT_TX_DPCPU_FW_ID_OFST 6 /* 2 bytes */
#define MC_CMD_GET_CAPABILITIES_OUT_HW_CAPS_OFST 12 /* 4 bytes */
#define MC_CMD_GET_CAPABILITIES_OUT_LICENSE_CAPS_OFST 16 /* 4 bytes */

/* Polling parameters. Mirrors EF10_MCDI_CMD_TIMEOUT_US in sfxge. */
#define SFC7120_MCDI_POLL_MIN_US         10
#define SFC7120_MCDI_POLL_MAX_US         (100 * 1000)
#define SFC7120_MCDI_TIMEOUT_US          (10 * 1000 * 1000)

#define SFC7120_MCDI_LOCK(sc)            mtx_lock(&(sc)->mcdi_mtx)
#define SFC7120_MCDI_UNLOCK(sc)          mtx_unlock(&(sc)->mcdi_mtx)

/* ---------------------------------------------------------------------- */
/* Mailbox accessors — keep them in one place so we never accidentally    */
/* reach into the buffer with the wrong width or alignment. Each lane is  */
/* a 32-bit little-endian dword; payload writes/reads are dword-aligned. */
/* ---------------------------------------------------------------------- */

static __inline void
mcdi_buf_write_dword(sfc7120_softc_t *sc, size_t offset, uint32_t v)
{
    *((volatile uint32_t *)((uint8_t *)sc->mcdi_buf + offset)) = htole32(v);
}

static __inline uint32_t
mcdi_buf_read_dword(sfc7120_softc_t *sc, size_t offset)
{
    return le32toh(*((volatile uint32_t *)((uint8_t *)sc->mcdi_buf + offset)));
}

static void
mcdi_buf_write_payload(sfc7120_softc_t *sc, size_t offset,
                       const void *src, size_t len)
{
    size_t i;
    const uint8_t *s = src;
    for (i = 0; i + 4 <= len; i += 4) {
        uint32_t v;
        memcpy(&v, s + i, 4);
        mcdi_buf_write_dword(sc, offset + i, v);
    }
    /* Tail (commands always have dword-multiple lengths in v1, so this
     * is defensive only). */
    if (i < len) {
        uint32_t v = 0;
        memcpy(&v, s + i, len - i);
        mcdi_buf_write_dword(sc, offset + i, v);
    }
}

static void
mcdi_buf_read_payload(sfc7120_softc_t *sc, size_t offset,
                      void *dst, size_t len)
{
    size_t i;
    uint8_t *d = dst;
    for (i = 0; i + 4 <= len; i += 4) {
        uint32_t v = mcdi_buf_read_dword(sc, offset + i);
        memcpy(d + i, &v, 4);
    }
    if (i < len) {
        uint32_t v = mcdi_buf_read_dword(sc, offset + i);
        memcpy(d + i, &v, len - i);
    }
}

/* ---------------------------------------------------------------------- */
/* DMA helper for the mailbox (one buffer; not worth a generic helper).   */
/* ---------------------------------------------------------------------- */

static void
sfc7120_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
    bus_addr_t *out = arg;
    *out = (error != 0 || nseg < 1) ? 0 : segs[0].ds_addr;
}

static int
sfc7120_mcdi_alloc_buf(sfc7120_softc_t *sc)
{
    int error;
    bus_size_t len = MCDI_HEADER_LEN + MCDI_PAYLOAD_LEN_MAX_V1;

    /* 256-byte alignment is mandatory: the EF10 doorbell recovery
     * algorithm requires the low byte of the mailbox address to be 0
     * (sfxge bug24769; see ef10_mcdi_init). */
    error = bus_dma_tag_create(bus_get_dma_tag(sc->dev),
                               256, 0,
                               BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
                               NULL, NULL,
                               len, 1, len,
                               0, NULL, NULL, &sc->mcdi_dtag);
    if (error != 0) {
        device_printf(sc->dev, "MCDI: bus_dma_tag_create failed: %d\n", error);
        return error;
    }

    error = bus_dmamem_alloc(sc->mcdi_dtag, &sc->mcdi_buf,
                             BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO,
                             &sc->mcdi_dmamap);
    if (error != 0) {
        device_printf(sc->dev, "MCDI: bus_dmamem_alloc failed: %d\n", error);
        bus_dma_tag_destroy(sc->mcdi_dtag);
        sc->mcdi_dtag = NULL;
        return error;
    }

    sc->mcdi_buf_paddr = 0;
    error = bus_dmamap_load(sc->mcdi_dtag, sc->mcdi_dmamap, sc->mcdi_buf, len,
                            sfc7120_dma_cb, &sc->mcdi_buf_paddr,
                            BUS_DMA_NOWAIT);
    if (error != 0 || sc->mcdi_buf_paddr == 0) {
        device_printf(sc->dev, "MCDI: bus_dmamap_load failed: %d\n", error);
        bus_dmamem_free(sc->mcdi_dtag, sc->mcdi_buf, sc->mcdi_dmamap);
        bus_dma_tag_destroy(sc->mcdi_dtag);
        sc->mcdi_buf = NULL;
        sc->mcdi_dtag = NULL;
        return (error != 0) ? error : ENOMEM;
    }

    KASSERT((sc->mcdi_buf_paddr & 0xff) == 0,
        ("MCDI mailbox not 256-byte aligned: %#jx",
         (uintmax_t)sc->mcdi_buf_paddr));
    return 0;
}

static void
sfc7120_mcdi_free_buf(sfc7120_softc_t *sc)
{
    if (sc->mcdi_buf_paddr != 0) {
        bus_dmamap_unload(sc->mcdi_dtag, sc->mcdi_dmamap);
        sc->mcdi_buf_paddr = 0;
    }
    if (sc->mcdi_buf != NULL) {
        bus_dmamem_free(sc->mcdi_dtag, sc->mcdi_buf, sc->mcdi_dmamap);
        sc->mcdi_buf = NULL;
    }
    if (sc->mcdi_dtag != NULL) {
        bus_dma_tag_destroy(sc->mcdi_dtag);
        sc->mcdi_dtag = NULL;
    }
}

/* ---------------------------------------------------------------------- */
/* PCIe FLR.                                                               */
/*                                                                         */
/* Currently UNUSED — the call site in sfc7120_fbsd_attach is #if 0'd      */
/* because a host-driven FLR puts the SFN7322F-R2 PTP-firmware variant     */
/* into a state where the BIU clock domain stays gated indefinitely (every */
/* MMIO read returns 0). Stock sfxge does not FLR at attach either. Kept   */
/* as a placeholder for a future MCDI-driven reset path (MC_CMD_REBOOT).   */
/* ---------------------------------------------------------------------- */

int
sfc7120_pcie_flr(sfc7120_softc_t *sc)
{
    /* 1 second cap matches what FreeBSD's pci(9) helpers use for FLR. The
     * `force` flag is false: if the device doesn't advertise FLR capability
     * we don't try to fake it. */
    if (!pcie_flr(sc->dev, 1000 * 1000, false)) {
        device_printf(sc->dev,
            "PCIe FLR not supported or failed; continuing\n");
    }
    return 0;
}

/* ---------------------------------------------------------------------- */
/* MCDI init / fini.                                                       */
/* ---------------------------------------------------------------------- */

int
sfc7120_mcdi_init(sfc7120_softc_t *sc)
{
    int      error;
    uint32_t hw_rev;
    uint32_t mc_sft_status;
    uint16_t pci_cmd;

    KASSERT(!sc->mcdi_initialized, ("MCDI re-init"));

    mtx_init(&sc->mcdi_mtx, device_get_nameunit(sc->dev),
             "sfc7120 mcdi", MTX_DEF);

    error = sfc7120_mcdi_alloc_buf(sc);
    if (error != 0) {
        mtx_destroy(&sc->mcdi_mtx);
        return error;
    }

    /* PCI memory decode + bus mastering must be on for both MMIO reads
     * and the MC's DMA reads of the mailbox. attach() sets these up; this
     * is defensive in case something downstream cleared them. */
    pci_cmd = pci_read_config(sc->dev, PCIR_COMMAND, 2);
    if ((pci_cmd & (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN)) !=
        (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN)) {
        device_printf(sc->dev,
            "MCDI: PCIR_COMMAND=0x%04x missing MEMEN or BUSMASTER\n",
            pci_cmd);
        error = ENXIO;
        goto fail;
    }

    /* Liveness check. BIU_HW_REV_ID is hardwired to 0xeb14face on a healthy
     * EF10. A read of 0 indicates the BIU clock domain is not up — the
     * usual cause is a host-driven pcie_flr() which this card does not
     * tolerate (see CLAUDE.md "Bringup notes"). 0xffffffff means the BAR
     * isn't decoding at all. Either way no MCDI command will succeed. */
    hw_rev = SFC7120_READ_REG(sc, EF10_REG_BIU_HW_REV_ID);
    if (hw_rev != EF10_REG_BIU_HW_REV_ID_RESET) {
        device_printf(sc->dev,
            "MCDI: BIU_HW_REV_ID=0x%08x (expected 0x%08x); aborting\n",
            hw_rev, EF10_REG_BIU_HW_REV_ID_RESET);
        error = ENXIO;
        goto fail;
    }

    /* Snapshot the MC soft status so callers can detect later MC reboots
     * by re-reading and comparing against mcdi_prev_reboot_status. */
    mc_sft_status = SFC7120_READ_REG(sc, EF10_REG_BIU_MC_SFT_STATUS);
    device_printf(sc->dev, "MC soft status at attach: 0x%08x\n",
                  mc_sft_status);

    /* sfxge ef10_mcdi_init kicks HWRD with the literal value 1 before any
     * commands so the MC's view of the doorbell is in a known state (see
     * sfxge bug24769 recovery algorithm). */
    SFC7120_WRITE_REG(sc, EF10_REG_MC_DB_HWRD, 1);

    sc->mcdi_seq = 0;
    sc->mcdi_new_epoch = true;
    sc->mcdi_prev_reboot_status = mc_sft_status;
    sc->mcdi_initialized = true;
    return 0;

fail:
    sfc7120_mcdi_free_buf(sc);
    mtx_destroy(&sc->mcdi_mtx);
    return error;
}

void
sfc7120_mcdi_fini(sfc7120_softc_t *sc)
{
    if (!sc->mcdi_initialized)
        return;
    sfc7120_mcdi_free_buf(sc);
    mtx_destroy(&sc->mcdi_mtx);
    sc->mcdi_initialized = false;
}

/* ---------------------------------------------------------------------- */
/* MCDI request/poll/read primitives. Caller must hold mcdi_mtx.          */
/* ---------------------------------------------------------------------- */

/* Build the MCDI request in the mailbox and ring the MC doorbell. Picks
 * v1 or v2 framing based on opcode and payload size; returns the byte
 * offset where the request/response payload starts (4 for v1, 8 for v2)
 * so the caller knows where to read the response payload from. */
static size_t
sfc7120_mcdi_send_locked(sfc7120_softc_t *sc, uint32_t cmd,
                         const void *in, size_t in_len, uint8_t seq)
{
    uint32_t v1_hdr;
    uint32_t v2_hdr;
    uint64_t paddr;
    size_t   payload_off;
    bool     use_v2 = (cmd >= MC_CMD_V2_EXTN) ||
                      (in_len > MCDI_PAYLOAD_LEN_MAX_V1);

    mtx_assert(&sc->mcdi_mtx, MA_OWNED);
    KASSERT(in_len <= MCDI_PAYLOAD_LEN_MAX_V1,
        ("MCDI request payload too large: %zu", in_len));

    if (use_v2) {
        /* v1 header carries the V2_EXTN escape opcode and zero datalen. */
        v1_hdr  = (MC_CMD_V2_EXTN & MCDI_HDR_CODE_MASK)
                                                << MCDI_HDR_CODE_SHIFT;
        v1_hdr |= 0u                            << MCDI_HDR_DATALEN_SHIFT;
        v1_hdr |= ((uint32_t)seq & MCDI_HDR_SEQ_MASK) << MCDI_HDR_SEQ_SHIFT;
        if (!sc->mcdi_new_epoch)
            v1_hdr |= 1u << MCDI_HDR_NOT_EPOCH_SHIFT;

        /* v2 ext header carries the real cmd and request length. */
        v2_hdr  = (cmd & MCDI_V2_EXT_CMD_MASK)  << MCDI_V2_EXT_CMD_SHIFT;
        v2_hdr |= ((uint32_t)in_len & MCDI_V2_EXT_LEN_MASK)
                                                << MCDI_V2_EXT_LEN_SHIFT;

        mcdi_buf_write_dword(sc, 0, v1_hdr);
        mcdi_buf_write_dword(sc, MCDI_HEADER_LEN, v2_hdr);
        payload_off = MCDI_V2_HEADER_LEN;
    } else {
        v1_hdr  = (cmd & MCDI_HDR_CODE_MASK)    << MCDI_HDR_CODE_SHIFT;
        v1_hdr |= ((uint32_t)in_len & MCDI_HDR_DATALEN_MASK)
                                                << MCDI_HDR_DATALEN_SHIFT;
        v1_hdr |= ((uint32_t)seq & MCDI_HDR_SEQ_MASK) << MCDI_HDR_SEQ_SHIFT;
        if (!sc->mcdi_new_epoch)
            v1_hdr |= 1u << MCDI_HDR_NOT_EPOCH_SHIFT;

        mcdi_buf_write_dword(sc, 0, v1_hdr);
        payload_off = MCDI_HEADER_LEN;
    }

    if (in_len > 0)
        mcdi_buf_write_payload(sc, payload_off, in, in_len);

    /* Make sure the mailbox writes hit memory before the doorbell PIO. */
    bus_dmamap_sync(sc->mcdi_dtag, sc->mcdi_dmamap,
                    BUS_DMASYNC_PREWRITE);

    /*
     * Post the mailbox physical address to the MC via the doorbell pair.
     * Order matters: high half to MC_DB_LWRD first (no trigger), low half
     * to MC_DB_HWRD second (this write is the trigger). The "LWRD"/"HWRD"
     * names refer to the registers' offset in the doorbell pair, not the
     * half of the address they receive. See ef10_mcdi.c:182-188.
     */
    paddr = (uint64_t)sc->mcdi_buf_paddr;
    SFC7120_WRITE_REG(sc, EF10_REG_MC_DB_LWRD, (uint32_t)(paddr >> 32));
    SFC7120_WRITE_REG(sc, EF10_REG_MC_DB_HWRD,
                      (uint32_t)(paddr & 0xffffffffu));

    return payload_off;
}


static bool
sfc7120_mcdi_poll_response(sfc7120_softc_t *sc)
{
    uint32_t header;

    mtx_assert(&sc->mcdi_mtx, MA_OWNED);
    bus_dmamap_sync(sc->mcdi_dtag, sc->mcdi_dmamap,
                    BUS_DMASYNC_POSTREAD);
    header = mcdi_buf_read_dword(sc, 0);
    return ((header >> MCDI_HDR_RESPONSE_SHIFT) & 0x1) != 0;
}

/*
 * Translate an MC error code (see ref_efx_regs_mcdi.h "MC_CMD_ERR_*") into a
 * FreeBSD errno. Anything we don't recognize gets EIO; the raw code is
 * always logged at the call site so post-mortem is possible.
 */
static int
sfc7120_mcdi_xlate_err(uint32_t mc_err)
{
    switch (mc_err) {
    case 0:   return 0;
    case 1:   return EPERM;
    case 2:   return ENOENT;
    case 4:   return EINTR;
    case 5:   return EIO;
    case 11:  return EAGAIN;
    case 12:  return ENOMEM;
    case 13:  return EACCES;
    case 16:  return EBUSY;
    case 22:  return EINVAL;
    case 38:  return ENOSYS;
    case 62:  return ETIMEDOUT;
    case 95:  return EOPNOTSUPP;
    default:  return EIO;
    }
}

/* ---------------------------------------------------------------------- */
/* Synchronous exec wrapper.                                              */
/* ---------------------------------------------------------------------- */

int
sfc7120_mcdi_exec(sfc7120_softc_t *sc, uint32_t cmd,
                  const void *in, size_t in_len,
                  void *out, size_t out_len, size_t *out_used)
{
    uint32_t header;
    uint32_t mc_err;
    uint32_t resp_datalen;
    uint32_t delay_us;
    uint32_t total_us;
    uint8_t  seq;
    size_t   payload_off;
    int rc;

    if (!sc->mcdi_initialized)
        return ENXIO;
    if (in_len > MCDI_PAYLOAD_LEN_MAX_V1 ||
        out_len > MCDI_PAYLOAD_LEN_MAX_V1)
        return EINVAL;
    if ((in_len & 3) != 0)
        return EINVAL;

    SFC7120_MCDI_LOCK(sc);

    seq = sc->mcdi_seq & MCDI_HDR_SEQ_MASK;
    payload_off = sfc7120_mcdi_send_locked(sc, cmd, in, in_len, seq);

    delay_us = SFC7120_MCDI_POLL_MIN_US;
    total_us = 0;
    while (!sfc7120_mcdi_poll_response(sc)) {
        if (total_us >= SFC7120_MCDI_TIMEOUT_US) {
            uint32_t hdr_at_timeout, w1, w2, w3;
            uint32_t hw_rev_now, mc_sft_now;
            uint16_t pci_cmd_now;

            /* Sync once more in case POSTREAD wasn't done at the moment
             * the MC actually updated the mailbox. */
            bus_dmamap_sync(sc->mcdi_dtag, sc->mcdi_dmamap,
                            BUS_DMASYNC_POSTREAD);
            hdr_at_timeout = mcdi_buf_read_dword(sc, 0);
            w1 = mcdi_buf_read_dword(sc, 4);
            w2 = mcdi_buf_read_dword(sc, 8);
            w3 = mcdi_buf_read_dword(sc, 12);

            /* Re-read identity / status registers to confirm the BAR is
             * still alive and see whether the MC tripped over a reboot. */
            hw_rev_now  = SFC7120_READ_REG(sc, EF10_REG_BIU_HW_REV_ID);
            mc_sft_now  = SFC7120_READ_REG(sc, EF10_REG_BIU_MC_SFT_STATUS);
            pci_cmd_now = pci_read_config(sc->dev, PCIR_COMMAND, 2);

            device_printf(sc->dev,
                "MCDI cmd %#x timed out after %u us\n", cmd, total_us);
            device_printf(sc->dev,
                "  mailbox[0..15]: %08x %08x %08x %08x  (response_bit=%u)\n",
                hdr_at_timeout, w1, w2, w3,
                (hdr_at_timeout >> MCDI_HDR_RESPONSE_SHIFT) & 0x1u);
            device_printf(sc->dev,
                "  HW_REV_ID=0x%08x  MC_SFT_STATUS=0x%08x"
                "  PCIR_COMMAND=0x%04x (MEMEN=%d BM=%d)\n",
                hw_rev_now, mc_sft_now, pci_cmd_now,
                (pci_cmd_now & PCIM_CMD_MEMEN)       ? 1 : 0,
                (pci_cmd_now & PCIM_CMD_BUSMASTEREN) ? 1 : 0);
            device_printf(sc->dev,
                "  mcdi_buf_paddr=0x%lx seq=%u\n",
                (unsigned long)sc->mcdi_buf_paddr, seq);

            /* Advance epoch/seq even on timeout so the next command can
             * try with a fresh seq number. */
            sc->mcdi_new_epoch = false;
            sc->mcdi_seq = (sc->mcdi_seq + 1) & MCDI_HDR_SEQ_MASK;

            SFC7120_MCDI_UNLOCK(sc);
            return ETIMEDOUT;
        }
        DELAY(delay_us);
        total_us += delay_us;
        delay_us *= 2;
        if (delay_us > SFC7120_MCDI_POLL_MAX_US)
            delay_us = SFC7120_MCDI_POLL_MAX_US;
    }

    /* Response is now in the mailbox. v2 responses re-use the V2_EXTN
     * escape: v1 header carries cmd=0x7f and datalen=0, with the real
     * response length in the v2 ext header at offset 4. */
    header = mcdi_buf_read_dword(sc, 0);
    if (payload_off == MCDI_V2_HEADER_LEN) {
        uint32_t v2_hdr = mcdi_buf_read_dword(sc, MCDI_HEADER_LEN);
        resp_datalen = (v2_hdr >> MCDI_V2_EXT_LEN_SHIFT) &
                       MCDI_V2_EXT_LEN_MASK;
    } else {
        resp_datalen = (header >> MCDI_HDR_DATALEN_SHIFT) &
                       MCDI_HDR_DATALEN_MASK;
    }

    uint8_t resp_seq = (header >> MCDI_HDR_SEQ_SHIFT) & MCDI_HDR_SEQ_MASK;
    if (resp_seq != seq) {
        device_printf(sc->dev,
            "MCDI cmd %#x: seq mismatch (got %u, expected %u)\n",
            cmd, resp_seq, seq);
        rc = EPROTO;
        goto out;
    }

    if ((header >> MCDI_HDR_ERROR_SHIFT) & 0x1) {
        /* MC error: payload[0] is the MC_CMD_ERR_* code. */
        mc_err = (resp_datalen >= 4)
            ? mcdi_buf_read_dword(sc, payload_off) : 0;
        rc = sfc7120_mcdi_xlate_err(mc_err);
        device_printf(sc->dev,
            "MCDI cmd %#x failed: MC_CMD_ERR=%u (errno=%d)\n",
            cmd, mc_err, rc);
        goto out;
    }

    if (out_len > 0 && out != NULL) {
        size_t copy = (resp_datalen < out_len) ? resp_datalen : out_len;
        if (copy > 0)
            mcdi_buf_read_payload(sc, payload_off, out, copy);
        /* Zero anything the MC didn't fill so callers don't see stale
         * mailbox bytes. */
        if (copy < out_len)
            memset((uint8_t *)out + copy, 0, out_len - copy);
        if (out_used != NULL)
            *out_used = copy;
    } else if (out_used != NULL) {
        *out_used = 0;
    }
    rc = 0;

out:
    /* Advance epoch + seq for next command. */
    sc->mcdi_new_epoch = false;
    sc->mcdi_seq = (sc->mcdi_seq + 1) & MCDI_HDR_SEQ_MASK;
    SFC7120_MCDI_UNLOCK(sc);
    return rc;
}

/* ---------------------------------------------------------------------- */
/* High-level MC_CMD wrappers.                                            */
/* ---------------------------------------------------------------------- */

/* Snapshot the BIU liveness register and MC soft status. Useful for
 * pinpointing which MCDI command (if any) caused the MC to self-reboot:
 * MC_SFT_STATUS increments on every MC reboot (low byte ~ reboot count),
 * and HW_REV_ID drops to 0 while the BIU clock domain is gated. */
void
sfc7120_mcdi_log_mc_state(sfc7120_softc_t *sc, const char *tag)
{
    uint32_t hw_rev = SFC7120_READ_REG(sc, EF10_REG_BIU_HW_REV_ID);
    uint32_t sft    = SFC7120_READ_REG(sc, EF10_REG_BIU_MC_SFT_STATUS);
    device_printf(sc->dev,
        "MC state @%s: HW_REV_ID=0x%08x MC_SFT_STATUS=0x%08x\n",
        tag, hw_rev, sft);
}

int
sfc7120_mcdi_clear_assertions(sfc7120_softc_t *sc)
{
    uint8_t  in[MC_CMD_GET_ASSERTS_IN_LEN]  = {0};
    uint8_t  resp[MC_CMD_GET_ASSERTS_OUT_LEN] = {0};
    size_t   used = 0;
    uint32_t clear = 1;
    uint32_t flags;
    int      rc = 0;
    int      retry;

    memcpy(&in[MC_CMD_GET_ASSERTS_IN_CLEAR_OFST], &clear, 4);

    /* sfxge retries this twice: a boot-time assertion can poison the very
     * first MCDI exchange, and we may race with a partner-port driver also
     * calling GET_ASSERTS (efx_mcdi.c:1192-1214). */
    for (retry = 0; retry < 3; retry++) {
        rc = sfc7120_mcdi_exec(sc, MC_CMD_GET_ASSERTS,
                               in, sizeof(in), resp, sizeof(resp), &used);
        if (rc != EINTR && rc != EIO)
            break;
    }

    if (rc == EACCES) {
        /* Unprivileged function — partner driver will clear assertions. */
        return 0;
    }
    if (rc != 0)
        return rc;

    if (used < 4) {
        device_printf(sc->dev,
            "MCDI GET_ASSERTS: short response (%zu bytes)\n", used);
        return 0;
    }
    memcpy(&flags, &resp[MC_CMD_GET_ASSERTS_OUT_GLOBAL_FLAGS_OFST], 4);
    device_printf(sc->dev, "MC GET_ASSERTS flags=%#x%s\n", flags,
        (flags == MC_CMD_GET_ASSERTS_FLAGS_NO_FAILS)
            ? " (no failures)" : " (assertion cleared)");
    return 0;
}

int
sfc7120_mcdi_entity_reset(sfc7120_softc_t *sc)
{
    uint8_t  in[MC_CMD_ENTITY_RESET_IN_LEN] = {0};
    uint32_t flag = MC_CMD_ENTITY_RESET_IN_FUNCTION_RESOURCE_RESET;
    int      rc;

    /* Per-function resource reset. Releases any VIs / queues / filters
     * still tied to this PCI function in the MC's bookkeeping, including
     * leftovers from a previous load of this driver that didn't go through
     * a clean detach. Does NOT reboot the MC. Mirrors the
     * MC_CMD_ENTITY_RESET command (opcode 0x20) issued by sfxge in
     * ef10_nic_reset (ef10_nic.c:2146-2155). */
    memcpy(&in[MC_CMD_ENTITY_RESET_IN_FLAG_OFST], &flag, 4);

    rc = sfc7120_mcdi_exec(sc, MC_CMD_ENTITY_RESET, in, sizeof(in),
                           NULL, 0, NULL);
    if (rc == EACCES) {
        device_printf(sc->dev,
            "MCDI ENTITY_RESET: not permitted (unprivileged function?)\n");
        return 0;
    }
    if (rc != 0) {
        device_printf(sc->dev, "MCDI ENTITY_RESET failed: %d\n", rc);
        return rc;
    }
    device_printf(sc->dev, "MC ENTITY_RESET (function resources): ok\n");
    return 0;
}

/* Pure diagnostic. Runs three side-effect-free MCDI queries that sfxge
 * executes in ef10_nic_board_cfg between DRV_ATTACH and ALLOC_VIS, and
 * dumps their results. Helps narrow down why ALLOC_VIS returns ENOENT:
 *
 *  - GET_PORT_ASSIGNMENT: which port (0/1) the firmware has bound this
 *    PF to. If this fails the function isn't bound to a port — that
 *    alone would explain ALLOC_VIS=ENOENT.
 *  - GET_FUNCTION_INFO: firmware's view of (PF, VF) for this function.
 *    Confirms our identity and that we're a PF (vf == 0xffff).
 *  - GET_CAPABILITIES: datapath firmware variant + capability flags.
 *    The TX/RX datapath FW IDs tell us whether we have a "low-latency",
 *    "full-featured", "DPDK", etc. variant — some variants gate VI
 *    allocation on extra setup.
 *
 * Errors are non-fatal; we log and keep going so the rest of the trace
 * is still produced. */
int
sfc7120_mcdi_dump_func_info(sfc7120_softc_t *sc)
{
    uint8_t  resp[MC_CMD_GET_CAPABILITIES_OUT_LEN] = {0};
    size_t   used = 0;
    uint32_t port = 0xffffffff;
    uint32_t pf   = 0xffffffff;
    uint32_t vf   = 0xffffffff;
    uint32_t flags1;
    int      rc;

    rc = sfc7120_mcdi_exec(sc, MC_CMD_GET_PORT_ASSIGNMENT,
                           NULL, 0, resp, MC_CMD_GET_PORT_ASSIGNMENT_OUT_LEN,
                           &used);
    if (rc != 0) {
        device_printf(sc->dev,
            "MCDI GET_PORT_ASSIGNMENT failed: %d\n", rc);
    } else if (used >= 4) {
        memcpy(&port, &resp[MC_CMD_GET_PORT_ASSIGNMENT_OUT_PORT_OFST], 4);
        device_printf(sc->dev,
            "MC GET_PORT_ASSIGNMENT: port=%u\n", port);
    }

    memset(resp, 0, sizeof(resp));
    used = 0;
    rc = sfc7120_mcdi_exec(sc, MC_CMD_GET_FUNCTION_INFO,
                           NULL, 0, resp, MC_CMD_GET_FUNCTION_INFO_OUT_LEN,
                           &used);
    if (rc != 0) {
        device_printf(sc->dev,
            "MCDI GET_FUNCTION_INFO failed: %d\n", rc);
    } else if (used >= 8) {
        memcpy(&pf, &resp[MC_CMD_GET_FUNCTION_INFO_OUT_PF_OFST], 4);
        memcpy(&vf, &resp[MC_CMD_GET_FUNCTION_INFO_OUT_VF_OFST], 4);
        /* Firmware returns vf=0xffff to mark "this is a PF" (sfxge:
         * EFX_PCI_FUNCTION_IS_PF — efx_impl.h). Otherwise vf is the
         * VF number and pf is the parent PF. */
        device_printf(sc->dev,
            "MC GET_FUNCTION_INFO: pf=%u vf=0x%x %s\n",
            pf, vf, (vf == 0xffff) ? "(PF)" : "(VF)");
    }

    memset(resp, 0, sizeof(resp));
    used = 0;
    rc = sfc7120_mcdi_exec(sc, MC_CMD_GET_CAPABILITIES,
                           NULL, 0, resp, MC_CMD_GET_CAPABILITIES_OUT_LEN,
                           &used);
    if (rc != 0) {
        device_printf(sc->dev,
            "MCDI GET_CAPABILITIES failed: %d\n", rc);
    } else if (used >= MC_CMD_GET_CAPABILITIES_OUT_LEN) {
        uint16_t rxdp_id, txdp_id;
        uint32_t hw_caps, lic_caps;
        memcpy(&flags1, &resp[MC_CMD_GET_CAPABILITIES_OUT_FLAGS1_OFST], 4);
        memcpy(&rxdp_id,
               &resp[MC_CMD_GET_CAPABILITIES_OUT_RX_DPCPU_FW_ID_OFST], 2);
        memcpy(&txdp_id,
               &resp[MC_CMD_GET_CAPABILITIES_OUT_TX_DPCPU_FW_ID_OFST], 2);
        memcpy(&hw_caps,
               &resp[MC_CMD_GET_CAPABILITIES_OUT_HW_CAPS_OFST], 4);
        memcpy(&lic_caps,
               &resp[MC_CMD_GET_CAPABILITIES_OUT_LICENSE_CAPS_OFST], 4);
        device_printf(sc->dev,
            "MC GET_CAPABILITIES: flags1=0x%08x rxdp_fw=0x%04x "
            "txdp_fw=0x%04x hw_caps=0x%08x lic_caps=0x%08x\n",
            flags1, le16toh(rxdp_id), le16toh(txdp_id),
            hw_caps, lic_caps);
    } else {
        device_printf(sc->dev,
            "MC GET_CAPABILITIES: short response (%zu bytes)\n", used);
    }

    return 0;
}

int
sfc7120_mcdi_reboot_after_assertion(sfc7120_softc_t *sc)
{
    uint8_t  in[MC_CMD_REBOOT_IN_LEN] = {0};
    uint32_t flags  = MC_CMD_REBOOT_FLAGS_AFTER_ASSERTION;
    uint32_t hw_rev = 0;
    int      rc;
    int      waited_us;

    memcpy(&in[MC_CMD_REBOOT_IN_FLAGS_OFST], &flags, 4);

    /* Three outcomes:
     *   1. No pending assertion → MC acks; exec returns 0 quickly.
     *   2. Assertion + privileged function → MC reboots, response is lost,
     *      exec times out (ETIMEDOUT). We then wait for BIU_HW_REV_ID to
     *      come back to 0xeb14face and reset the MCDI epoch.
     *   3. Unprivileged function → EACCES, treat as success.
     * Mirrors sfxge efx_mcdi_do_reboot (efx_mcdi.c:1110-1157). */
    rc = sfc7120_mcdi_exec(sc, MC_CMD_REBOOT, in, sizeof(in), NULL, 0, NULL);

    if (rc == 0 || rc == EACCES)
        return 0;
    if (rc != ETIMEDOUT && rc != EIO) {
        device_printf(sc->dev,
            "MCDI REBOOT(AFTER_ASSERTION) unexpected rc=%d\n", rc);
        return rc;
    }

    /* MC is rebooting (or has rebooted). Wait up to 5s for the BIU clock
     * domain to come back. Same pattern the post-FLR wait would use; see
     * CLAUDE.md "Bringup notes". */
    for (waited_us = 0; waited_us < 5 * 1000 * 1000; waited_us += 1000) {
        hw_rev = SFC7120_READ_REG(sc, EF10_REG_BIU_HW_REV_ID);
        if (hw_rev == EF10_REG_BIU_HW_REV_ID_RESET)
            break;
        DELAY(1000);
    }
    if (hw_rev != EF10_REG_BIU_HW_REV_ID_RESET) {
        device_printf(sc->dev,
            "MC reboot wait: BIU_HW_REV_ID=0x%08x after %d us\n",
            hw_rev, waited_us);
        return EIO;
    }
    device_printf(sc->dev,
        "MC reboot recovered (BIU alive after %d us)\n", waited_us);

    /* Reset MCDI epoch state. Post-reboot the MC counts seq from 0 again
     * with NOT_EPOCH=0 expected on the first command. */
    SFC7120_MCDI_LOCK(sc);
    sc->mcdi_new_epoch = true;
    sc->mcdi_seq       = 0;
    SFC7120_MCDI_UNLOCK(sc);

    /* Re-prime the doorbell — same kick that sfc7120_mcdi_init does after
     * the BAR comes alive (sfxge bug24769 recovery). */
    SFC7120_WRITE_REG(sc, EF10_REG_MC_DB_HWRD, 1);

    return 0;
}

int
sfc7120_mcdi_get_version(sfc7120_softc_t *sc)
{
    //buffer to hold NIC response 
    uint8_t  resp[MC_CMD_GET_VERSION_OUT_LEN] = {0};
    //nic writes this to say how many bytes were written to buffer
    size_t   used = 0;
    uint32_t v_lo, v_hi;
    int      rc;

    //sfc7120_mcdi_exec is generic command sending function
    //we send NULL 0 payload because get version takes no arguments
    rc = sfc7120_mcdi_exec(sc, MC_CMD_GET_VERSION,
                           NULL, 0, resp, sizeof(resp), &used);
    if (rc != 0)
        return rc;
    //make sure we actually got enough bytes to read a version number
    if (used < MC_CMD_GET_VERSION_OUT_VERSION_OFST + 8) {
        device_printf(sc->dev,
            "MCDI GET_VERSION: short response (%zu bytes)\n", used);
        return EPROTO;
    }

    uint32_t raw_lo, raw_hi;
    //grab the lower 32 and upper 32 bits from the buffer and store them in our stack allocated values
    memcpy(&raw_lo, &resp[MC_CMD_GET_VERSION_OUT_VERSION_OFST + 0], 4);
    memcpy(&raw_hi, &resp[MC_CMD_GET_VERSION_OUT_VERSION_OFST + 4], 4);

    //finn: added this - MCDI is little endian, just made it so that values are still correct on big endian host
    v_lo = le32toh(raw_lo);
    v_hi = le32toh(raw_hi);

    sc->fw_version[0] = v_lo;
    sc->fw_version[1] = v_hi;
    device_printf(sc->dev, "MC firmware version %u.%u.%u.%u\n",
                  (v_hi >> 16) & 0xffff,
                  v_hi & 0xffff,
                  (v_lo >> 16) & 0xffff,
                  v_lo & 0xffff);
    return 0;
}

int
sfc7120_mcdi_drv_attach(sfc7120_softc_t *sc)
{
    uint8_t  in[MC_CMD_DRV_ATTACH_IN_LEN] = {0};
    uint8_t  resp[MC_CMD_DRV_ATTACH_EXT_OUT_LEN] = {0};
    size_t   used = 0;
    uint32_t new_state, update, fw_id;
    int      rc;

    /* NEW_STATE: bit 0 = ATTACH. We're attaching, not pre-boot, no
     * VI-spreading or sub-variant awareness. */
    new_state = 0x1;
    update    = 0x1;
    fw_id     = MC_CMD_FW_DONT_CARE;
    memcpy(&in[MC_CMD_DRV_ATTACH_IN_NEW_STATE_OFST],   &new_state, 4);
    memcpy(&in[MC_CMD_DRV_ATTACH_IN_UPDATE_OFST],      &update,    4);
    memcpy(&in[MC_CMD_DRV_ATTACH_IN_FIRMWARE_ID_OFST], &fw_id,     4);

    rc = sfc7120_mcdi_exec(sc, MC_CMD_DRV_ATTACH,
                           in, sizeof(in), resp, sizeof(resp), &used);
    if (rc != 0)
        return rc;

    sc->mcdi_func_flags = 0;
    if (used >= MC_CMD_DRV_ATTACH_EXT_OUT_LEN) {
        memcpy(&sc->mcdi_func_flags,
               &resp[MC_CMD_DRV_ATTACH_EXT_OUT_FUNC_FLAGS_OFST], 4);
    } else if (used < MC_CMD_DRV_ATTACH_OUT_LEN) {
        device_printf(sc->dev,
            "MCDI DRV_ATTACH: short response (%zu bytes)\n", used);
        return EPROTO;
    }

    sc->drv_attached = true;
    device_printf(sc->dev, "MC: driver attached (func_flags=%#x)\n",
                  sc->mcdi_func_flags);
    return 0;
}

int
sfc7120_mcdi_drv_detach(sfc7120_softc_t *sc)
{
    uint8_t  in[MC_CMD_DRV_ATTACH_IN_LEN] = {0};
    uint8_t  resp[MC_CMD_DRV_ATTACH_EXT_OUT_LEN] = {0};
    uint32_t new_state, update, fw_id;
    int      rc;

    if (!sc->drv_attached)
        return 0;

    /* NEW_STATE bit 0 cleared = detach. */
    new_state = 0x0;
    update    = 0x1;
    fw_id     = MC_CMD_FW_DONT_CARE;
    memcpy(&in[MC_CMD_DRV_ATTACH_IN_NEW_STATE_OFST],   &new_state, 4);
    memcpy(&in[MC_CMD_DRV_ATTACH_IN_UPDATE_OFST],      &update,    4);
    memcpy(&in[MC_CMD_DRV_ATTACH_IN_FIRMWARE_ID_OFST], &fw_id,     4);

    rc = sfc7120_mcdi_exec(sc, MC_CMD_DRV_ATTACH,
                           in, sizeof(in), resp, sizeof(resp), NULL);
    if (rc == 0)
        sc->drv_attached = false;
    return rc;
}

int
sfc7120_mcdi_get_mac(sfc7120_softc_t *sc)
{
    uint8_t resp[MC_CMD_GET_MAC_ADDRESSES_OUT_LEN] = {0};
    size_t  used = 0;
    int     rc;

    rc = sfc7120_mcdi_exec(sc, MC_CMD_GET_MAC_ADDRESSES,
                           NULL, 0, resp, sizeof(resp), &used);
    if (rc != 0)
        return rc;
    if (used < MC_CMD_GET_MAC_ADDRESSES_OUT_BASE_OFST + 6) {
        device_printf(sc->dev,
            "MCDI GET_MAC_ADDRESSES: short response (%zu bytes)\n", used);
        return EPROTO;
    }

    memcpy(sc->mac_addr,
           &resp[MC_CMD_GET_MAC_ADDRESSES_OUT_BASE_OFST], 6);
    device_printf(sc->dev,
        "MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
        sc->mac_addr[0], sc->mac_addr[1], sc->mac_addr[2],
        sc->mac_addr[3], sc->mac_addr[4], sc->mac_addr[5]);
    return 0;
}

int
sfc7120_mcdi_alloc_vis(sfc7120_softc_t *sc,
                       uint32_t min_count, uint32_t max_count)
{
    uint8_t in[MC_CMD_ALLOC_VIS_IN_LEN] = {0};
    uint8_t resp[MC_CMD_ALLOC_VIS_OUT_LEN] = {0};
    size_t  used = 0;
    int     rc;

    /* The pre-ALLOC FREE_VIS that used to live here was redundant once
     * sfc7120_hw_init runs MC_CMD_ENTITY_RESET (FUNCTION_RESOURCE_RESET)
     * after DRV_ATTACH — that already releases any VIs the function held
     * from a prior load. Dropping the FREE_VIS lets us see whether the MC
     * reboots on FREE_VIS-of-nothing or only on ALLOC_VIS itself. */

    memcpy(&in[MC_CMD_ALLOC_VIS_IN_MIN_OFST], &min_count, 4);
    memcpy(&in[MC_CMD_ALLOC_VIS_IN_MAX_OFST], &max_count, 4);

    rc = sfc7120_mcdi_exec(sc, MC_CMD_ALLOC_VIS,
                           in, sizeof(in), resp, sizeof(resp), &used);
    if (rc != 0)
        return rc;
    if (used < MC_CMD_ALLOC_VIS_OUT_LEN) {
        device_printf(sc->dev,
            "MCDI ALLOC_VIS: short response (%zu bytes)\n", used);
        return EPROTO;
    }

    memcpy(&sc->vi_count, &resp[MC_CMD_ALLOC_VIS_OUT_VI_COUNT_OFST], 4);
    memcpy(&sc->vi_base,  &resp[MC_CMD_ALLOC_VIS_OUT_VI_BASE_OFST],  4);
    sc->vis_allocated = true;
    device_printf(sc->dev, "MC: allocated %u VIs starting at base %u\n",
                  sc->vi_count, sc->vi_base);
    return 0;
}

int
sfc7120_mcdi_free_vis(sfc7120_softc_t *sc)
{
    int rc;

    if (!sc->vis_allocated)
        return 0;
    rc = sfc7120_mcdi_exec(sc, MC_CMD_FREE_VIS, NULL, 0, NULL, 0, NULL);
    if (rc == 0) {
        sc->vis_allocated = false;
        sc->vi_count = 0;
        sc->vi_base  = 0;
    }
    return rc;
}
