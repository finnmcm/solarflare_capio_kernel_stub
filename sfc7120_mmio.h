#ifndef SFC7120_MMIO_HEADER
#define SFC7120_MMIO_HEADER

#include "capio.h"
#include "sfc7120.h"

/*
 * Solarflare EF10 (Huntington / SFN7xxx) BAR0 register offsets.
 *
 * EF10 is fundamentally different from e1000 — it talks to firmware (the
 * Management Controller, "MC") through MCDI: a doorbell + DMA-resident
 * mailbox protocol, similar in spirit to mlx5pol's command queue. The
 * register set userspace cares about is small:
 *
 *   - MC doorbell   — kick MCDI requests
 *   - EVQ doorbells — write read pointer to ack events
 *   - RX/TX doorbells — push descriptor producer pointers
 *
 * The values below are PLACEHOLDERS keyed off public Solarflare MCDI/EF10
 * documentation patterns and the FreeBSD `sfxge` driver. Cross-check
 * against the EF10 register documentation for your specific SFN7xxx board
 * before depending on these.
 */

/* MCDI request doorbell. EF10 splits this into a low-word (0x0200) and
 * high-word (0x0204) register; for MCDI you only need to write the low word.
 * Verified against ER_DZ_MC_DB_LWRD_REG_OFST in efx_regs_ef10.h. */
#define SFC7120_REG_MCDB            0x0200    /* MC doorbell low word */
/* #define SFC7120_REG_MC_EVENT — does not exist as an MMIO register on EF10.
 * MC events are delivered through the EVQ ring (DMA buffer), not via a
 * dedicated BAR address. No slice entry should reference this. */

/* Per-VI doorbells for function-local VI 0.
 * Formula: reg_offset + (vi_local_index << vi_window_shift)
 * Huntington uses EFX_VI_WINDOW_SHIFT_8K (shift=13, stride=8192). For VI 0
 * the shift term is 0, so these match the base offsets exactly.
 * Confirmed from efx_regs_ef10.h: ER_DZ_EVQ_RPTR_REG_OFST,
 * ER_DZ_RX_DESC_UPD_REG_OFST, ER_DZ_TX_DESC_UPD_REG_OFST. */
#define SFC7120_REG_EVQ_RPTR_DBL    0x0400  /* ERF_DZ_EVQ_RPTR  LBN=0 WIDTH=15 */
#define SFC7120_REG_RX_DESC_DBL     0x0830  /* ERF_DZ_RX_DESC_WPTR LBN=0 WIDTH=12; align to 8 */
#define SFC7120_REG_TX_DESC_DBL     0x0a10  /* 128-bit TX_DESC_UPD base (full push) */
/* TX write-pointer-only doorbell: EFX_BAR_VI_WRITED2 adds 8 bytes (2 dwords)
 * to the base, landing on dword[2] which holds ERF_DZ_TX_DESC_WPTR (LBN=64,
 * WIDTH=12 of the 128-bit register). See ef10_tx.c:ef10_tx_qpush. */
#define SFC7120_REG_TX_WPTR_DBL     (SFC7120_REG_TX_DESC_DBL + 8)  /* 0x0a18 */

/* Boot/identity registers.
 * BIU_HW_REV_ID verified against ER_DZ_BIU_HW_REV_ID_REG_OFST in efx_regs_ef10.h. */
#define SFC7120_REG_BIU_HW_REV_ID   0x0000
/* #define SFC7120_REG_MC_STATUS — does not exist as an MMIO register on EF10.
 * "MC_STATUS" in sfxge is a magic value (0xb007b007 / 0xdeaddead) written by
 * firmware into the MCDI *response buffer* in DMA memory, not a BAR offset. */

/* Userspace BAR window size — full BAR0. Caller must populate via
 * rman_get_size(). The slice manifest below is what bounds individual
 * userspace capabilities. */

/* ----------------------------------------------------------------------
 * R/W helpers — same shape as e1000's macros so existing code patterns
 * port cleanly. The `debug_reg_ops` flag in the softc, when true, logs
 * every access via device_printf for bringup debugging.
 * ---------------------------------------------------------------------- */

static __inline uint32_t
SFC7120_READ_REG(sfc7120_softc_t *sc, bus_size_t off)
{
    uint32_t v = bus_space_read_4(sc->mem_bus_tag, sc->mem_bsh, off);
    if (sc->debug_reg_ops)
        device_printf(sc->dev, "R [%04lx] = %08x\n",
                      (unsigned long)off, v);
    return v;
}

static __inline void
SFC7120_WRITE_REG(sfc7120_softc_t *sc, bus_size_t off, uint32_t v)
{
    if (sc->debug_reg_ops)
        device_printf(sc->dev, "W [%04lx] = %08x\n",
                      (unsigned long)off, v);
    bus_space_write_4(sc->mem_bus_tag, sc->mem_bsh, off, v);
}

/* Slice manifest — populated in sfc7120_tables.c. */
extern slice_def_t sfc7120_reg_slices[];
extern const size_t SFC7120_MMIO_SLICE_COUNT;

/* Diagnostic dump (optional; mirrors e1000e_dump_regs). */
void sfc7120_dump_regs(sfc7120_softc_t *sc);

#endif /* SFC7120_MMIO_HEADER */
