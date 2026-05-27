#include "sfc7120_mmio.h"

/*
 * CAPIO MMIO slice manifest for the Solarflare 7120 (Huntington / EF10).
 *
 * Each entry defines a sub-capability that userspace can derive into the
 * BAR0 capability. modmap enforces these bounds in hardware: registers
 * not listed cannot be accessed by userspace, even if the offsets are
 * known. Read-only registers get CHERI_PERM_LOAD only.
 *
 * This is the SHORT initial set covering the registers a CAPIO
 * userspace driver minimally needs:
 *
 *   - MC doorbell (RW)               — push MCDI requests (low word only)
 *   - EVQ read-pointer doorbell (RW) — ack events
 *   - RX/TX descriptor doorbells (RW) — push producer pointer
 *   - HW_REV_ID (RO)                 — sanity check at attach
 *
 * MC_EVENT and MC_STATUS are omitted: neither is a real BAR register on EF10.
 * MC events arrive via the EVQ DMA ring; MC status is a magic value in the
 * MCDI response buffer.
 *
 * Extend this table when adding multi-queue support. Mirror the per-queue
 * pattern from mlx5pol: one slice per channel, named with the channel
 * index suffix.
 */
slice_def_t sfc7120_reg_slices[] = {
    { SFC7120_REG_MCDB,         "MC_DOORBELL",     false, 4  },
    { SFC7120_REG_EVQ_RPTR_DBL, "EVQ_RPTR_DBL",    false, 4  },
    { SFC7120_REG_RX_DESC_DBL,  "RX_DESC_DBL",     false, 4  },
    /* TX_DESC_UPD is a 128-bit (16-byte) register.  The userspace driver needs
     * two dwords: base+0 (descriptor LWORD) and base+8 (wptr-only push, dword[2]
     * = ERF_DZ_TX_DESC_WPTR).  12 bytes covers both without exposing dword[3]. */
    { SFC7120_REG_TX_DESC_DBL,  "TX_DESC_DBL",     false, 12 },
    { SFC7120_REG_BIU_HW_REV_ID,"HW_REV_ID",       true,  4  },
};

const size_t SFC7120_MMIO_SLICE_COUNT =
    sizeof(sfc7120_reg_slices) / sizeof(sfc7120_reg_slices[0]);
