/*
 * sfc7120_mcdi.h — kernel-side MCDI (Management Controller Driver Interface)
 * plumbing for the Solarflare EF10 (Huntington / SFN7xxx) NIC.
 *
 * MCDI is a synchronous request/response protocol. Each exchange is a single
 * DMA-resident mailbox + a doorbell write that hands the mailbox's physical
 * address to the MC firmware. Polling the response is done by reading the
 * RESPONSE bit of the header word back from the same mailbox.
 *
 * Wire format (little-endian, MCDI v1):
 *
 *   word0 (header):
 *     bits[6:0]   command code
 *     bit  [7]    resync (always 0 for ordinary requests)
 *     bits[15:8]  payload length in bytes
 *     bits[19:16] sequence number
 *     bit  [21]   NOT_EPOCH (0 = first command of a new epoch)
 *     bit  [22]   error  (response only)
 *     bit  [23]   response (set in the response)
 *     bits[31:24] xflags
 *
 *   word1..N: command-specific payload, written immediately after the header.
 *
 * Reference: ~/cheri/cheribsd/sys/dev/sfxge/common/ef10_mcdi.c and
 * ref_efx_regs_mcdi.h (local copy of the EF10 MCDI command catalogue).
 */
#ifndef SFC7120_MCDI_H
#define SFC7120_MCDI_H

#include "sfc7120.h"

/* PCIe Function Level Reset before any MCDI traffic. Best-effort: returns 0
 * if FLR succeeded or was unsupported, non-zero only on hard failure. */
int  sfc7120_pcie_flr(sfc7120_softc_t *sc);

/* Allocate the MCDI mailbox (DMA-coherent, 256-byte aligned), initialize the
 * MCDI mutex and seq/epoch state, and ping the MC doorbell to announce the
 * fresh transport. Mirrors ef10_mcdi_init() in sfxge. */
int  sfc7120_mcdi_init(sfc7120_softc_t *sc);

/* Tear down the mailbox + lock. Idempotent. */
void sfc7120_mcdi_fini(sfc7120_softc_t *sc);

/* Synchronous MCDI exchange. Serialized by the MCDI mutex; polls the response
 * with exponential backoff up to a 10-second cap (matches EF10_MCDI_CMD_TIMEOUT_US).
 *
 *   cmd      — command code (MC_CMD_*)
 *   in       — request payload, may be NULL if in_len == 0
 *   in_len   — request payload length in bytes (must be a multiple of 4)
 *   out      — response payload buffer, may be NULL if out_len == 0
 *   out_len  — capacity of out
 *   out_used — actual response payload length (optional, may be NULL)
 *
 * Returns 0 on success, ETIMEDOUT on poll timeout, or an MC_CMD_ERR_* derived
 * errno when the firmware reports an error in the response header. */
int  sfc7120_mcdi_exec(sfc7120_softc_t *sc, uint32_t cmd,
                       const void *in, size_t in_len,
                       void *out, size_t out_len, size_t *out_used);

/* High-level wrappers used by sfc7120_hw_init / sfc7120_hw_teardown. */
int  sfc7120_mcdi_get_version(sfc7120_softc_t *sc);
int  sfc7120_mcdi_drv_attach(sfc7120_softc_t *sc);
int  sfc7120_mcdi_drv_detach(sfc7120_softc_t *sc);
int  sfc7120_mcdi_get_mac(sfc7120_softc_t *sc);
int  sfc7120_mcdi_alloc_vis(sfc7120_softc_t *sc,
                            uint32_t min_count, uint32_t max_count);
int  sfc7120_mcdi_free_vis(sfc7120_softc_t *sc);

#endif /* SFC7120_MCDI_H */
