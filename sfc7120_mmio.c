#include "sfc7120_mmio.h"

/* Documented power-on value of BIU_HW_REV_ID at BAR2 offset 0.
 * From efx_regs_ef10.h: ER_DZ_BIU_HW_REV_ID_REG_RESET. A live read of the
 * register returns this; 0x00000000 means the BIU clock domain is gated /
 * the read never reached live register state. */
#define SFC7120_HW_REV_ID_RESET     0xeb14faceu

void
sfc7120_dump_regs(sfc7120_softc_t *sc)
{
    uint32_t hw_rev_id = SFC7120_READ_REG(sc, SFC7120_REG_BIU_HW_REV_ID);

    device_printf(sc->dev, "Solarflare 7120 register dump\n");
    device_printf(sc->dev, "  HW_REV_ID  = %08x (%s) [expect %08x]\n",
                  hw_rev_id,
                  hw_rev_id == SFC7120_HW_REV_ID_RESET ? "LIVE" : "DEAD",
                  SFC7120_HW_REV_ID_RESET);
    /* TODO: extend with per-channel EVQ/RXQ/TXQ state once bringup lands. */
}
