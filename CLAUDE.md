# CheriBsdSolarflare7120 — Solarflare 7120 CAPIO Kernel Driver Stub

## What This Directory Is

Out-of-tree workspace for the Solarflare SFN7000-series (Huntington / EF10)
CAPIO kernel driver stub. Modeled on `~/CheriBsdE1000/`: cross-compile via
bmake on Linux, native make on CheriBSD, output `sfc7120pol.ko`.

**Status: ATTACHES CLEANLY WITH MAC + PHY PROGRAMMED (as of 2026-05-19).**
The PCI driver, CAPIO wiring, IOCTL dispatch, slice manifest, MCDI
transport (v1 + v2 framing), identity bringup (`GET_VERSION` /
`DRV_ATTACH` / `GET_MAC` / `GET_PORT_ASSIGNMENT` / `GET_FUNCTION_INFO` /
`GET_CAPABILITIES`), per-function reset (`ENTITY_RESET`), assertion
clearing (`GET_ASSERTS`), VI allocation (`ALLOC_VIS` — 32 VIs at base
1024 on PF1, base 1 on PF0), vAdaptor allocation
(`VADAPTOR_ALLOC` on `EVB_PORT_ID_ASSIGNED` with `PERMIT_SET_MAC_*`
cap-gated), per-queue init (`INIT_EVQ` / `INIT_RXQ` / `INIT_TXQ` —
512 descriptors each, all targeting EVQ 0), and **MAC + PHY bring-up
(`GET_PHY_CFG` / `SET_MAC` / `SET_LINK` / polled `GET_LINK`)** all
succeed. The DMA mailbox, DMA rings + packet buffers, cdev creation,
smem population, and `init_capio_sc` complete end-to-end; both PF0 and
PF1 reach `sfc7120pol attached`. With no SFP+ module / no cable / no
peer on the front faceplate, `GET_LINK` reports `link=DOWN
mac_fault=XGMII_LOCAL` for the full 3 s poll and attach continues
gracefully — this is the **expected** outcome for an unwired port, not
a driver failure (see "Bringup notes (2026-05-19)" below).

Still **TODO**: MSI-X interrupt handler + EVQ event walking
(`LINKCHANGE` handling will replace the post-`SET_LINK` `GET_LINK`
poll), TX/RX IOCTL bodies, in-tree copy at
`~/cheri/cheribsd/sys/modules/sfc7120pol/`, userspace driver at
`~/E1000Lwip/netif/sfc7120_driver.c`.

---

## Hardware Target

**Card under test:** Solarflare **SFN7322F-R2** — Flareon Ultra Dual-Port
10GbE PCIe 3.0 Server I/O Adapter (Precision Time edition).

- Controller: **SFC9120** (Huntington, EF10 family)
- Two physical 10GbE ports → two PFs (function 0 = port 0, function 1 = port 1)
- Vendor / device IDs: `0x1924` / `0x0903` (PF), `0x1924` / `0x1903` (VF)
- Subsystem: `0x1924:0x8007` (this is the "-R2 Precision Time" SKU)

`pciconf -lvb` for the unit our driver attaches against (Function 1):

```
class=0x020000 rev=0x01 hdr=0x00
vendor=0x1924 device=0x0903 subvendor=0x1924 subdevice=0x8007
    vendor     = 'AMD Solarflare'
    device     = 'SFC9120 10G Ethernet Controller'
    cap 01[40] = powerspec 3  supports D0 D1 D2 D3  current D0
    cap 05[50] = MSI supports 1 message, 64 bit
    cap 10[70] = PCI-Express 2 endpoint max data 128(2048) FLR RO NS
                 max read 512
                 link x8(x8) speed 8.0(8.0) ASPM disabled(L0s/L1)
    cap 11[b0] = MSI-X supports 32 messages
                 Table in map 0x20[0x0], PBA in map 0x20[0x2000]
    cap 03[d0] = VPD
    ecap 0001[100] = AER 2 0 fatal 1 non-fatal 1 corrected
    ecap 0003[140] = Serial 1 000f53ffff285490
    ecap 000e[150] = ARI 1
    ecap 0019[160] = PCIe Sec 1 lane errors 0
    ecap 0010[180] = SR-IOV 1 IOV disabled, Memory Space disabled, ARI disabled
                     0 VFs configured out of 0 supported
                     First VF RID Offset 0x0001, VF RID Stride 0x0001
                     VF Device ID 0x1903
                     Page Sizes: 4096 (enabled), 8192, 65536, 262144, 1048576, 4194304
    ecap 0017[1c0] = TPH Requester 1
```

Notable bits:
- **FLR is advertised** (cap 10), but **DO NOT USE IT** on this card —
  see the bringup notes below. We replace it with `MC_CMD_ENTITY_RESET`
  in `sfc7120_hw_init`. The `sfc7120_pcie_flr` helper is kept around
  but no callers — the call site in `sfc7120_fbsd_attach` is `#if 0`'d.
- **MSI-X table lives in BAR4** (`map 0x20[0x0]`). The 8 MB BAR
  (`PCIR_BAR(2)`) is the function MMIO / MCDI doorbell window —
  matches sfxge's `EFX_MEM_BAR_HUNTINGTON_PF = 2`.
- **Link is x8 / 8.0 GT/s** — full PCIe 3.0 negotiated, no degraded link.
- **SR-IOV is present but disabled** — we attach as a regular PF
  (`vf=0xffff` from `GET_FUNCTION_INFO` confirms PF identity).

### Bringup notes (2026-05-07)

**RESOLVED — FLR breaks this card.** Removing the `pcie_flr()` call at
the top of `sfc7120_fbsd_attach` makes MMIO live immediately, MCDI
responds, `MC_CMD_GET_VERSION` returns `1001.7.2.6`, `DRV_ATTACH`
succeeds with `func_flags=0x6` (LINKCTRL+TRUSTED), and
`GET_MAC_ADDRESSES` reads `00:0f:53:28:54:91`. Before the fix,
`HW_REV_ID` and the entire 8 MB BAR returned all-zeros indefinitely — a
host-driven FLR puts the SFN7322F-R2 into a state where the BIU clock
domain stays gated, even after multi-second waits. **Stock sfxge does
not FLR at attach; do not add it back without a different reset path
(MC_CMD_REBOOT via MCDI, or per-VI soft reset).** The FLR call is
currently `#if 0`'d out at the top of `sfc7120_fbsd_attach`.

A second consequence: without FLR, firmware state survives module
unload. We solve this with `MC_CMD_ENTITY_RESET (0x20)` with the
`FUNCTION_RESOURCE_RESET` flag, called between `DRV_ATTACH` and
`GET_MAC` in `sfc7120_hw_init`. ENTITY_RESET releases any VIs / queues
/ filters left tied to this PCI function from a prior load. Sfxge has
the same helper at `ef10_nic.c:2146-2155`, though it only calls it from
recovery paths; we call it on every attach since FLR isn't available.

Other small notes captured during bringup:
- **`MC_DB_LWRD` (0x200) is write-only.** Reads return zero regardless of
  what was written. The "LWRD probe" diagnostic in `sfc7120_mcdi_init`
  reports "MMIO path dead" when in fact the register is write-only —
  trust `HW_REV_ID` (0x000) for liveness, not LWRD.
- **PF1 MAC differs from the serial number's last byte by 1.** The
  pciconf serial reports the PF0 base MAC; PF1's MAC is base+1.

### Bringup notes (2026-05-08) — **The MCDI v1/v2 framing trap**

**RESOLVED — `ALLOC_VIS` finally works after fixing MCDI v2 framing.**
Multiple days of `MC_CMD_ALLOC_VIS` (opcode `0x8b`) returning bogus
errors (`EOPNOTSUPP=95`, `ENOENT=2`, seq mismatches) traced back to a
single bug in `sfc7120_mcdi_send_locked`: the v1 header truncated the
command field to 7 bits.

```c
header  = (cmd & MCDI_HDR_CODE_MASK) << MCDI_HDR_CODE_SHIFT;
                /* MCDI_HDR_CODE_MASK = 0x7f */
```

EF10 MCDI has two framings:
- **v1**: single 32-bit header, 7-bit cmd field (opcodes `0x00..0x7E`),
  8-bit datalen (payload up to 252 bytes). Payload starts at offset 4.
- **v2 escape**: v1 header carries `cmd = MC_CMD_V2_EXTN (0x7F)` and
  `datalen = 0`. A second 32-bit "ext header" follows at offset 4 with
  the real 15-bit cmd and 10-bit length. Real payload starts at offset
  8. Used for opcodes `>= 0x7F` or payloads `> 252` bytes. See sfxge
  `efx_mcdi.c:296-327` for the canonical implementation.

Every opcode `>= 0x80` was being silently truncated to its low 7 bits.
The MC was responding to a *different* command each time, which is why
the errors looked plausible but inconsistent:

| Real opcode | What we sent (cmd & 0x7F) | Symptom |
|---|---|---|
| `0x8b` ALLOC_VIS  | `0x0b` (whatever that is) | "ENOENT" / "EOPNOTSUPP" |
| `0x8c` FREE_VIS   | `0x0c` (whatever that is) | "returned: 0" (false success) |
| `0xb8` GET_PORT_ASSIGNMENT | `0x38` | EINVAL |
| `0xbe` GET_CAPABILITIES   | `0x3e` | succeeded, but bytes were not capabilities |
| `0xec` GET_FUNCTION_INFO  | `0x6c` | ENOSYS |

Hours of investigation chased phantoms — assertion-handler state,
firmware variant, lingering VI ownership, MC self-reboots — none of
which were really happening. **Lesson:** if a multi-byte field shrinks
silently in your wire format, you can spend a week debugging the
firmware rather than your encoder. When in doubt, dump the literal
bytes you put on the wire and compare against sfxge's encoder before
trusting an error code.

The v2-aware `sfc7120_mcdi_send_locked` returns the payload offset (4
for v1, 8 for v2) so `sfc7120_mcdi_exec` can read the response from
the right location. The exec wrapper also pulls `resp_datalen` from
the v2 ext header when v2 was used. See "MCDI Implementation" below.

### Bringup notes (2026-05-18) — **The MCDI byte-length alignment trap**

**RESOLVED — `VADAPTOR_ALLOC` + queue init (`INIT_EVQ` / `INIT_RXQ` /
`INIT_TXQ`) now work after removing an over-strict alignment check in
`sfc7120_mcdi_exec`.** Several days of `MC_CMD_VADAPTOR_ALLOC` (opcode
`0x98`) returning `rc=22` across every argument permutation (cap-gated
vs zero flags, AUTO_MAC vs explicit MAC) traced to:

```c
/* sfc7120_mcdi_exec — DO NOT add this back */
if ((in_len & 3) != 0)
    return EINVAL;
```

`MC_CMD_VADAPTOR_ALLOC_IN_LEN = 30` (UPSTREAM_PORT_ID 4 + gap 4 +
FLAGS 4 + NUM_VLANS 4 + NUM_VLAN_TAGS 4 + VLAN_TAGS 4 + **MACADDR 6**)
is not a multiple of 4. The pre-flight check silently rejected every
attempt **before the request ever reached the MC**. The MCDI v1
`datalen` field and v2 ACTUAL_LEN field are byte counts, not dword
counts — the firmware accepts arbitrary byte lengths, and
`mcdi_buf_write_payload` already zero-pads the trailing partial dword.

The diagnostic smoking gun: the failure had **no** `MCDI cmd 0x98
failed: MC_CMD_ERR=...` log line. That line is only emitted on the
header-error path inside `sfc7120_mcdi_exec`; its absence means the
rejection happened *inside* exec, not on the wire. Compare to
`VADAPTOR_QUERY` failures from the same debug session, which did
produce the log line (`MCDI cmd 0x61 failed: MC_CMD_ERR=4097` —
`NO_VADAPTOR`).

**Lesson:** when a translated errno appears at a call site but no
corresponding MC-error log line shows up, the failure is in your
exec/encode path, not in the firmware. Check the early-return paths
(initialization, size bounds, alignment) before debugging firmware
behavior. A long debug detour into privilege masks, EVB capabilities,
and four-variant payload probes diagnosed the wrong layer — the actual
fix was a one-line deletion.

The current `sfc7120_mcdi_vadaptor_alloc` is a single-call wrapper
matching sfxge's `efx_mcdi_vadaptor_alloc` exactly: cap-gated
`PERMIT_SET_MAC_WHEN_FILTERS_INSTALLED` flag, AUTO_MAC, no preemptive
FREE.

### Bringup notes (2026-05-19) — **MAC + PHY bring-up and the unwired-port `XGMII_LOCAL` signal**

`MC_CMD_GET_PHY_CFG` (0x24), `MC_CMD_SET_MAC` (0x2c, v1 framing /
`IN_LEN=28`), `MC_CMD_SET_LINK` (0x2a), and `MC_CMD_GET_LINK` (0x29)
all succeed on first attach. Wired into `sfc7120_hw_init` immediately
after `VADAPTOR_ALLOC`:

```
vadaptor_alloc
get_phy_cfg     — reads SUPPORTED_CAP (caches in sc->phy_supported_cap_mask)
                  and MEDIA_TYPE; applies the sfxge FEC-REQUESTED quirk
                  (firmware omits *_FEC_REQUESTED bits even when supported,
                  so we OR them in by hand).
set_mac         — MTU=1544 (= EFX_MAC_PDU(1500) = roundup(1500+38, 8)),
                  DRAIN=0, ADDR=sc->mac_addr, REJECT=0, FCNTL=AUTO,
                  FLAGS=0. EACCES from the MC is non-fatal (matches sfxge's
                  efx_mcdi_execute_quiet handling for unprivileged
                  functions); on PF1 (LINKCTRL+TRUSTED) we get 0.
set_link        — CAP = sc->phy_supported_cap_mask | PHY_CAP_AN,
                  LOOPBACK_MODE = NONE, no flags. Kicks autoneg.
get_link poll   — up to 30 × 100 ms (3 s cap) waiting for LINK_UP. Logs
                  every iteration so you can see the negotiation timeline.
                  Link-down at timeout is non-fatal; attach continues.
```

On the SFN7322F-R2 PF1 this prints:

- `MC GET_PHY_CFG: supported_cap=0x7c0 media_type=5`
  - `0x7c0` = `1000FDX | 10000FDX | PAUSE | ASYM | AN` (bits 6,7,8,9,10)
  - `media_type=5` = `MC_CMD_MEDIA_SFP_PLUS`
- `MC SET_MAC: addr=00:0f:53:28:54:91 mtu=1544 fcntl=AUTO`
- `MC SET_LINK: adv_cap=0x7c0`
- 30× `MC GET_LINK: speed=10000 Mbps link=DOWN FDX fcntl=2 mac_fault=0x1`
- `hw_init: link still DOWN after 3000 ms; continuing`

**The `mac_fault=0x1` (= `MC_CMD_MAC_FAULT_XGMII_LOCAL`) report when
nothing is plugged into the SFP+ cage is expected and not a driver
bug.** It's the local MAC saying "the local PHY isn't giving me valid
10G symbols," which is exactly what happens when there's no SFP+
module installed, no cable in the module, or no live link partner at
the other end. The firmware also reports a non-zero `LINK_SPEED`
(10000) and `FULL_DUPLEX=1` in this state — those are the *intended*
post-negotiation values, not the actual link status. Trust the
`LINK_UP` flag (bit 0 of FLAGS), not LINK_SPEED, for liveness.

Also worth noting from this run: `GET_LINK` reports `fcntl=2`
(`MC_CMD_FCNTL_BIDIR`) while link is down, even though `SET_MAC` was
told `FCNTL_AUTO`. That's the firmware's default-pre-negotiation
value; it'll be replaced by the auto-negotiated value once a peer
comes up.

**Why a synchronous poll instead of waiting for `LINKCHANGE`:** sfxge
relies on a firmware-generated `LINKCHANGE` EVQ event to update link
state and never spins on `GET_LINK`. Our EVQ event walker is still
TODO, so the poll is a temporary scaffold to get a useful bringup
log line. Marked `/* TODO: remove once interrupts land */` in
`sfc7120_hw_init`. Once the EVQ event handler is wired, the natural
flow is: program `SET_LINK`, return immediately, let the `LINKCHANGE`
event update `sc->link_up` / `sc->link_speed_mbps` /
`sc->full_duplex` asynchronously.

**Why `SET_MAC` v1 instead of v2/EXT:** the EXT framing (`IN_LEN=32`
with `CONTROL` bits) is for *selective* reconfiguration of individual
fields and requires firmware to advertise the `SET_MAC_ENHANCED`
capability. We don't currently parse that capability and don't have a
use case for selective updates, so v1 is the right choice. sfxge's
`ef10_mac_reconfigure` (`ef10_mac.c:292-368`) does the same thing —
it only goes to EXT when `efx_mcdi_mtu_set` does an isolated MTU
change. If we later need DRAIN-only toggling for graceful teardown,
that's the spot to switch to EXT.

**softc additions for this stage:** `phy_supported_cap_mask`,
`phy_adv_cap_mask`, `phy_media_type`, `link_up`, `full_duplex`,
`link_speed_mbps`, `link_fcntl`, `mac_configured`, `link_configured`.
No `phy_*_configured` teardown gating needed — `ENTITY_RESET` on the
next attach drops MAC/PHY state, and a graceful detach via DRAIN=1 +
TXDIS=1 is optional polish, not required.

### Firmware variant (this card)

`MC_CMD_GET_CAPABILITIES` on PF1 reports:

- `rxdp_fw_id = 0x0001` → `RXDP_LOW_LATENCY`
- `txdp_fw_id = 0x0001` → `TXDP_LOW_LATENCY`

This is the **low-latency datapath firmware variant**, *not* the
"full-featured" variant. Consistent with the SFN7322F-R2 being the
"Precision Time" SKU. Per-queue init (`INIT_EVQ` / `INIT_RXQ` /
`INIT_TXQ`) succeeds without `FULL_FEATURED`; the queue init flags we
currently send (EVQ: `0x39`, RXQ: `0x300`, TXQ: `0x06`) are all
accepted. Some MCDI features (notably parts of TSO, RSS-mode
extensions) remain gated behind `FULL_FEATURED` — keep this in mind
once we get to MAC/link bring-up and any optional offloads.

`func_flags = 0x6` (LINKCTRL + TRUSTED, no PRIMARY because we attach
to PF1; PF0 holds PRIMARY).

`ALLOC_VIS(min=1, max=32)` consistently grants 32 VIs starting at base
1024. `pf=1, vf=0xffff` confirms PF identity.

### BAR layout for reference

From `pciconf -lbv` (PF0 shown; PF1 is identical type/size, shifted in
address):

| RID | Offset | Type | Size | PF0 base | PF1 base |
|---|---|---|---|---|---|
| `PCIR_BAR(0)` | `0x10` | I/O | 256 B | `0x3100` | `0x3000` |
| `PCIR_BAR(2)` | `0x18` | mem64 | 8 MB | `0x60000000` | `0x60800000` |
| `PCIR_BAR(4)` | `0x20` | mem64 | 16 KB | `0x61000000` | `0x61004000` |

`PCIR_BAR(2)` (8 MB) is the function MMIO window — matches sfxge's
`EFX_MEM_BAR_HUNTINGTON_PF = 2`. `PCIR_BAR(4)` (16 KB) holds the MSI-X
table+PBA per the `cap 11` line of `pciconf`. The 256 B I/O BAR0 we
don't currently use.

---

## Hardware Target

**Card under test:** Solarflare **SFN7322F-R2** — Flareon Ultra Dual-Port
10GbE PCIe 3.0 Server I/O Adapter (Precision Time edition).

- Controller: **SFC9120** (Huntington, EF10 family)
- Two physical 10GbE ports → two PFs (function 0 = port 0, function 1 = port 1)
- Vendor / device IDs: `0x1924` / `0x0903` (PF), `0x1924` / `0x1903` (VF)
- Subsystem: `0x1924:0x8007` (this is the "-R2 Precision Time" SKU)

`pciconf -lvb` for the unit our driver attaches against (Function 1):

```
class=0x020000 rev=0x01 hdr=0x00
vendor=0x1924 device=0x0903 subvendor=0x1924 subdevice=0x8007
    vendor     = 'AMD Solarflare'
    device     = 'SFC9120 10G Ethernet Controller'
    cap 01[40] = powerspec 3  supports D0 D1 D2 D3  current D0
    cap 05[50] = MSI supports 1 message, 64 bit
    cap 10[70] = PCI-Express 2 endpoint max data 128(2048) FLR RO NS
                 max read 512
                 link x8(x8) speed 8.0(8.0) ASPM disabled(L0s/L1)
    cap 11[b0] = MSI-X supports 32 messages
                 Table in map 0x20[0x0], PBA in map 0x20[0x2000]
    cap 03[d0] = VPD
    ecap 0001[100] = AER 2 0 fatal 1 non-fatal 1 corrected
    ecap 0003[140] = Serial 1 000f53ffff285490
    ecap 000e[150] = ARI 1
    ecap 0019[160] = PCIe Sec 1 lane errors 0
    ecap 0010[180] = SR-IOV 1 IOV disabled, Memory Space disabled, ARI disabled
                     0 VFs configured out of 0 supported
                     First VF RID Offset 0x0001, VF RID Stride 0x0001
                     VF Device ID 0x1903
                     Page Sizes: 4096 (enabled), 8192, 65536, 262144, 1048576, 4194304
    ecap 0017[1c0] = TPH Requester 1
```

Notable bits:
- **FLR is supported** (cap 10) — `pcie_flr()` should work; we use it before
  BAR alloc.
- **MSI-X table lives in BAR4** (`map 0x20[0x0]`), so BAR4 is *not* the
  function-control window. The 8 MB BAR (currently mapped via `PCIR_BAR(2)`)
  is the candidate for the MMIO/MCDI doorbell window — but see "Open
  question" below.
- **Link is x8 / 8.0 GT/s** — full PCIe 3.0 negotiated, no degraded link.
- **SR-IOV is present but disabled** — we attach as a regular PF.

### Bringup notes (2026-05-07)

**RESOLVED — FLR breaks this card.** Removing the `pcie_flr()` call at
the top of `sfc7120_fbsd_attach` makes MMIO live immediately, MCDI
responds, `MC_CMD_GET_VERSION` returns `1001.7.2.6`, `DRV_ATTACH`
succeeds with `func_flags=0x6` (LINKCTRL+TRUSTED), and
`GET_MAC_ADDRESSES` reads `00:0f:53:28:54:91`. Before the fix,
`HW_REV_ID` and the entire 8 MB BAR returned all-zeros indefinitely — a
host-driven FLR puts the SFN7322F-R2 into a state where the BIU clock
domain stays gated, even after multi-second waits. **Stock sfxge does
not FLR at attach; do not add it back without a different reset path
(MC_CMD_REBOOT via MCDI, or per-VI soft reset).** The FLR call is
currently `#if 0`'d out at the top of `sfc7120_fbsd_attach`.

A second consequence: without FLR, firmware state survives module
unload. If a previous load `DRV_ATTACH`'d and was unloaded without a
matching `FREE_VIS`, the firmware still believes the previous driver
owns those VIs, and `MC_CMD_ALLOC_VIS` returns `MC_CMD_ERR=95`
(`EOPNOTSUPP` / errno 45). Fix: unconditionally issue
`MC_CMD_FREE_VIS` before `MC_CMD_ALLOC_VIS` (sfxge does this — see
`ef10_nic.c:2218-2220`). Already wired in `sfc7120_mcdi_alloc_vis`.

Other small notes captured during bringup:
- **`MC_DB_LWRD` (0x200) is write-only.** Reads return zero regardless of
  what was written. The "LWRD probe" diagnostic in `sfc7120_mcdi_init`
  reports "MMIO path dead" when in fact the register is write-only —
  trust `HW_REV_ID` (0x000) for liveness, not LWRD.
- **PF1 MAC differs from the serial number's last byte by 1.** The
  pciconf serial reports the PF0 base MAC; PF1's MAC is base+1.

### Earlier (now-resolved) symptom log

Prior to disabling FLR, attach symptoms were:

- BAR layout from `pciconf -lbv` (PF0 shown; PF1 is identical type/size,
  shifted in address):

  | RID | Offset | Type | Size | PF0 base | PF1 base |
  |---|---|---|---|---|---|
  | `PCIR_BAR(0)` | `0x10` | I/O | 256 B | `0x3100` | `0x3000` |
  | `PCIR_BAR(2)` | `0x18` | mem64 | 8 MB | `0x60000000` | `0x60800000` |
  | `PCIR_BAR(4)` | `0x20` | mem64 | 16 KB | `0x61000000` | `0x61004000` |

  `PCIR_BAR(2)` (8 MB) is the function MMIO window — matches sfxge's
  `EFX_MEM_BAR_HUNTINGTON_PF = 2`. `PCIR_BAR(4)` (16 KB) holds the MSI-X
  table+PBA per the `cap 11` line of `pciconf`. So the BAR choice is
  correct; "wrong BAR" is ruled out.
- `PCIR_COMMAND` shows `MEMEN=1, BUSMASTER=1` both before and after the
  failed command (so the device hasn't disabled itself mid-command).
- Doorbell write order matches sfxge `ef10_mcdi_send_request` (high half
  to LWRD at `0x200`, then low half to HWRD at `0x204` as the trigger).
- The "doorbell-clear" init kick (HWRD ← 1) from `ef10_mcdi_init` is
  performed.

Remaining candidates:

1. **Device not finished coming out of reset.** `pcie_flr()` returns at
   1 s but Huntington's MC firmware can take several seconds to re-boot.
   Until the BIU clock domain is fully up, MMIO reads return `0`. Fix:
   after FLR, poll `HW_REV_ID` for up to ~5 s waiting for `0xeb14face`
   before issuing any MCDI command.
2. **CHERI `bus_space` capability bounds.** A scan over the first 4 KB
   would distinguish this from (1): all-zeros = hardware-not-up;
   `0xffffffff` past some offset = capability bounds issue.
3. **Per-function init order.** Some EF10 firmware variants gate PF1
   MMIO behind PF0 MCDI bringup. Worth ruling out by attaching to PF0
   instead, or leaving stock `sfxge` on PF0 while we drive PF1.

Next debug step is the BAR magic-value scan that's already wired into
`sfc7120_mcdi_init` (uncommitted local change as of 2026-05-07) plus a
post-FLR wait loop on `HW_REV_ID`.

---

## Reference Reading (Required)

Before modifying anything in this directory, read:

1. `~/CheriBsdE1000/CLAUDE.md` — first CAPIO driver, simpler reference
2. `~/cheri/cheribsd/sys/modules/mlx5pol/CLAUDE.md` — complex CAPIO driver
   with firmware command channel; the closest existing pattern for EF10's
   MCDI

The CAPIO conventions (mandatory softc layout, attach/detach ordering,
slice manifest semantics, IOCTL token validation) are documented there
and apply identically here.

---

## MCDI Implementation

Lives in `sfc7120_mcdi.c` / `sfc7120_mcdi.h`. Reference copy of the EF10
MCDI command catalogue is checked in as `ref_efx_regs_mcdi.h` (verbatim
from `~/cheri/cheribsd/sys/dev/sfxge/common/efx_regs_mcdi.h`); a small
sfxge MCDI excerpt is in `ref_sfxge_mcdi.c`.

### Wire format

EF10 MCDI is a synchronous request/response protocol over a DMA-coherent
mailbox plus a doorbell pair. Mailbox is 256 bytes, 256-byte aligned
(sfxge bug24769 — the doorbell recovery algorithm requires the low byte
of the mailbox physaddr to be 0). The MC reads our request out of the
mailbox via DMA, processes, writes the response back, and we poll the
RESPONSE bit of the header word.

**v1 framing** — single 32-bit header at offset 0:

```
bits[ 6:0]   command code (7-bit; opcodes 0x00..0x7E only)
bit  [7]     resync (always 0 for ordinary requests)
bits[15:8]   payload length in bytes (0..252)
bits[19:16]  4-bit sequence number
bit  [21]    NOT_EPOCH (0 = first command of a fresh epoch)
bit  [22]    error    (response only)
bit  [23]    response (set by MC in the response)
bits[31:24]  xflags
```

Payload starts at offset 4.

**v2 escape** — used for opcodes `>= 0x7F` or payloads `> 252` bytes:

```
offset 0: v1 header with cmd = MC_CMD_V2_EXTN (0x7F), datalen = 0
offset 4: v2 ext header
            bits[14:0]   real 15-bit cmd
            bit  [15]    unused
            bits[25:16]  real 10-bit payload length
offset 8: payload
```

Responses use the same framing as the request: if v2 was used to send,
the v1 response header still carries `cmd=0x7F` / `datalen=0`, with the
real response length in the v2 ext header at offset 4 and the payload
at offset 8.

`sfc7120_mcdi_send_locked` picks v1 vs v2 automatically based on cmd
size and `in_len`; it returns the byte offset where the payload begins
(4 or 8) so `sfc7120_mcdi_exec` can read the response from the right
place. Critical: every opcode used in this driver `>= 0x80` — including
`ALLOC_VIS (0x8b)`, `FREE_VIS (0x8c)`, `GET_PORT_ASSIGNMENT (0xb8)`,
`GET_CAPABILITIES (0xbe)`, `GET_FUNCTION_INFO (0xec)` — needs v2.
**Do not "optimize" out the v2 path** without re-validating the cmd
range; see the MCDI v1/v2 framing trap note above for what happens.

### Sequence numbers and epochs

`sc->mcdi_seq` is a 4-bit counter incremented after every exec (mod 16).
`sc->mcdi_new_epoch` controls `NOT_EPOCH` in the header: cleared on the
first command of a fresh transport, set thereafter so the MC sequence
tracker stays in sync. After a confirmed MC reboot, both must be reset
(`mcdi_seq = 0`, `mcdi_new_epoch = true`); right now only
`sfc7120_mcdi_reboot_after_assertion` does this, and it isn't actively
called.

### Helpers (`sfc7120_mcdi.h`)

| Function | Purpose |
|---|---|
| `sfc7120_pcie_flr` | UNUSED; keeps the FLR helper around. Calling it bricks the BIU clock on this card; see "Bringup notes". |
| `sfc7120_mcdi_init` / `_fini` | Allocate/free the mailbox, init/destroy the lock + epoch state, do the doorbell-clear init kick. |
| `sfc7120_mcdi_exec` | Synchronous request/response with timeout (10s) + seq + epoch handling. Routes through v1 or v2 framing. |
| `sfc7120_mcdi_log_mc_state(sc, tag)` | Read & log `HW_REV_ID` and `MC_SFT_STATUS`. Useful between MCDI steps to catch MC reboots. |
| `sfc7120_mcdi_clear_assertions` | `MC_CMD_GET_ASSERTS (0x06)` with CLEAR=1. Drains any pending firmware assertion. Mirrors sfxge `efx_mcdi_read_assertion`. |
| `sfc7120_mcdi_reboot_after_assertion` | `MC_CMD_REBOOT (0x3D)` with `AFTER_ASSERTION` flag. **Defined but not currently called** — kept available as an MC reboot recovery path. |
| `sfc7120_mcdi_entity_reset` | `MC_CMD_ENTITY_RESET (0x20)` with `FUNCTION_RESOURCE_RESET`. Per-function reset that releases lingering VIs/queues/filters. **The thing that lets us re-attach without FLR.** |
| `sfc7120_mcdi_get_version` | `MC_CMD_GET_VERSION (0x08)`. Logs `1001.7.2.6` on this card. |
| `sfc7120_mcdi_drv_attach` / `_drv_detach` | `MC_CMD_DRV_ATTACH (0x1C)`. Attach with FW_DONT_CARE. |
| `sfc7120_mcdi_get_mac` | `MC_CMD_GET_MAC_ADDRESSES (0x55)`. |
| `sfc7120_mcdi_dump_func_info` | Diagnostic. Runs `GET_PORT_ASSIGNMENT (0xB8)`, `GET_FUNCTION_INFO (0xEC)`, `GET_CAPABILITIES (0xBE)` and dumps the result. Fires on every attach. |
| `sfc7120_mcdi_alloc_vis` / `_free_vis` | `MC_CMD_ALLOC_VIS (0x8B)` / `MC_CMD_FREE_VIS (0x8C)`. Currently asks for `min=1, max=32`. |
| `sfc7120_mcdi_vadaptor_alloc` / `_free` | `MC_CMD_VADAPTOR_ALLOC (0x98)` / `MC_CMD_VADAPTOR_FREE (0x99)` on `EVB_PORT_ID_ASSIGNED`. Cap-gates `PERMIT_SET_MAC_WHEN_FILTERS_INSTALLED` (FLAGS1 bit 11). AUTO_MAC. **Do not add an alignment check to `sfc7120_mcdi_exec`** — `IN_LEN=30` is not dword-aligned; see "MCDI byte-length alignment trap" above. |
| `sfc7120_mcdi_init_evq` / `_fini_evq` | `MC_CMD_INIT_EVQ (0x80)` / `MC_CMD_FINI_EVQ (0x83)`. Currently EVQ 0, 512 entries, ring backed by a single page from `sc->evq_ring_paddr`. Wire flags = `0x39`. |
| `sfc7120_mcdi_init_rxq` / `_fini_rxq` | `MC_CMD_INIT_RXQ (0x81)` / `MC_CMD_FINI_RXQ (0x84)`. Currently RXQ 0 targeting EVQ 0, 512 descriptors. Wire flags = `0x300`. `PORT_ID = EVB_PORT_ID_ASSIGNED`. |
| `sfc7120_mcdi_init_txq` / `_fini_txq` | `MC_CMD_INIT_TXQ (0x82)` / `MC_CMD_FINI_TXQ (0x85)`. Currently TXQ 0 targeting EVQ 0, 512 descriptors. Wire flags = `0x06`. `PORT_ID = EVB_PORT_ID_ASSIGNED`. |

### Error code translation

`sfc7120_mcdi_xlate_err` maps the MC error byte from the response
payload into a FreeBSD errno. Updated as we hit each one:

| `MC_CMD_ERR_*` | errno |
|---|---|
| 0 | 0 (success) |
| 1 EPERM | EPERM |
| 2 ENOENT | ENOENT |
| 4 EINTR | EINTR |
| 5 EIO | EIO |
| 11 EAGAIN | EAGAIN |
| 12 ENOMEM | ENOMEM |
| 13 EACCES | EACCES |
| 16 EBUSY | EBUSY |
| 22 EINVAL | EINVAL |
| 38 ENOSYS | ENOSYS |
| 62 ETIMEDOUT | ETIMEDOUT |
| 95 EOPNOTSUPP | EOPNOTSUPP |
| (anything else) | EIO |

If a future bringup step turns up an MC error code not in this table,
add it here so log lines show a meaningful errno.

### Current `sfc7120_hw_init` order

```
mcdi_init                 (alloc mailbox, init epoch, prime doorbell)
clear_assertions          (drain any pre-existing firmware assertion)
get_version               (log fw version)
drv_attach                (declare us as the function's driver)
entity_reset              (release any lingering function-scope state)
get_mac                   (populate sc->mac_addr)
dump_func_info            (port, pf/vf, capabilities; caches FLAGS1 into
                           sc->mcdi_cap_flags1 so vadaptor_alloc can
                           cap-gate the PERMIT_SET_MAC_* flag)
alloc_vis(1, 32)          (32 VIs at base 1024 on PF1; base 1 on PF0)
vadaptor_alloc            (binds vAdaptor to EVB_PORT_ID_ASSIGNED with
                           PERMIT_SET_MAC_WHEN_FILTERS_INSTALLED if
                           FLAGS1 bit 11 advertised — it is on this fw)
```

After `hw_init` returns, `sfc7120_fbsd_attach` calls
`alloc_dma_resources` (EVQ ring, RX/TX desc rings, RX/TX packet
buffers) and then `init_evq(0) → init_rxq(0, target=0) → init_txq(0,
target=0)` to program one event queue + one RX queue + one TX queue,
each 512 entries. Order matters — RXQ/TXQ name EVQ 0 as their target,
so EVQ 0 must exist first; symmetrically, `fini_txq` / `fini_rxq` run
before `fini_evq` in teardown.

The failure-path-only `log_mc_state` calls in `sfc7120_hw_init` make
MC reboots obvious if a step regresses. They only fire when something
breaks, so the success path is no longer noisy.

---

## Role in the Larger System

```
Userspace (E1000Lwip/netif/sfc7120_driver.c — TO BE WRITTEN)
  │  open("/dev/sfc7120pol")
  │  ioctl(CAPIO_ATTACH)          ──────► Kernel (this module)
  │                                         seals userspace cap
  │◄── returns sealed CHERI capability ────
  │
  │  mmap(SFC7120_TX_BUFFER /     ──────► capio_mmap_single_extra()
  │       SFC7120_RX_BUFFER /                validates token, builds
  │       SFC7120_MMIO_REGION)                bounded VM object
  │◄── userspace VA backed by NIC DMA/MMIO
  │
  │  TX/RX: write descriptor ring + ring doorbell (no syscalls)
```

After `CAPIO_ATTACH` + mmap, the userspace driver pushes TX descriptors
and reads RX events directly through the CHERI capability. The kernel is
not in the data path; CHERI hardware enforces the bounds of every
capability.

---

## File Map

| File | Purpose |
|---|---|
| `sfc7120.c` | PCI driver: probe/attach/detach, vtable, IOCTL dispatch, `hw_init` orchestration |
| `sfc7120.h` | Softc, region enum (`SFC7120_TX_BUFFER` / `_RX_BUFFER` / `_MMIO_REGION`), IOCTL request structs |
| `sfc7120_mcdi.c` | MCDI transport: mailbox alloc, v1/v2 framing, exec wrapper, all `MC_CMD_*` helpers |
| `sfc7120_mcdi.h` | Public MCDI prototypes (init, exec, command wrappers, diagnostics) |
| `sfc7120_mmio.h` | Register offsets, `SFC7120_READ_REG` / `SFC7120_WRITE_REG` |
| `sfc7120_mmio.c` | `sfc7120_dump_regs()` — diagnostic register dump |
| `sfc7120_tables.c` | Slice manifest (`sfc7120_reg_slices[]`) for the MMIO region |
| `capio.c` | Local copy of CAPIO framework (mirrors `~/CheriBsdE1000/capio.c`) |
| `capio.h` | CAPIO types: `capio_softc_t`, `slice_def_t`, etc. |
| `ref_efx_regs_mcdi.h` | Verbatim copy of sfxge's MCDI command catalogue (`MC_CMD_*` opcodes, IN/OUT structs). Don't modify; re-import from sfxge if you need newer commands. |
| `ref_sfxge_mcdi.c` | Excerpt of sfxge's MCDI helper layer for cross-reference. |
| `Makefile` | FreeBSD kmod Makefile; cross-compile via `CROSS_COMPILE=1` |
| `build.sh` | Detects Linux/FreeBSD, checks deps, invokes bmake or make |
| `install.sh` | SCPs the built `.ko` to the Morello board |
| `.gitignore`, `README.md` | Standard housekeeping |

`capio.c`/`capio.h` are local copies (not VPATH-shared) because this is an
out-of-tree workspace. If you change them, sync to the in-tree version
once it exists, and to `~/CheriBsdE1000/` if the change is generic.

---

## The Mandatory CAPIO Softc Layout

Same constraint as the other CAPIO drivers — `capio.c` casts `void *sc`
straight to `capio_softc_header_t *`:

```c
typedef struct {
    device_t      dev;        /* offset 0 — MUST be first */
    capio_softc_t capio_sc;   /* immediately after — MUST be second */
} capio_softc_header_t;
```

`sfc7120_softc_t` (`sfc7120.h`) opens with exactly:

```c
typedef struct sfc7120_softc {
    device_t            dev;
    capio_softc_t       capio_sc;
    shared_mem_region_t smem[SFC7120_REGION_COUNT];
    /* ... hardware fields ... */
} sfc7120_softc_t;
```

Anything before `dev` or between `dev` and `capio_sc` will panic.

---

## Memory Regions (Initial Set)

Three regions, mirroring the e1000 stub. Multi-queue can be added later
following the mlx5pol per-queue pattern.

| Index | Enum | `is_physical` | Size | Sliced? | Description |
|---|---|---|---|---|---|
| 0 | `SFC7120_TX_BUFFER` | false | 64 × 2048 = 128 KB | No | DMA TX packet buffer (kernel VA) |
| 1 | `SFC7120_RX_BUFFER` | false | 64 × 2048 = 128 KB | No | DMA RX packet buffer (kernel VA) |
| 2 | `SFC7120_MMIO_REGION` | true | BAR0 size | Yes | NIC register space (BAR0 physical addr) |

The MMIO slice manifest (`sfc7120_reg_slices[]`) starts with the registers
a CAPIO userspace driver minimally needs:

| Register | Direction | Notes |
|---|---|---|
| `MC_DOORBELL` (`0x0e80`) | RW | Kicks MCDI requests |
| `MC_EVENT`   (`0x0e84`) | RO | MC status |
| `EVQ_RPTR_DBL` (`0x0500`) | RW | Ack events |
| `RX_DESC_DBL`  (`0x0510`) | RW | Push RX descriptor producer pointer |
| `TX_DESC_DBL`  (`0x0518`) | RW | Push TX descriptor producer pointer |
| `HW_REV_ID`    (`0x0010`) | RO | Sanity check at attach |
| `MC_STATUS`    (`0x0c7c`) | RO | Health |

**These offsets are placeholders.** Cross-check against the EF10 register
documentation for your specific SFN7xxx board (and against the FreeBSD
`sfxge` driver in `~/cheri/cheribsd/sys/dev/sfxge/`) before treating them
as authoritative.

---

## What Is and Isn't Implemented

| Component | State | Notes |
|---|---|---|
| PCI probe | Done | `sfc7120_fbsd_devs[]` matches vendor 0x1924 + device IDs 0x0903 (PF) / 0x1903 (VF). Verified against `pciconf -lv` for SFN7322F-R2. |
| Attach ordering | Done | BAR alloc → hw_init → DMA alloc → make_dev_capio → smem populate → init_capio_sc → vm_object_handle alloc — all observed in the dmesg trace. |
| Detach ordering | Done | dying flag → free handles → modmap_unregister → destroy_dev → free DMA → hw_teardown → release BAR → capio_destroy → destroy locks |
| `capio_ops_t` vtable | Done | `ioctl`, `get_buffer_size`, `is_dying` all wired |
| Slice manifest | Skeleton | 7 entries; extend for multi-queue |
| Cdev (`/dev/sfc7120pol`) | Done | open/close/poll/ioctl wired |
| MCDI transport (v1 + v2) | Done | `sfc7120_mcdi.c`. Mailbox alloc, framing, exec wrapper, error xlate. **Do not break v2 framing** — see "MCDI v1/v2 framing trap" above. |
| MCDI identity bringup | Done | `GET_VERSION`, `DRV_ATTACH`, `GET_MAC`, `GET_PORT_ASSIGNMENT`, `GET_FUNCTION_INFO`, `GET_CAPABILITIES`, `GET_ASSERTS`. Populates `sc->mac_addr`. |
| Per-function reset | Done | `MC_CMD_ENTITY_RESET` between `DRV_ATTACH` and `GET_MAC`; replaces FLR for clean re-attach. |
| VI allocation | Done | `MC_CMD_ALLOC_VIS(min=1, max=32)` returns 32 VIs at base 1024. Stored in `sc->vi_count` / `sc->vi_base`. |
| MCDI mailbox DMA | Done | 256-byte aligned, BUS_DMA_COHERENT. Allocated in `sfc7120_mcdi_init`. |
| TX/RX packet-buffer DMA | Done | `sfc7120_alloc_dma_resources` allocates EVQ ring, RX desc ring, TX desc ring (one page each), plus 1 MB RX and 1 MB TX packet buffers, all BUS_DMA_COHERENT. Physical addresses cached in `sc->evq_ring_paddr` / `sc->rx_desc_ring_paddr` / `sc->tx_desc_ring_paddr` etc. for the queue-init MCDI calls. |
| vAdaptor allocation | Done | `MC_CMD_VADAPTOR_ALLOC` on `EVB_PORT_ID_ASSIGNED` with `PERMIT_SET_MAC_WHEN_FILTERS_INSTALLED` cap-gated. Freed in `hw_teardown`. See "MCDI byte-length alignment trap" — `IN_LEN=30` is not dword-aligned and **must not** be pre-rejected in `sfc7120_mcdi_exec`. |
| EVQ init | Done | `MC_CMD_INIT_EVQ (0x80)` programs EVQ 0 with 512 entries, flags `0x39`, ring at `sc->evq_ring_paddr`. `fini_evq` is called in teardown. |
| RXQ init | Done | `MC_CMD_INIT_RXQ (0x81)` programs RXQ 0 → EVQ 0, 512 descriptors, flags `0x300`, ring at `sc->rx_desc_ring_paddr`, `PORT_ID = EVB_PORT_ID_ASSIGNED`. |
| TXQ init | Done | `MC_CMD_INIT_TXQ (0x82)` programs TXQ 0 → EVQ 0, 512 descriptors, flags `0x06`, ring at `sc->tx_desc_ring_paddr`, `PORT_ID = EVB_PORT_ID_ASSIGNED`. |
| PHY config discovery | Done | `MC_CMD_GET_PHY_CFG (0x24)` caches `SUPPORTED_CAP` and `MEDIA_TYPE` into `sc->phy_supported_cap_mask` / `sc->phy_media_type`. Applies the sfxge FEC-REQUESTED quirk. On SFN7322F-R2 PF1: `supported_cap=0x7c0` (`1000FDX|10000FDX|PAUSE|ASYM|AN`), `media_type=5` (`SFP_PLUS`). |
| MAC reconfigure | Done | `MC_CMD_SET_MAC (0x2c)` v1 framing (`IN_LEN=28`). MTU=1544 (= `EFX_MAC_PDU(1500)`), DRAIN=0, ADDR=`sc->mac_addr`, REJECT=0, FCNTL=AUTO, FLAGS=0. EACCES is non-fatal. Mirrors sfxge `ef10_mac_reconfigure`. |
| Link bring-up | Done | `MC_CMD_SET_LINK (0x2a)` advertises `sc->phy_supported_cap_mask \| PHY_CAP_AN`, LOOPBACK=NONE. `MC_CMD_GET_LINK (0x29)` parses both v1 (`OUT_LEN=28`) and v2 (`OUT_LEN=44`) responses into `sc->link_up` / `sc->link_speed_mbps` / `sc->full_duplex` / `sc->link_fcntl`. A 3 s / 30 × 100 ms `GET_LINK` poll in `hw_init` covers the gap until the EVQ event handler is wired — link-down at timeout is non-fatal. See "Bringup notes (2026-05-19)" for the unwired-port `XGMII_LOCAL` behavior. |
| Interrupt handler | **TODO** | Skeleton (`sfc7120_interrupt_handler`, `sfc7120_rx_task_handler`) exists but unused — will be wired once MSI-X allocation lands. Wiring this also lets us delete the post-`SET_LINK` poll loop in `hw_init` (replaced by `LINKCHANGE` event handling). |
| TX/RX IOCTLs (`SFC7120_TX`, `SFC7120_RX`) | **TODO** | Return ENOSYS today |
| `SFC7120_GET_MAC` | Done | Returns `sc->mac_addr` populated by `MC_CMD_GET_MAC_ADDRESSES`. |
| In-tree copy | Not yet | Add to `~/cheri/cheribsd/sys/modules/sfc7120pol/` once queue init lands. |
| Userspace counterpart | Not yet | Add to `~/E1000Lwip/netif/sfc7120_driver.c` (mirror `mlx5_driver.c`). |

---

## Recommended Implementation Order

✅ **`sfc7120_alloc_dma_resources` / `_free_dma_resources`** — runs
clean per dmesg. Worth auditing once before queue init that it actually
allocates the rings/buffers we'll need (EVQ ring, TX desc ring, RX desc
ring, TX packet buffer, RX packet buffer; MCDI mailbox is allocated
separately by `sfc7120_mcdi_init`).

✅ **MCDI transport** — `sfc7120_mcdi.c`. v1 + v2 framing, exec wrapper,
seq/epoch handling, error xlate. See "MCDI Implementation" above.

✅ **MCDI identity bringup** — `GET_VERSION`, `DRV_ATTACH`,
`ENTITY_RESET`, `GET_MAC`, `GET_PORT_ASSIGNMENT`, `GET_FUNCTION_INFO`,
`GET_CAPABILITIES`, `ALLOC_VIS`. Currently grants 32 VIs at base 1024
on PF1 (base 1 on PF0).

✅ **vAdaptor allocation** — `MC_CMD_VADAPTOR_ALLOC` on
`EVB_PORT_ID_ASSIGNED`. Cap-gates `PERMIT_SET_MAC_WHEN_FILTERS_INSTALLED`.
Single-call wrapper that mirrors sfxge's `efx_mcdi_vadaptor_alloc`.

✅ **Per-queue MCDI init** — `MC_CMD_INIT_EVQ`, `MC_CMD_INIT_RXQ`,
`MC_CMD_INIT_TXQ`. Currently one queue of each, 512 descriptors,
ring DMA addresses come from `sfc7120_alloc_dma_resources`. (We did
**not** end up needing `ALLOC_BUFTBL_CHUNK` / `PROGRAM_BUFTBL_ENTRIES`
for this firmware variant — `INIT_*Q` accepted the ring paddrs
directly.) Reference: `ef10_ev.c`, `ef10_rx.c`, `ef10_tx.c` in sfxge.

✅ **MAC config + link bring** — `MC_CMD_GET_PHY_CFG`,
`MC_CMD_SET_MAC` (v1, 28 bytes), `MC_CMD_SET_LINK`, polled
`MC_CMD_GET_LINK`. Wired into `sfc7120_hw_init` right after
`vadaptor_alloc`. The poll is temporary scaffolding pending the EVQ
event handler. See "Bringup notes (2026-05-19)".

🔜 **Interrupt handler** — MSI-X allocation, EVQ event walking. The
`sfc7120_interrupt_handler` and `sfc7120_rx_task_handler` skeletons
already exist (currently unused, hence the `-Wunused-function`
warnings during build — those will go away once they're wired).
Wiring this also replaces the post-`SET_LINK` `GET_LINK` poll in
`hw_init` with `LINKCHANGE` event handling, the way sfxge does it.

🔜 **TX/RX IOCTLs** — kernel-mediated fallback paths for testing before
the userspace driver lands.

🔜 **Userspace driver** — `~/E1000Lwip/netif/sfc7120_driver.c`, registers
as a lwIP `netif`. Mirror `~/E1000Lwip/netif/mlx5_driver.c`.

🔜 **In-tree copy** — once stable, copy under
`~/cheri/cheribsd/sys/modules/sfc7120pol/` so it builds into the
CheriBSD image. Use the mlx5pol-style 7-line Makefile with VPATH
pointing at `../em_pol_test` for `capio.c`. **At that point sync the
local `capio.c` / `capio.h` with the in-tree version.**

---

## Build

### Cross-compile (Linux → Morello):
```bash
cd ~/CheriBsdSolarflare7120
./build.sh build
mv sfc7120pol.ko ~/mod_out/    # match the convention used by other modules
```

### Native (on CheriBSD board):
```bash
./build.sh build            # CROSS_COMPILE=0
sudo ./build.sh install     # kldunload old, kldload new
```

### Compilation database (clangd):
```bash
./build.sh bear
```

### Dependencies:

| Module | Why |
|---|---|
| `modmap.ko` | `init_capio_sc` registers modmap callbacks; modmap must load first |
| FreeBSD `pci` KLD | Standard PCI bus methods |
| Stock `sfxge` | If loaded, will conflict for the same device. `kldunload sfxge` before `kldload sfc7120pol`. |

---

## Devices Supported (placeholder)

| Vendor | Device ID | Description |
|---|---|---|
| 0x1924 | 0x0903 | Solarflare SFC9120 10G Ethernet Controller (EF10) PF |
| 0x1924 | 0x1903 | Solarflare SFC9120 10G Ethernet Controller (EF10) VF |

Confirm against the actual board with `pciconf -lv` and adjust
`sfc7120_fbsd_devs[]` in `sfc7120.c`.

---

## Known Gaps

- **Only one EVQ / RXQ / TXQ.** We program a single triplet at
  instance 0. Multi-queue (one queue per VI, RSS, etc.) is future
  work; the slice manifest will need extending to expose multiple
  doorbell windows then.
- **Link comes up only with a real peer.** `SET_MAC` / `SET_LINK` run
  on attach, but the SFN7322F-R2 needs an SFP+ module + cable + live
  10G peer at the other end before `GET_LINK` reports `LINK_UP`. With
  the cage empty (or cable unplugged), `GET_LINK` reports
  `link=DOWN mac_fault=XGMII_LOCAL` and the 3-second poll in `hw_init`
  times out gracefully. This is **expected**, not a driver bug —
  attach still completes and the rest of the bringup is reachable.
  The driver will pick up the link the moment a peer comes up once
  the EVQ event handler is wired (LINKCHANGE event).
- **GET_LINK polling is temporary scaffolding.** The 30-iteration
  `GET_LINK` loop in `sfc7120_hw_init` exists because we don't yet
  parse EVQ events. Delete the loop and rely on the LINKCHANGE event
  handler the moment the interrupt path is functional.
- **No interrupt path.** MSI-X allocation + EVQ walking still TODO.
  `sfc7120_interrupt_handler` is defined but unused (compiler warning).
  EVQ 0 is currently programmed with `INTERRUPTING` set (flags `0x39`
  includes bit 3) but no MSI-X vector is allocated, so events would
  back up if traffic flowed today.
- **`sfc7120_mcdi_reboot_after_assertion` is dead code right now.**
  Defined but not called. Decide: keep as a recovery utility, or
  delete. Removing requires no callers; keep if you anticipate needing
  to recover from MC reboot mid-run.
- **Slice manifest offsets are placeholders.** Confirm the doorbell /
  ring-pointer offsets against the EF10 register layout in sfxge before
  exposing them to userspace. Wrong offsets here become a CHERI bounds
  violation at the userspace driver, not a kernel panic — easy to miss.
- **No in-tree copy.** Add `~/cheri/cheribsd/sys/modules/sfc7120pol/`
  with VPATH-shared `capio.c` once driver stabilizes.
- **No userspace driver.** Add `~/E1000Lwip/netif/sfc7120_driver.c`.
- `vm_object_handle_t` allocations *are* freed in detach (the loop is
  in `sfc7120_fbsd_detach` before `destroy_dev`) — flagging this only
  because the e1000 out-of-tree copy is missing that loop.
