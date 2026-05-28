# CheriBsdSolarflare7120 — Solarflare 7120 CAPIO Kernel Driver + Userspace Library

## Status (2026-05-26)

Out-of-tree workspace for the Solarflare SFN7000-series (Huntington / EF10)
CAPIO driver stack. Contains two components that live together in this repo:

- **Kernel module** (`sfc7120pol.ko`) — PCI driver, MCDI firmware transport,
  queue init, CAPIO wiring. Cross-compile via bmake on Linux, native make on
  CheriBSD. Modeled on `~/CheriBsdE1000/` for the kernel conventions.
- **Userspace library** (`userlib/sfc7120_user.c`) — hand-rolled
  Ethernet/IP/UDP driver that opens `/dev/sfc7120pol`, attaches via CAPIO,
  and communicates directly with the NIC through mmapped DMA buffers and MMIO
  register slices. No lwIP dependency. Phase 1 drives TX/RX through ioctls;
  phase 3 removes the kernel from the data path entirely via direct EVQ
  polling.

**TODO:** EVQ event delivery unexercised on hardware (no `EVQ ...` dmesg
line yet — no traffic, link was already up at attach). TX-completion reaping;
TX/RX IOCTL bodies; in-tree copy; userspace driver.


## Reference Reading (Required)

Before modifying anything here, read:
1. `~/CheriBsdE1000/CLAUDE.md` — first CAPIO driver, simpler reference
2. `~/cheri/cheribsd/sys/modules/mlx5pol/CLAUDE.md` — closest pattern for EF10 MCDI

## Hardware Target

**Card:** Solarflare **SFN7322F-R2** — SFC9120 (Huntington/EF10), dual-port 10GbE PCIe 3.0.
- Vendor/device: `0x1924` / `0x0903` (PF), `0x1924` / `0x1903` (VF)
- Two PFs: PF0 = port 0, PF1 = port 1; SR-IOV disabled; `vf=0xffff` confirms PF
- MC firmware: `1001.7.2.6`; `func_flags`: PF0=`0x7` (PRIMARY+LINKCTRL+TRUSTED), PF1=`0x6`
- Data path: PF0 and PF1 cabled to each other via front-faceplate SFP+ DAC (port-to-port loopback)

### BAR layout (PF0; PF1 is identical type/size, shifted)

| RID | Offset | Type | Size | PF0 base | PF1 base |
|---|---|---|---|---|---|
| `PCIR_BAR(0)` | `0x10` | I/O | 256 B | `0x3100` | `0x3000` |
| `PCIR_BAR(2)` | `0x18` | mem64 | 8 MB | `0x60000000` | `0x60800000` |
| `PCIR_BAR(4)` | `0x20` | mem64 | 16 KB | `0x61000000` | `0x61004000` |

`PCIR_BAR(2)` = function MMIO / MCDI doorbell window (`EFX_MEM_BAR_HUNTINGTON_PF=2`).
`PCIR_BAR(4)` = MSI-X table+PBA (Table at `0x20[0x0]`, PBA at `0x20[0x2000]`). 32 MSI-X messages.

---

## Solarflare Quirks — Do Not Violate

- **DO NOT FLR.** `pcie_flr()` gates the BIU clock indefinitely — `HW_REV_ID` and the full
  8 MB BAR return all-zeros. Call site in `sfc7120_fbsd_attach` is `#if 0`'d. Use
  `MC_CMD_ENTITY_RESET (0x20)` with `FUNCTION_RESOURCE_RESET` instead — releases lingering
  VIs/queues/filters from prior loads, making reload clean.

- **Opcodes ≥ 0x80 require v2 framing.** v1 truncates cmd to 7 bits silently; `0x8b ALLOC_VIS`
  becomes `0x0b` and returns plausible-but-wrong errnos. `sfc7120_mcdi_send_locked` already picks
  v1 vs v2; do not "optimize" out the v2 path. Symptom: errno that doesn't match the command sent.

- **DO NOT add an alignment check to `sfc7120_mcdi_exec`.** MCDI lengths are byte counts, not
  dword counts. `MC_CMD_VADAPTOR_ALLOC_IN_LEN=30` is not dword-aligned — a pre-flight
  `(in_len & 3) != 0 → EINVAL` rejects it **before the wire** with no MC log line.

- **`MC_DB_LWRD` (0x200) is write-only.** Use `HW_REV_ID` (0x000) for MMIO liveness checks.

- **PF1 MAC is base+1.** `pciconf` reports PF0's base MAC; PF1 = base+1 (`00:0f:53:28:54:91`).

- **Stock `sfxge` conflicts.** `kldunload sfxge` before `kldload sfc7120pol`.

- **Low-latency firmware.** `GET_CAPABILITIES` reports `rxdp_fw_id=0x0001` (`RXDP_LOW_LATENCY`).
  Queue init flags (`EVQ 0x39`, `RXQ 0x300`, `TXQ 0x06`) accepted as-is. TSO/RSS extensions
  gated behind `FULL_FEATURED` — keep in mind for optional offloads.

- **`mac_fault=XGMII_LOCAL` with empty cage is expected**, not a driver bug. Trust `LINK_UP`
  flag, not `LINK_SPEED`, for liveness.

- **`GET_LINK fcntl=2` (BIDIR) is the real negotiated value** with both ends advertising
  `PAUSE|ASYM`; it is not a pre-negotiation default.

---

## MCDI Implementation

Lives in `sfc7120_mcdi.c` / `sfc7120_mcdi.h`. Reference catalogue in `ref_efx_regs_mcdi.h`
(verbatim from `~/cheri/cheribsd/sys/dev/sfxge/common/efx_regs_mcdi.h`).

### Wire format

256-byte DMA-coherent mailbox, 256-byte aligned (sfxge bug24769 — doorbell recovery requires
low byte of mailbox physaddr = 0). MC reads request, writes response, sets RESPONSE bit.

**v1 framing** (opcodes 0x00–0x7E, payload ≤ 252 B) — single 32-bit header at offset 0:
```
bits[ 6:0]  command (7-bit)       bits[15:8]  payload length (bytes)
bits[19:16] sequence number       bit  [22]   error (response only)
bit  [21]   NOT_EPOCH             bit  [23]   response (set by MC)
```
Payload at offset 4.

**v2 escape** (opcodes ≥ 0x7F or payload > 252 B):
```
offset 0: v1 header with cmd=0x7F (MC_CMD_V2_EXTN), datalen=0
offset 4: v2 ext — bits[14:0] real cmd, bits[25:16] real length
offset 8: payload
```
Response mirrors request framing: v2 response has real length in ext header at offset 4.

### Sequence numbers and epochs

`sc->mcdi_seq` is a 4-bit counter (mod 16), incremented after every exec.
`sc->mcdi_new_epoch` clears `NOT_EPOCH` on the first command of a fresh transport.
After MC reboot: reset both (`mcdi_seq=0`, `mcdi_new_epoch=true`).

### Helpers

| Function | Purpose |
|---|---|
| `sfc7120_mcdi_init` / `_fini` | Alloc/free mailbox, init epoch state, prime doorbell |
| `sfc7120_mcdi_exec` | Synchronous exec: 10 s timeout, seq/epoch, v1/v2 routing |
| `sfc7120_mcdi_log_mc_state` | Log `HW_REV_ID` + `MC_SFT_STATUS`; catch MC reboots |
| `sfc7120_mcdi_clear_assertions` | `MC_CMD_GET_ASSERTS (0x06)` CLEAR=1 |
| `sfc7120_mcdi_reboot_after_assertion` | `MC_CMD_REBOOT (0x3D)` — defined, not called |
| `sfc7120_mcdi_entity_reset` | `MC_CMD_ENTITY_RESET (0x20)` — clean re-attach without FLR |
| `sfc7120_mcdi_get_version` | `MC_CMD_GET_VERSION (0x08)` — logs `1001.7.2.6` |
| `sfc7120_mcdi_drv_attach` / `_drv_detach` | `MC_CMD_DRV_ATTACH (0x1C)` FW_DONT_CARE |
| `sfc7120_mcdi_get_mac` | `MC_CMD_GET_MAC_ADDRESSES (0x55)` |
| `sfc7120_mcdi_dump_func_info` | `GET_PORT_ASSIGNMENT` + `GET_FUNCTION_INFO` + `GET_CAPABILITIES`; caches FLAGS1 |
| `sfc7120_mcdi_alloc_vis` / `_free_vis` | `MC_CMD_ALLOC_VIS (0x8B)` / `FREE_VIS (0x8C)` — 32 VIs, base 1 (PF0) / 1024 (PF1) |
| `sfc7120_mcdi_vadaptor_alloc` / `_free` | `VADAPTOR_ALLOC (0x98)` on `EVB_PORT_ID_ASSIGNED`, cap-gated `PERMIT_SET_MAC_*`. `IN_LEN=30` — do not alignment-check |
| `sfc7120_mcdi_filter_insert` / `_remove` | `FILTER_OP (0x8a)` — exact-DST_MAC → RXQ 0. MAC bytes in **network byte order**. Handle in `sc->rx_filter_handle` |
| `sfc7120_mcdi_init_evq` / `_fini_evq` | `INIT_EVQ (0x80)` — EVQ 0, 512 entries, flags `0x39` |
| `sfc7120_mcdi_init_rxq` / `_fini_rxq` | `INIT_RXQ (0x81)` — RXQ 0 → EVQ 0, 512 descs, flags `0x300` |
| `sfc7120_mcdi_init_txq` / `_fini_txq` | `INIT_TXQ (0x82)` — TXQ 0 → EVQ 0, 512 descs, flags `0x06` |

### Error code translation (`sfc7120_mcdi_xlate_err`)

| MC err | errno | | MC err | errno |
|---|---|---|---|---|
| 0 | 0 | | 12 ENOMEM | ENOMEM |
| 1 EPERM | EPERM | | 13 EACCES | EACCES |
| 2 ENOENT | ENOENT | | 16 EBUSY | EBUSY |
| 4 EINTR | EINTR | | 22 EINVAL | EINVAL |
| 5 EIO | EIO | | 38 ENOSYS | ENOSYS |
| 11 EAGAIN | EAGAIN | | 62 ETIMEDOUT | ETIMEDOUT |
| 95 EOPNOTSUPP | EOPNOTSUPP | | else | EIO |

### `sfc7120_hw_init` order

```
mcdi_init → get_version → drv_attach → clear_assertions → entity_reset
→ get_mac → dump_func_info → alloc_vis(1,32) → vadaptor_alloc
→ get_phy_cfg → set_mac → set_link → get_link (single seed)
```

`clear_assertions` after `drv_attach`; `entity_reset` after that — needs TRUSTED privilege
gained via `DRV_ATTACH`. After `hw_init`: `intr_setup` (MSI-X) → `alloc_dma_resources`
→ `init_evq(0)` → `init_rxq(0)` → `init_txq(0)` → `filter_insert`. Teardown reverses:
`filter_remove` → `fini_txq` → `fini_rxq` → `fini_evq`.

`intr_setup` runs **before** `INIT_EVQ` so the vector exists when the queue arms; a spurious
early interrupt is harmless (ISR bails on `!evq_initialized`).

---

## Role in the Larger System

Userspace (userlib/sfc7120_user.c — IN PROGRESS)
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

Port 0 (PF0) and port 1 (PF1) are cabled via SFP+ DAC — frames TX'd on one port
ingress the other. Both PFs must be open simultaneously; userspace bridges between them.

---

## File Map

| File | Purpose |
|---|---|
| `sfc7120.c` | PCI probe/attach/detach, IOCTL dispatch, `hw_init` orchestration |
| `sfc7120.h` | Softc, region enum, IOCTL request structs |
| `sfc7120_mcdi.c` | MCDI transport: mailbox, v1/v2 framing, all `MC_CMD_*` helpers |
| `sfc7120_mcdi.h` | Public MCDI prototypes |
| `sfc7120_mmio.h` | Register offsets, `SFC7120_READ/WRITE_REG` macros |
| `sfc7120_mmio.c` | `sfc7120_dump_regs()` diagnostic |
| `sfc7120_tables.c` | Slice manifest (`sfc7120_reg_slices[]`) |
| `capio.c` / `capio.h` | Local CAPIO framework copy (sync to `~/CheriBsdE1000/` on generic changes) |
| `ref_efx_regs_mcdi.h` | Verbatim sfxge MCDI catalogue — do not modify |
| `ref_sfxge_mcdi.c` | sfxge MCDI helper excerpt for cross-reference |

---

## The Mandatory CAPIO Softc Layout

`capio.c` casts `void *sc` to `capio_softc_header_t *` — layout is non-negotiable:

```c
typedef struct sfc7120_softc {
    device_t            dev;        /* MUST be first */
    capio_softc_t       capio_sc;   /* MUST be second */
    shared_mem_region_t smem[SFC7120_REGION_COUNT];
    /* hardware fields ... */
} sfc7120_softc_t;
```

---

## Memory Regions

| Index | Enum | `is_physical` | Size | Sliced? |
|---|---|---|---|---|
| 0 | `SFC7120_TX_BUFFER` | false | 128 KB (64×2048) | No |
| 1 | `SFC7120_RX_BUFFER` | false | 128 KB (64×2048) | No |
| 2 | `SFC7120_MMIO_REGION` | true | BAR2 size (8 MB) | Yes |

Slice manifest offsets are **placeholders** — cross-check against EF10 register docs / sfxge
before exposing to userspace. Wrong offsets → CHERI bounds violation in userspace, not a panic.

---

## Implementation Status

| Component | State | Notes |
|---|---|---|
| PCI probe | Done | vendor `0x1924`, devices `0x0903`/`0x1903` |
| Attach / detach ordering | Done | Full ordering confirmed in dmesg |
| `capio_ops_t` vtable | Done | `ioctl`, `get_buffer_size`, `is_dying` |
| Slice manifest | Skeleton | 7 entries; extend for multi-queue |
| Cdev `/dev/sfc7120pol` | Done | open/close/poll/ioctl |
| MCDI transport (v1+v2) | Done | Do not break v2 framing — see quirks |
| MCDI identity bringup | Done | `GET_VERSION` → `DRV_ATTACH` → `ENTITY_RESET` → `GET_MAC` → func info → `ALLOC_VIS` |
| vAdaptor | Done | `VADAPTOR_ALLOC` on `EVB_PORT_ID_ASSIGNED`; `IN_LEN=30` not dword-aligned |
| DMA resources | Done | EVQ/RX/TX rings (1 page each), 1 MB RX + 1 MB TX packet buffers, BUS_DMA_COHERENT |
| EVQ / RXQ / TXQ init | Done | 512 entries each, EVQ flags `0x39`, RXQ `0x300`, TXQ `0x06` |
| RX filter | Done | Exact-DST_MAC → RXQ 0; handle in `sc->rx_filter_handle`; confirmed on hardware |
| MAC + link bring-up | Done | `GET_PHY_CFG` / `SET_MAC` (v1, `IN_LEN=28`) / `SET_LINK` / seed `GET_LINK` |
| MSI-X + ISR | Done | `pci_alloc_msix(1)`, `bus_setup_intr(sfc7120_interrupt_handler)` |
| EVQ event delivery | **Unverified** | ISR registered; no `EVQ ...` dmesg line yet — no traffic, link was already up at attach |
| TX-completion reaping | **TODO** | Inside ISR |
| TX/RX IOCTLs | **TODO** | Return `ENOSYS` today |
| `SFC7120_GET_MAC` | Done | Returns `sc->mac_addr` |
| In-tree copy | Not yet | `~/cheri/cheribsd/sys/modules/sfc7120pol/` |
| Userspace driver | Not yet | `~/E1000Lwip/netif/sfc7120_driver.c` |


## Next Steps

1. **Exercise EVQ event delivery** — reseat the loopback cable to force a LINKCHANGE event;
   confirm `EVQ LINKCHANGE: link=UP` appears in dmesg.
2. **TX/RX IOCTLs** — kernel-mediated fallback for testing before userspace driver.
3. **Userspace driver** — `~/E1000Lwip/netif/sfc7120_driver.c` as a lwIP `netif`
   (mirror `mlx5_driver.c`).
4. **In-tree copy** — `~/cheri/cheribsd/sys/modules/sfc7120pol/`; sync `capio.c`/`.h`.

---

## Build

```bash
# Cross-compile (Linux → Morello):
./build.sh build
mv sfc7120pol.ko ~/mod_out/

# Native (CheriBSD):
./build.sh build && sudo ./build.sh install

# Compilation database:
./build.sh bear
```

Dependencies: `modmap.ko` must load first. `kldunload sfxge` if loaded.

---

## Devices Supported

| Vendor | Device ID | Description |
|---|---|---|
<<<<<<< HEAD
| 0x1924 | 0x0903 | Solarflare SFC9120 10G (EF10) PF |
| 0x1924 | 0x1903 | Solarflare SFC9120 10G (EF10) VF |
