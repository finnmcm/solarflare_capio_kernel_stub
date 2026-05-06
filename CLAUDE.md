# CheriBsdSolarflare7120 — Solarflare 7120 CAPIO Kernel Driver Stub

## What This Directory Is

Out-of-tree workspace for the Solarflare SFN7000-series (Huntington / EF10)
CAPIO kernel driver stub. Modeled on `~/CheriBsdE1000/`: cross-compile via
bmake on Linux, native make on CheriBSD, output `sfc7120pol.ko`.

**Status: SKELETON.** The PCI driver shell, CAPIO wiring, IOCTL dispatch,
and slice manifest exist. The hardware bringup (MCDI handshake, EVQ/RXQ/TXQ
init, MAC config, link config) is intentionally stubbed with TODO blocks.
This directory is the iteration workspace; an in-tree copy at
`~/cheri/cheribsd/sys/modules/sfc7120pol/` should follow once the driver
stabilizes (matching the e1000 / mlx5pol layout).

There is currently no in-tree counterpart and no userspace driver. Add
both as the work progresses.

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
| `sfc7120.c` | PCI driver: probe/attach/detach, vtable, IOCTL dispatch |
| `sfc7120.h` | Softc, region enum (`SFC7120_TX_BUFFER` / `_RX_BUFFER` / `_MMIO_REGION`), IOCTL request structs |
| `sfc7120_mmio.h` | BAR0 register offsets (placeholders — confirm against EF10 docs), `SFC7120_READ_REG` / `SFC7120_WRITE_REG` |
| `sfc7120_mmio.c` | `sfc7120_dump_regs()` — diagnostic register dump |
| `sfc7120_tables.c` | Slice manifest (`sfc7120_reg_slices[]`) for the MMIO region |
| `capio.c` | Local copy of CAPIO framework (mirrors `~/CheriBsdE1000/capio.c`) |
| `capio.h` | CAPIO types: `capio_softc_t`, `slice_def_t`, etc. |
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
| PCI probe | Partial | Skeleton table with vendor 0x1924 + device IDs 0x0903 (PF) / 0x1903 (VF). Confirm with `pciconf -lv`. |
| Attach ordering | Done | BAR alloc → hw_init → DMA alloc → make_dev_capio → smem populate → init_capio_sc → vm_object_handle alloc |
| Detach ordering | Done | dying flag → free handles → modmap_unregister → destroy_dev → free DMA → hw_teardown → release BAR → capio_destroy → destroy locks |
| `capio_ops_t` vtable | Done | `ioctl`, `get_buffer_size`, `is_dying` all wired |
| Slice manifest | Skeleton | 7 entries; extend for multi-queue |
| Cdev (`/dev/sfc7120pol`) | Done | open/close/poll/ioctl wired |
| MCDI / firmware bringup | **TODO** | `sfc7120_hw_init` / `sfc7120_hw_teardown` are stubs |
| DMA allocation | **TODO** | `sfc7120_alloc_dma_resources` is a stub; mirror e1000_alloc_*_dma |
| EVQ / TXQ / RXQ init | **TODO** | Triggered through MCDI commands |
| Interrupt handler | **TODO** | Skeleton exists; needs to walk EVQ |
| TX/RX IOCTLs (`SFC7120_TX`, `SFC7120_RX`) | **TODO** | Return ENOSYS today |
| `SFC7120_GET_MAC` | Done shape | Returns `sc->mac_addr`; that field is unpopulated until MCDI lands |
| In-tree copy | Not yet | Add to `~/cheri/cheribsd/sys/modules/sfc7120pol/` once stable |
| Userspace counterpart | Not yet | Add to `~/E1000Lwip/netif/sfc7120_driver.c` |

---

## Recommended Implementation Order

1. **`sfc7120_alloc_dma_resources` / `sfc7120_free_dma_resources`** — start
   with a `mlx5_dmabuf_t`-style helper (single tag/map/load wrapper). EF10
   needs at minimum: EVQ ring, TX desc ring, RX desc ring, TX packet
   buffer, RX packet buffer, MCDI message buffer. That's ~6 buffers — too
   many for the e1000 per-buffer pattern.

2. **MCDI plumbing** — synchronous request/response over a DMA-resident
   message buffer + the MC doorbell. EF10's MCDI is structurally similar
   to mlx5pol's command queue: write request to mailbox, kick doorbell,
   poll for completion, read response. Use `~/cheri/cheribsd/sys/dev/sfxge/`
   as the reference.

3. **MCDI bringup commands** — at minimum: `MC_CMD_GET_VERSION`,
   `MC_CMD_GET_BOARD_CFG` (yields the MAC), `MC_CMD_INIT_EVQ`,
   `MC_CMD_INIT_RXQ`, `MC_CMD_INIT_TXQ`, `MC_CMD_MAC_RECONFIGURE`.
   Populate `sc->mac_addr` from `GET_BOARD_CFG`.

4. **Interrupt handler** — MSI-X allocation, EVQ event walking.

5. **TX/RX IOCTLs** — kernel-mediated fallback paths for testing before
   the userspace driver lands.

6. **Userspace driver** — `~/E1000Lwip/netif/sfc7120_driver.c`, registers
   as a lwIP `netif`. Mirror `~/E1000Lwip/netif/mlx5_driver.c`.

7. **In-tree copy** — once stable, copy under
   `~/cheri/cheribsd/sys/modules/sfc7120pol/` so it builds into the
   CheriBSD image. Use the mlx5pol-style 7-line Makefile with VPATH
   pointing at `../em_pol_test` for `capio.c`.

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

- All MCDI / hardware bringup is stubbed.
- Slice manifest offsets are placeholder values from EF10 documentation
  conventions; cross-check against `~/cheri/cheribsd/sys/dev/sfxge/`.
- No in-tree copy.
- No userspace driver.
- `vm_object_handle_t` allocations are not freed in detach? They are —
  the loop is in `sfc7120_fbsd_detach` before `destroy_dev`. Mention this
  because the e1000 out-of-tree copy is missing that loop.
