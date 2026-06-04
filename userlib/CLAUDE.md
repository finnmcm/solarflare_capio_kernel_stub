# CAPIO ef_vi — Userspace Kernel-Bypass Library for the Solarflare 7120

## What This Directory Is

`userlib/` is the **userspace half** of the Solarflare SFN7322F-R2 (SFC9120 /
EF10 "Huntington") CAPIO stack. It is the process-side counterpart to the
kernel module one directory up (`../sfc7120.c` et al.).

The end goal of this directory is a **CAPIO implementation of AMD's ef_vi** —
the lowest-level Solarflare userspace networking API. In ef_vi, a process's
**VI** (Virtual Interface = an event queue + a TX descriptor ring + an RX
descriptor ring + a per-VI doorbell window) is mapped directly into the
process's address space. The process posts descriptors straight into the rings
and rings the NIC's doorbells itself by writing MMIO. **There is no kernel in
the data path** — after setup, sending and receiving a packet involves zero
syscalls.

The twist that makes this a research artifact: we build it as a **CAPIO**
driver, so every region the process maps is a **bounded, sealed CHERI
capability**. Hardware — not a software check, not the kernel — enforces that
this process can only touch *its own* doorbells, *its own* descriptor rings, and
*its own* packet buffers. A stray pointer or a malicious descriptor address
traps in hardware instead of corrupting the NIC, another VI, or kernel state.

```
   ef_vi (AMD)                          CAPIO ef_vi (this directory)
   ──────────                           ────────────────────────────
   VI mmap'd into process               VI mmap'd as CHERI capabilities
   raw BAR doorbell page                sliced MMIO region (per-doorbell caps)
   pinned DMA buffers                   bounded packet-buffer caps + kernel PA
   poll EVQ, post desc, no kernel       same — but every access is cap-checked
```

### Relationship to the rest of the tree

- **AMD `ef_vi`** — the API and operational model this directory targets: a VI
  mapped into the process, descriptors posted into rings, doorbells rung from
  userspace, EVQ polled, no kernel in the data path. We follow ef_vi's *shape*
  (the `ef_vi_transmit` / `ef_vi_receive_post` / `ef_eventq_poll` operational
  model and its kernel-resource-manager-hands-userspace-the-VI design), reframed
  onto our `sfc7120_*` API and CAPIO capability model.
- **sfxge** (`~/cheri/cheribsd/sys/dev/sfxge/common/ef10_{tx,rx,ev}.c`,
  `efx_regs_ef10.h`) — the EF10 wire-format reference: descriptor layouts, event
  decode, doorbell math, VI window addressing. This is the authority for *what
  the EF10 hardware expects*.
- **`../sfc7120.c` / `../sfc7120_mcdi.c`** — the kernel module (itself modeled on
  sfxge). It does all device bring-up (PCI attach, MCDI firmware handshake,
  VI/queue allocation, MAC/link), owns **EVQ 0** (control: link/MCDI/error
  events, interrupting), and exposes the data-path resources to us via
  `/dev/sfc7120pol{0,1}` + CAPIO. Its ioctl handlers are the **verified oracle**
  for this exact card (below).
- **`~/E1000Lwip/netif/mlx5_driver.c`** — a ConnectX-4 driver, *different NIC
  family*, but the **only pre-existing CAPIO userspace driver**. Use it solely
  as a structural precedent for the CAPIO plumbing that is NIC-agnostic — region
  mmap via CHERI caps, sliced-doorbell writes, the `dsb sy` discipline, and the
  kernel-provides-physical-address (PA-mode) handoff. Do **not** copy its WQE/CQE
  semantics; the EF10 behavioral model comes from ef_vi + sfxge + the oracle.

---

## Goals

1. **Kernel out of the data path.** After attach + mmap, `sfc7120_tx` /
   `sfc7120_rx` / `sfc7120_poll` issue **no syscalls**. They write descriptors
   into mapped rings and ring mapped doorbells directly.
2. **CHERI-bounded safety on every region.** Doorbells are exposed as a *sliced*
   MMIO region — one bounded capability per register, so the process literally
   cannot form a pointer to a doorbell it doesn't own. Descriptor rings and
   packet buffers are bounded caps. The security story is "the hardware enforces
   the driver's blast radius," and we will have tests that prove a wrong address
   traps rather than escapes.
3. **Low latency.** This is the reason ef_vi exists. Favor the TX inline-push
   doorbell, batch RX re-posting, minimize barriers to the one `dsb sy` the
   hardware actually requires, and lay state out cache-line-friendly. The
   benchmark target is "beat the ioctl path's per-packet RTT, and approach the
   mlx5 CAPIO numbers in spirit."
4. **A small, faithful ef_vi model.** Not the full ef_vi API surface — one VI,
   one TXQ, one RXQ, one data EVQ. Enough to demonstrate the architecture and
   measure it. Multi-VI / RSS is explicitly out of scope for now.

---

## Background — ef_vi in One Page

A **VI** is the unit of NIC access. It bundles:

- **EVQ (event queue)** — a DMA ring the NIC writes 8-byte completion events
  into (RX arrived, TX done). The process *polls* this ring; no interrupt.
- **TXQ (TX descriptor ring)** — a DMA ring the process writes 8-byte transmit
  descriptors into. Each descriptor points at a packet buffer by **bus
  address**.
- **RXQ (RX descriptor ring)** — same, for receive: the process pre-posts
  descriptors pointing at empty buffers; the NIC fills them and posts an event.
- **Doorbell window** — a small MMIO region (one 8 KB page per VI on EF10). The
  process writes a new producer/read pointer here to tell the NIC "I added
  descriptors" or "I consumed events."

The transmit loop, with no kernel involved, is: *copy packet into a TX buffer
slot → write a descriptor naming that slot's bus address → memory barrier →
write the TX doorbell → later, poll the EVQ for the TX-done event.* Receive is
the mirror image.

**How CAPIO maps each piece:** the kernel allocates all of these (DMA-coherent
rings + buffers, plus the function's MMIO BAR) and hands the process bounded
CHERI capabilities to them via `mmap` through `/dev/modmap`. The doorbell window
is handed over as a **sliced** region: instead of one fat capability to the
whole BAR, the process gets a small array of capabilities, one per allowed
register, each bounded to exactly that register.

---

## Current State (as of 2026-06-03)

**Phase 1 (ioctl path) works; Phase A (kernel ring exposure) is done.** The
kernel now exposes the TX/RX descriptor rings and the data EVQ ring as CAPIO
regions and answers `SFC7120_GET_VI_INFO` (see the Phase A entry below) —
hardware-verified. Userspace does not yet *use* these (that is Phase C+);
`test.c` still drives traffic through the ioctl path.

**Phase 1 (ioctl path) works.** `test.c` (`sfctest`) sends frames PF0→PF1 over a
DAC cable and receives them, today. That path is:

- `sfc7120_init` (`sfc7120_user.c:35`): `open("/dev/sfc7120pol{0,1}")` →
  `CAPIO_ATTACH` (seals a malloc'd token page) → mmap the TX and RX **packet
  buffers** via `/dev/modmap` → `SFC7120_GET_MAC`.
- `sfc7120_tx` / `sfc7120_rx` (`sfc7120_user.c:107`, `:132`): package the buffer
  into a `sfc7120_tx_req_t` / `sfc7120_rx_req_t` and call the **`SFC7120_TX` /
  `SFC7120_RX` ioctls**. The *kernel* copies the packet, writes the descriptor,
  rings the doorbell, and polls the data EVQ.

So today the process maps only the two packet buffers and the (sliced) MMIO
region, and the kernel still does every per-packet operation. The descriptor
rings and the data EVQ ring are now **mappable** (Phase A added the regions +
`GET_VI_INFO`), but the userspace library does not yet map or use them —
closing that gap is the whole roadmap below.

On the kernel side, the relevant facts are already in place: two EVQs exist
(**EVQ 0** control/interrupting, owned by the kernel ISR; **EVQ 1** data/
non-interrupting, carries TX/RX completions), the descriptor rings and EVQ ring
are allocated DMA-coherent, and the buffer physical addresses + VI base are
cached in the softc. See `../CLAUDE.md` for the kernel status matrix.

### The single most important principle for this directory

> **The kernel's ioctl handlers are the verified oracle.** `SFC7120_TX`
> (`../sfc7120.c` ~`:911-1037`) and `SFC7120_RX` (~`:1038-1153`) already contain
> the **working** descriptor formats, event decode, doorbell offsets, and
> RX-prefix handling for *this exact card and its low-latency firmware variant*.
> When building the userspace data path, **port that logic verbatim across the
> mmap boundary.** Do not re-derive it from the datasheet or from sfxge — those
> describe the general EF10 family; the kernel handler describes *this NIC, as
> observed working.* The only genuinely new code is the mmap plumbing and the
> CHERI capability handling. Everything about *what bytes the hardware wants* is
> already solved one directory up.

This is not a style preference; it is the risk-management strategy. Each phase
below is validated **against the ioctl path as a live oracle** before the ioctl
path is retired.

---

## Architecture — Verified Hardware Facts

These are recorded here so future sessions do not re-research them. Cross-checked
against the working kernel handlers, sfxge `common/ef10_*.c` +
`efx_regs_ef10.h`, and the EF10 register layout. **If any of these ever seem
wrong, trust the kernel handler over this table and fix the table.**

### Queue geometry (`../sfc7120_uapi.h`)

| Constant | Value |
|---|---|
| `SFC7120_NUM_TX_DESC` / `_RX_DESC` / `_EVQ_ENTRY` | 512 each |
| `SFC7120_TX_BUFFER_SIZE` / `_RX_BUFFER_SIZE` | 2048 B per slot |
| `SFC7120_TX_DESC_SIZE` / `_RX_DESC_SIZE` / `_EVQ_ENTRY_SIZE` | 8 B each |
| TX/RX packet buffer total | 512 × 2048 = 1 MB each |
| Each desc ring / EVQ ring total | 512 × 8 = 4 KB each |

### TX descriptor — 8 bytes (`ESF_DZ_TX_KER_*`)

| Field | Bits | Notes |
|---|---|---|
| `TYPE` | 63 | 0 = kernel descriptor |
| `CONT` | 62 | 0 = end-of-packet, 1 = more fragments |
| `BYTE_CNT` | 47:34 | packet length |
| `BUF_ADDR_DW1` | 63:48 | bus address high |
| `BUF_ADDR_DW0` | 31:0 | bus address low |

### RX descriptor — 8 bytes (`ESF_DZ_RX_KER_*`)

| Field | Bits | Notes |
|---|---|---|
| `BYTE_CNT` | 61:48 | buffer size posted (2048) |
| `BUF_ADDR_DW1` | 47:32 | bus address high |
| `BUF_ADDR_DW0` | 31:0 | bus address low |

### Event — 8 bytes

| Field | Bits | Notes |
|---|---|---|
| `EV_CODE` | top nibble (63:60) | `RX_EV`=0, `TX_EV`=2, `DRIVER_EV`=5, `MCDI_EV`=12 |
| RX `RX_BYTES` | 13:0 | received length (RX_EV) |
| TX `TX_DESCR_INDX` | 15:0 | completed descriptor index (TX_EV) |

**Empty-slot detection: there is no phase bit.** The EVQ ring is initialized to
all-`0xFF`; the NIC overwrites a slot with a real event (EV_CODE ∈ {0,2,5,12}).
A slot whose EV_CODE nibble is `0xF` is empty. Poll = read at `read_ptr`, check
the nibble, and stop when it's `0xF`.

### Doorbells — per-VI window

- VI window shift = **13** (8 KB stride). A VI's doorbell base in the BAR is
  `vi_instance << 13`. Per-window register offsets:
  - `EVQ_RPTR` (ack events): `0x400`
  - `RX_DESC_UPD` (push RX producer): `0x830`
  - `TX_DESC_UPD` (push TX): `0xa10`; **`+8` = WPTR-only push** (`0xa18`)
- **Therefore the data EVQ (instance 1) RPTR doorbell is at
  `(1<<13)+0x400 = 0x2400`, NOT `0x400`.** The RXQ/TXQ are instance 0, so their
  doorbells sit at the base offsets. *Always confirm the instance numbers against
  what the kernel handlers actually write* — see `../sfc7120_mmio.h` for the
  current `SFC7120_REG_*` values and the kernel TX/RX handlers for the live
  offsets.
- **RX write pointer must be 8-descriptor aligned** (`EF10_RX_WPTR_ALIGN`) when
  ringing `RX_DESC_UPD`. Post in batches of 8.

### RX packet prefix

The low-latency firmware on this card **prepends a metadata prefix** to received
packets in the buffer. The kernel RX handler strips a **14-byte prefix** and
reads the packet length from **prefix offset +8** (LE uint16). **Mirror the
kernel handler exactly** — do *not* assume the sfxge "standard datapath has no
prefix" default, which does not match this firmware variant.

### DMA addressing

Descriptors carry **bus (physical) addresses**, not virtual ones — the EF10
PA-mode pattern (sfxge `ef10_{tx,rx}_qpost` build the descriptor `BUF_ADDR` from
DMA addresses). This is the CAPIO analog of ef_vi's `ef_memreg` model, where
registering a buffer hands userspace a NIC-visible address: here the kernel
pre-allocates the DMA-coherent buffers and hands back their bus addresses. The
address for slot *n* is `*_buffer_paddr + n * 2048`. The process learns
`tx_buffer_paddr` / `rx_buffer_paddr` from the kernel at attach (see the planned
`SFC7120_GET_VI_INFO` ioctl); it never computes or owns physical memory itself.

### Memory barrier

On ARM64 (Morello), issue **`dsb sy`** after writing a descriptor / updating a
ring index and **before** writing the doorbell, so the descriptor write is
visible to the NIC's DMA engine before the doorbell kick. mlx5_driver.c does this
on every post.

---

## Kernel / Userspace Interface

### Today (Phase 1)

| Region (`sfc7120_vm_map_type_t`) | Mapped by userspace? | Backing |
|---|---|---|
| `SFC7120_TX_BUFFER` | yes (`map_buffer`) | 1 MB DMA packet buffer |
| `SFC7120_RX_BUFFER` | yes (`map_buffer`) | 1 MB DMA packet buffer |
| `SFC7120_MMIO_REGION` | yes (sliced) | BAR2 (8 MB), bounded per-register |

IOCTLs: `CAPIO_ATTACH` / `CAPIO_GOODBYE` (capio.h), `SFC7120_TX` / `SFC7120_RX` /
`SFC7120_GET_MAC` (`../sfc7120_uapi.h`).

Slice manifest (`../sfc7120_tables.c`, corrected in Phase B): `MC_DOORBELL`
(`0x200`), `DATA_EVQ_RPTR_DBL` (`0x2400` — instance 1's window),
`RX_DESC_DBL` (`0x830`), `TX_DESC_DBL` (`0xa10`, 12 B), `HW_REV_ID` (RO).
The control EVQ 0 RPTR (`0x400`) is deliberately not exposed — kernel-owned.

### What must be added (Phases A–B)

- Three new regions: `SFC7120_TX_DESC_RING`, `SFC7120_RX_DESC_RING`,
  `SFC7120_EVQ_RING` (the data EVQ, instance 1) — `is_physical=false`, backed by
  the already-allocated `sc->tx_desc_ring` / `rx_desc_ring` / `data_evq_ring`.
- A `SFC7120_GET_VI_INFO` ioctl that follows ef_vi's resource-manager model —
  the kernel hands userspace everything it needs to drive the VI itself:
  `tx_buffer_paddr`, `rx_buffer_paddr` (the ef_memreg-style NIC-visible buffer
  addresses), `vi_base`, the EVQ/RXQ/TXQ instance numbers, ring/desc counts, and
  the initial head pointers.
- Corrected doorbell slices for the *actual* data-path VI instances.

---

## Implementation Roadmap

Each phase ends with a concrete **verify** step, and every data-path phase is
checked against the ioctl path as a live oracle before moving on. Per project
guidance, **no phase is marked "done" here until it has been loaded and verified
on the Morello board** — a clean cross-compile is necessary but not sufficient.

### ✅ Phase A — Kernel: expose the data-path rings + DMA bases (done 2026-06-03)
Added three ring regions to the enum (`../sfc7120_uapi.h`):
`SFC7120_TX_DESC_RING`, `SFC7120_RX_DESC_RING`, `SFC7120_EVQ_RING` (the data
EVQ = `sc->data_evq_ring`, instance 1). Populated `sc->smem[]` for them in
attach (`../sfc7120.c`, replacing the old "EVQ ring intentionally NOT
registered" comment), all `is_physical=false`/`is_sliced=false`, 4 KB each;
added matching `sfc7120_get_buffer_size` cases. Added the `SFC7120_GET_VI_INFO`
ioctl (`_IOWR('S', 4, ...)`) — mirrors `GET_MAC`, returns `tx/rx_buffer_paddr`
(ef_memreg-style PAs), `vi_base`, instance numbers (EVQ=1, RXQ=0, TXQ=0), ring
counts (512 each), and current head pointers.
**Verified on hardware (2026-06-03):** both PFs still attach with the larger
`smem[]`; old `sfctest` ioctl path still passes; the three new regions mmap at
4 KB; `GET_VI_INFO` returns sane PAs and `vi_base`.

### ✅ Phase B — Kernel: correct the slice manifest for the data-path VIs (done 2026-06-03)
Per-VI window math + `SFC7120_REG_DATA_*` offsets added to
`../sfc7120_mmio.h`; manifest now exposes `DATA_EVQ_RPTR_DBL` at `0x2400`
and drops the kernel-owned control EVQ 0 RPTR (`0x400`). Bonus: TX/RX ioctl
handlers now ack the data EVQ at `0x2400` (previously skipped).
**Verified on hardware (2026-06-03):** `sfctest tx`/`rx` passes with every
TX/RX ringing `0x2400`. Userspace slice verify (HW_REV_ID read,
out-of-slice trap) folded into Phase C.

### 🔜 Phase C — Userspace: direct EVQ polling (read path first)
Implement `sfc7120_poll` over the mapped data-EVQ ring: all-`0xFF` empty
detection, decode RX_EV/TX_EV, advance the read pointer, write the EVQ-1 RPTR
doorbell through the MMIO slice cap, `dsb sy`. Keep the kernel posting
descriptors for now.
**Verify:** poll reports the same events, in order, that the ioctl oracle
consumes. Also (carried from Phase B): read `HW_REV_ID` through its slice
cap, and confirm an out-of-slice doorbell write **traps** (CHERI bounds)
rather than corrupting the device.

### 🔜 Phase D — Userspace: direct RX (post + doorbell)
Pre-post RX descriptors into the mapped `rx_desc_ring`
(`BUF_ADDR = rx_buffer_paddr + slot*2048`, `BYTE_CNT = 2048`), ring
`RX_DESC_UPD` with an 8-aligned wptr. On RX_EV: read from the RX buffer slot
(strip the 14-byte prefix, length at +8, exactly as the kernel does), re-post the
descriptor, ring the doorbell.
**Verify:** `sfctest rx` receives PF0→PF1 frames with the direct RX path and the
kernel TX path.

### 🔜 Phase E — Userspace: direct TX (post + doorbell)
Build the 8-byte TX descriptor in the mapped `tx_desc_ring`
(`BUF_ADDR = tx_buffer_paddr + slot*2048`), copy the packet into the TX buffer
slot, `dsb sy`, ring `TX_DESC_UPD` (push or WPTR-only — port the kernel's exact
choice), then poll the EVQ for TX_EV. Port the kernel's descriptor build + push
verbatim.
**Verify:** `sfctest tx` transmits with the direct TX path; full direct RX+TX
dual-port test passes.

### 🔜 Phase F — Cutover
`sfc7120_tx` / `sfc7120_rx` become zero-syscall direct ops; `sfc7120_poll` is the
new public entry point for draining completions. Keep the ioctl path behind a
runtime/compile flag as the fallback oracle. Dual-port `sfctest` passes
end-to-end over the direct path; only then remove the ioctl data path.
**Verify:** zero syscalls in the TX/RX/poll hot loop (confirm with `truss` /
counters); dual-port test green.

### 🔜 Phase G — Low-latency tuning + CHERI security validation
TX inline-push path, batched 8-aligned RX re-posting, barrier minimization
(one `dsb sy` where required, not more), cache-line-friendly state layout.
Security tests: out-of-slice doorbell write traps; a descriptor cannot name a bus
address outside the packet-buffer capability; PF0/PF1 isolation holds. Benchmark
per-packet latency vs the ioctl path.
**Verify:** latency improvement measured and recorded; security tests demonstrate
hardware-enforced bounds.

---

## File Map

| File | Purpose |
|---|---|
| `sfc7120_user.c` | Userspace library: attach, region mmap, TX/RX (ioctl today; direct ef_vi path per roadmap) |
| `sfc7120_user.h` | Public API (`sfc7120_if_t`, `sfc7120_init/destroy/tx/rx`); device-node paths |
| `test.c` | `sfctest` — dual-port PF0→PF1 smoke test (producer/consumer) |
| `build.sh` | Cross-compile (Linux→Morello purecap) or native build of `sfctest` |
| `install.sh` | SCP sources to the board + native build |
| `sfctest` | Built purecap test binary (gitignored build product) |

### Reference files (read these before implementing a data-path phase)

| File | Why |
|---|---|
| `../sfc7120.c` `SFC7120_TX`/`SFC7120_RX` handlers | **The oracle** — verified descriptor/event/doorbell/prefix logic for this card |
| `../sfc7120_uapi.h` | Region enum, queue geometry constants, ioctl request structs |
| `../sfc7120_mmio.h` / `../sfc7120_tables.c` | Doorbell offsets + the slice manifest to extend |
| `../sfc7120.h` | Softc: ring VAs, `*_paddr`, `vi_base`, instance bookkeeping |
| **AMD ef_vi** (API model) | **The behavioral model** — VI operational shape, descriptor-post/doorbell/poll flow, ef_memreg-style buffer-address handoff |
| sfxge `~/cheri/.../sys/dev/sfxge/common/ef10_{tx,rx,ev}.c`, `efx_regs_ef10.h` | **EF10 wire-format authority** — descriptor/event layouts, doorbell math, VI window addressing (the oracle is authoritative for *this* card's quirks) |
| `~/E1000Lwip/netif/mlx5_driver.c` + `mlx5.h` | CAPIO *plumbing* precedent only (different NIC) — region mmap, sliced doorbells, `dsb sy`, PA-mode handoff. **Do not copy its WQE/CQE semantics.** |

---

## Gotchas

1. **Trust the kernel oracle, not the datasheet.** Every data-path detail
   (descriptor bits, event decode, doorbell offset, RX prefix) is already proven
   working in the kernel handler. Port it; don't re-derive it.
2. **RX prefix is 14 bytes on this firmware**, length at +8 — not the sfxge
   "no prefix" default. Strip it exactly as the kernel does.
3. **RX write pointer must be 8-descriptor aligned** when ringing `RX_DESC_UPD`.
   Post in batches of 8.
4. **`dsb sy` before every doorbell write.** Missing the barrier means the NIC
   may ring on a descriptor it can't yet see — intermittent, maddening.
5. **The data EVQ is instance 1**, so its RPTR doorbell is at `0x2400`, not
   `0x400`. Off-by-a-VI-window is a silent "events never get acked / queue backs
   up" failure. Confirm instance numbers against the kernel.
6. **A wrong doorbell offset becomes a userspace CHERI trap, not a kernel
   panic** — because the doorbells are sliced. Good for safety, but it means a
   bad offset shows up as a SIGPROT in `sfctest`, not a dmesg line. Check the
   slice bounds first when a direct write faults.
7. **Descriptors carry bus addresses** (`*_buffer_paddr + slot*2048`), supplied
   by the kernel via `GET_VI_INFO`. Userspace never owns physical memory; never
   put a virtual address in a descriptor.
8. **No phase bit on the EVQ** — empty = EV_CODE nibble `0xF` (ring init'd to
   all-`0xFF`). On wrap, the consumer must keep the read pointer coherent with
   the RPTR doorbell.

---

## Build & Deploy

```bash
# Cross-compile from Linux (default), or run on the board for a native build:
cd ~/CheriBsdSolarflare7120/userlib
sh ./build.sh build          # produces ./sfctest (purecap)

# Deploy sources + build natively on the Morello board:
sh ./install.sh

# On the board (kernel modules must be loaded first — see ../CLAUDE.md):
#   consumer:  ./sfctest rx     # opens /dev/sfc7120pol1 (PF1), blocks on RX
#   producer:  ./sfctest tx     # opens /dev/sfc7120pol0 (PF0), sends frames
```

Dependencies: `modmap.ko` + `sfc7120pol.ko` loaded on the board; the Morello
purecap SDK under `~/cheri/output/morello-sdk` for cross-compiling; `modmap.h` /
`capio.h` headers reachable (see `build.sh`'s `MODMAP_INC`).
