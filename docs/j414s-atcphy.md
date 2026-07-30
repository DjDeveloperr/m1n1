# j414s-atcphy: Apple Type-C PHY driver for T6020 (J414s / M2 Pro)

Branch: `feature/j414s-windows-unified`. Companion code:
[`src/atcphy_core.h`](../src/atcphy_core.h) /
[`src/atcphy_core.c`](../src/atcphy_core.c) (pure logic, host-tested),
[`src/atcphy.h`](../src/atcphy.h) / [`src/atcphy.c`](../src/atcphy.c) (m1n1
MMIO glue), [`tests/atcphy/`](../tests/atcphy/) (host tests).

**Status: designed + host-tested + compiles for the real target (clang
`--target=aarch64-linux-gnu`, this repo's exact `CFLAGS`). NOTHING in this
patch is hardware-proven.** No proxy session, no register trace, no boot
attempt was performed or is claimed. The session that produced this patch
operated under an absolute hardware-access ban (a live Windows session was
running on the target machine over USB serial) and never touched
`proxyclient/`, `screen`, or any `tools/run-*.sh`.

Labeling discipline: **designed** (written, not yet built) / **compiles**
(builds clean for the real target) / **host-tested** (pure-logic assertions
pass, including under ASan/UBSan) / **hardware-proven** (booted on the real
J414s). Everything here reaches at most **host-tested + compiles**.

---

## 0. The problem this patch targets

Hardware evidence from tonight's session (grade A, live measurement, not
reproduced or re-verified by this patch):

- J414s right-side USB-C port (`usb-drd2` @ `0xf02280000` = ACPI `XHC2`),
  HPM `hpm2` @ i2c0 `0x3b`. On attach, `STATUS` byte0 = `0xfd`:
  `PLUG_PRESENT`(bit0), `PLUG_UPSIDE_DOWN`(bit4), `PORTROLE`(bit5) +
  `DATAROLE`(bit6) already Source/DFP, `VCONN`(bit7). `INT_EVENT1` latches
  `02 05 00…` (PLUG_EVENT + status update) on every attach; acking
  `INT_CLEAR1` does not restore function and the same event re-latches.
- xHCI `PORTSC` for XHC2 stays `0x2a0`: `PP=1`, **`CCS=0`** -- the
  controller never sees a connection.
- Exactly one attach after boot works; every later attach is invisible.
  HPM role swaps change nothing; rewriting `PortInfo` is rejected.

The working hypothesis handed to this session: nothing in the boot chain
ever programs ATCPHY lane/orientation muxing, so the PHY's crossbar is left
in an undefined state that happens to pass through exactly one enumeration.
**Section 8 below gives this hypothesis a harder look than "assume it's
right and go implement the fix"** -- the short version is: it is plausible
and the delta this patch adds is a real, previously-completely-missing
piece of the boot chain, but this session cannot prove it is *the*
explanation, and an alternative (xHCI/ACPI-side state, outside this tree)
remains live.

---

## 1. Source inventory: what exists and where

Found first, per the task brief, before writing any code.

| Location | Content | Grade / use |
|---|---|---|
| `/Users/dj/Developer/Asahi/repos/linux-asahi` @ `030248d39b40`, branch `asahi` — `drivers/phy/apple/atc.c` (GPL-2.0 OR BSD-2-Clause, used here under BSD-2-Clause) | **The** authoritative RE artifact for this PHY. 2316 lines, reverse engineered by the Asahi authors from XNU debug output (file's own header comment). Register names/offsets, the full mode table, tunable application order, pipehandler BIST sequence, AUSPLL/DP bring-up. | Grade B for bit *semantics* (RE'd, not datasheet); grade A for *offsets* where independently cross-checked against the live ADT (sec 2). |
| same tree — `drivers/soc/apple/tunable.c` (GPL-2.0-only OR MIT, used under MIT) | The generic Apple-tunable RMW helper (`apple_tunable_apply`) and its wire format. 81 lines. | Grade A (small, mechanical, no RE ambiguity). |
| same tree — `arch/arm64/boot/dts/apple/t602x-dieX.dtsi`, `t602x-j414-j416.dtsi`, `t600x-j314-j316.dtsi` | T6020 node topology, HPM wiring, port roles. | Cited by the sibling doc below; not independently re-walked in this session (superseded by the live ADT capture, sec 2, for anything that matters to this patch). |
| `/Users/dj/Developer/m1n1` (this tree) — `src/usb.c`, `src/kboot_atc.c` | The **deployed, working** m1n1 binary's own USB2/pipehandler bring-up and the ADT→FDT tunable-forwarding table for Linux guests. Ground truth for "what m1n1 does today." | Grade A (it's the exact code running on the target). |
| `~/Developer/asnt-j414s-atcphy-usb4` (git worktree of `apple_silicon_nt_drivers`, branch `feature/j414s-atcphy-usb4`) — `docs/j414s-atcphy-usb4.md`, `drivers/AppleAtcPhy/AppleAtcPhyCore.{h,c}` | A same-night, same-family effort (one of the "three other agents" mentioned in this task's brief) building the **Windows-side** ATCPHY driver contract from the same sources. Read-only for this session (that tree belongs to a different agent); **not edited**. | Used as a second opinion / cross-check on the atc.c transcription (independently arrived at identical register offsets and mode table), and as the source for the "USB-C DP-altmode has no dcpext wiring on J414s" finding cited in sec 9. |
| A live ADT capture of the actual target: `~/Library/Application Support/ntasi/private-hardware-evidence/20260729-142731-j414s-ans-v5/j414s-adt.bin`, sha256 `93d96b4a3ea736288278606b723f263361c6ae6c3d5c4f24f08f6f7a73f4b66e` | The real J414s Apple Device Tree. Parsed with `PYTHONPATH=proxyclient .venv/bin/python3` + `m1n1.adt.load_adt` + `node.get_reg(i)`, hand-rolling the child-node recursion (per this project's own `m1n1-adt-walk-tree-is-broken` note -- `adt_first_child_offset`/`adt_next_sibling_offset` walked correctly for this purpose once done manually; `walk_tree` itself was not used). | **Grade A** for every address it produced. This is the single biggest addition this session made on top of prior research: it resolves an unknown the sibling doc explicitly flagged as unresolved (their U2). See sec 2. |

There is no local Linux driver for USB4/Thunderbolt NHI, ACIO, or the
per-lane DP analog calibration payload; none exists anywhere upstream
either (confirmed both independently and via the sibling doc's own survey).

---

## 2. Register map, with per-fact provenance

### 2.1 The window-ordering unknown, resolved from a live ADT capture

`atc.c` addresses everything through five named `devm_ioremap`'d windows
(`core`, `lpdptx`, `axi2af`, `usb2phy`, `pipehandler`, `atc.c:2217-2221`).
m1n1's own `usb.c` only ever uses two of them, addressed by raw ADT `reg[]`
index rather than name (`usb_drd_get_regs`, `usb.c:101-138`):
`regs.atc = atc-phyN reg[0]` and `regs.drd_regs_unk3 = usb-drdN reg[3]`. It
was **not previously known which ADT `reg[]` index corresponds to `core`**
(the window every crossbar/lane-mode/power-domain register in this patch
lives in) -- the sibling repo's doc explicitly recorded this as unresolved
(their "U2").

This session parsed the live ADT and dumped every `reg[]` entry (translated
through `node.get_reg(i)`, i.e. already parent-relative-corrected) for
`/arm-io/atc-phy0..3` and `/arm-io/usb-drd0..2`. Full table for port 0
(`atc-phy0`, base `0x700000000`; ports 1/2/3 are byte-identical with the
base swapped to `0xB00000000`/`0xF00000000`/`0x1300000000`):

| `reg[]` idx | Absolute addr (port 0) | Offset from port base | Identified as | Cross-check |
|---|---|---|---|---|
| 0 | `0x702a90000` | `+0x02a90000` | **usb2phy** | Matches the task brief's given `atc-phy0/1/2` addresses exactly; matches `usb.c`'s existing usage; matches `m1n1:proxyclient/hv/trace_atc.py` `REGMAPS[0] = Usb2PhyRegs` |
| 1 | `0x702800000` | `+0x02800000` | *unidentified* | Also present as `usb-drd0 reg[4]` (same address) -- a resource shared between the two ADT nodes. Not used by this patch; recorded so a future porter doesn't waste time re-discovering it. |
| 2 | `0x703020000` | `+0x03020000` | ATCPHY_POWER_CTRL/STAT/MISC block | = core-base(`reg[3]`) + `0x20000`, i.e. an ADT-level alias for the same silicon reachable from `core` by offset. This patch addresses it via `core + 0x20000`, not this separate reg entry. |
| **3** | `0x703000000` | `+0x03000000` | **core** (base of `ACIOPHY_CFG0`/`CROSSBAR`/`LANE_MODE`/`BIST_*`/`SLEEP_CTRL`/`PLL_PCTL_FSM_CTRL1`/`PLL_COMMON_CTRL`, all offsets `< 0x2000`) | `atc.c` compatible string `apple,t6020-atcphy`/`apple,t8103-atcphy`; T6050 layout stability claim in the sibling doc |
| 4 | `0x703002000` | `+0x02000` (core-rel.) | AUSPLL "top" (`AUSPLL_APB_CMD_OVERRIDE` @ `0x2000` etc.) | = `kboot_atc.c`'s `tunable_AUSPLL_TOP` block_offset `0x2000` exactly |
| 5 | `0x703002200` | `+0x2200` | AUSPLL "core" (`AUSPLL_CLKOUT_MASTER`/`BGR`/`DTC_VREG`/`FREQ_CFG` etc.) | = `kboot_atc.c`'s `tunable_AUSPLL_CORE` block_offset `0x2200` |
| 6 | `0x703002800` | `+0x2800` | CIO3PLL "top" | = `kboot_atc.c`'s `tunable_CIO3PLL_TOP`/`tunable_CIO_CIO3PLL_TOP` `0x2800` |
| 7 | `0x703002a00` | `+0x2a00` | CIO3PLL "core" (`CIO3PLL_CLK_CTRL` @ `0x2a00`) | = `kboot_atc.c`'s `tunable_CIO3PLL_CORE` `0x2a00`; matches `atc.c:112` |
| 8 | `0x703007000` | `+0x7000` | DP TX control block (`ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0` @ `0x7000`, `ACIOPHY_DP_PCLK_STAT` @ `0x7044`) | matches `atc.c:205,217` |
| 9 | `0x703000a00` | `+0xa00` | AUS_CMN_SHM | = `kboot_atc.c`'s `tunable_AUS_CMN_SHM` `0xa00` |
| 10 | `0x703000800` | `+0x800` | AUS_CMN_TOP | = `kboot_atc.c`'s `tunable_AUS_CMN_TOP` `0x800` |
| 11/13/15/17/26 | `+0xd000/0xc000/0xb000/0x9000/0xa000` | — | Lane-0 AUSPMA TX_SHM/TX_TOP/RX_SHM/RX_TOP/RX_EQ | all five match `atc.c:220-224` `LN0_AUSPMA_*` exactly |
| 12/14/16/18/27 | `+0x14000/0x13000/0x12000/0x10000/0x11000` | — | Lane-1 AUSPMA TX_SHM/TX_TOP/RX_SHM/RX_TOP/RX_EQ | match `atc.c:226-230` `LN1_AUSPMA_*` |
| 19 | `+0x1000` | — | overlaps `reg[3]`'s range (PLL/BIST shared region) | not separately addressed by this patch |
| **20** | `0x703050000` | `+0x03050000` | **lpdptx** | matches sibling doc's `dtsi`-cited `+0x03050000` |
| 21 | `0x703050200` | lpdptx `+0x200` | lpdptx AUX_SHM_CFG_BLK_AUX_CTRL_REG0/1 | = `atc.c:412,415` (`0x0204`/`0x0208`) |
| 22 | `0x703054000` | lpdptx `+0x4000` | lpdptx AUX_CONTROL block | = `atc.c:419` (`0x4000`) |
| 23 | `0x29e2cc000` | *absolute, identical across all 4 ports* | *unidentified* shared/global block, size `0x2000` | Not used by this patch. Flagged as an open question -- possibly a shared efuse/calibration SRAM, not confirmed. |
| **24** | `0x700000000` | `+0x0`, size `0x1000000` (16MB) | **axi2af** (the real per-tunable AXI2AF registers occupy only the first page) | = `kboot_atc.c`'s `tunable_ATC0AXI2AF` block_offset `0x0` |
| **25** | `0x703045000` | `+0x45000` | ATC fabric | = `kboot_atc.c`'s `tunable_ATC_FABRIC` block_offset `0x45000` exactly |

**Every one of `kboot_atc.c`'s 28 tunable `(adt_name, block_offset)` pairs
matched a distinct `reg[]` entry at exactly that core-relative offset.**
This is a strong, ADT-grounded (not merely RE'd-and-hoped) cross-check that
the offsets this patch uses for the `core` window are correct for this
specific machine -- upgrading the sibling doc's grade-B "T6050 layout
carries over to T6020, only the base changes" claim to grade A for J414s
specifically, for every offset this patch touches.

`usb-drd0/1/2 reg[3]` (pipehandler) was already used by the deployed
`usb.c`; this session did not need to re-derive it, only note that it is a
**different ADT node** (`usb-drd`, not `atc-phy`) from everything else in
this table.

**Consequence for the DWC3-safety rule:** `usb-drd` node's `reg[0]`
(`0x702280000`, size `0x11800`, the dwc3 "core"+"apple" windows) and
`reg[2]` (`0x70228c000`, size `0x5800`) are the actual DWC3 controller
register file. **This patch's code never resolves or touches those two
`reg[]` entries at all** -- `atcphy_get_regs()` (`src/atcphy.c`) only ever
requests `usb-drd` `reg[3]` (pipehandler). This is a structural guarantee,
not just a policy statement: there is no code path in this patch that can
compute a dwc3-core address.

### 2.2 Bit-field semantics (grade B unless noted)

All bit names/offsets below are transcribed from `atc.c` with line
citations inline in `src/atcphy_core.h`/`.c`; only the highlights are
repeated here. Everything m1n1's own `usb.c` already exercises (`usb2phy`,
`pipehandler` MUX_CTRL/AON_GEN/NONSELECTED_OVERRIDE) is grade A (the
deployed binary proves these bits do what `usb.c`'s comments say). Every
other core-window register (`ACIOPHY_CFG0`, `ACIOPHY_CROSSBAR`,
`ACIOPHY_LANE_MODE`, `ATCPHY_POWER_CTRL/STAT/MISC`, `ACIOPHY_SLEEP_CTRL`,
`ACIOPHY_TOP_BIST_*`, all of `AUSPLL_*`/`CIO3PLL_*`, `LPDPTX_*`) is grade B:
plausible names from XNU debug strings, **never exercised by m1n1 before
this patch, and not independently confirmed by this session** beyond the
address cross-check in 2.1.

- `ACIOPHY_CROSSBAR` (`0x4c`): 5-bit protocol field.
  `USB3=0xa`/`USB3_SWAPPED=0xb`, `USB3+DP=0x10`/`0x11`, `DP=0x14`,
  `USB4=0x0`/`0x1`. Plus a 12-bit `DP_SINGLE_PMA` field and a `DP_BOTH_PMA`
  bit for DP lane-PMA routing.
- `ACIOPHY_LANE_MODE` (`0x48`): four 3-bit fields (RX0/TX0/RX1/TX1),
  values `USB4=0`/`USB3=1`/`DP=2`/`OFF=3`.
- `ATCPHY_POWER_CTRL`/`_STAT` (`0x20000`/`0x20004`): `SLEEP_SMALL`,
  `SLEEP_BIG`, `CLAMP_EN`, `APB_RESET_N`, `PHY_RESET_N` -- a small "wake the
  power domain, wait for STAT to agree, drop the clamp" state machine that
  m1n1 has **never run** (sec 3).
- `ATCPHY_MISC` (`0x20008`): `RESET_N`, `LANE_SWAP` (bit 2) -- this is the
  bit that actually encodes "cable is flipped" independent of the
  crossbar's protocol-value encoding.
- Pipehandler `MUX_CTRL` (`0x0c`): two 3-bit fields (CLK/DATA).
  `DUMMY=CLK4|DATA2`, `USB3=CLK1|DATA0`, `USB4=CLK2|DATA1`. m1n1's own
  whole-register writes `0x22`/`0x08`/`0x11` (`usb.c:44-46`) decode exactly
  onto these fields -- this is the one piece of the pipehandler model that
  is grade A end to end.

---

## 3. What m1n1 does today (hardware-proven baseline; nothing here changed)

`usb_phy_bringup()` (`usb.c:140-174`) and `usb_phy_handoff_host()`
(`usb.c:183-236`) between them touch exactly:

- the **usb2phy** window (`USBCTL`/`CTL`/`SIG`/`MISCTUNE`, offsets
  `0x00`/`0x04`/`0x08`/`0x1c`) -- device-mode bring-up, then a full
  SIDDQ/reset/APB-reset dance to re-latch the role to host at guest
  handoff; and
- the **pipehandler** window (`MUX_CTRL`/`AON_GEN`/`NONSELECTED_OVERRIDE`)
  -- always leaving `MUX_CTRL = DUMMY` (SuperSpeed permanently on the
  "dummy" PIPE backend, USB2-only).

**m1n1 never writes a single register in the `core`, `axi2af`, or `lpdptx`
windows.** `ACIOPHY_CROSSBAR`, `ACIOPHY_LANE_MODE`, `ATCPHY_POWER_CTRL`,
`ATCPHY_MISC`, every tunable target -- all of it sits at whatever value the
SoC's power-domain-enable (`pmgr_adt_power_enable`, a *different*,
SoC-wide PMGR mechanism from the ATCPHY-internal `POWER_CTRL`/`POWER_STAT`
state machine) left behind. `kboot_atc.c` reads the ADT tunables and
forwards them into the FDT so a **Linux guest's own driver** can apply
them; it applies nothing itself and the Windows path never consumes it
(`kboot_atc.c:306-322`, called from `kboot`, not from the Windows boot
path at all).

This is the literal, complete truth of "there is no ATC PHY driver in
m1n1" from the task brief -- confirmed by reading the deployed source, not
assumed.

---

## 4. The mode × orientation table

Transcribed field-for-field from `atcphy_modes[]` (`atc.c:634-786`) into
`atcphy_mode_config(mode, swapped)` (`src/atcphy_core.c`). Full table
(only the fields that differ from "off" are worth restating here; see the
header for the exhaustive citations):

| Mode | crossbar (n / s) | lane_mode[0,1] (n / s) | set_swap (n/s) | dp_aux | pipe_state |
|---|---|---|---|---|---|
| OFF | `0xa` / `0xb` | OFF,OFF / OFF,OFF | no/no | no | DUMMY |
| USB2 | `0xa` / `0xb` | OFF,OFF / OFF,OFF | no/no | no | DUMMY |
| USB3 | `0x10` / `0x11` | USB3,DP / DP,USB3 | no/**yes** | no | **USB3** |
| USB3+DP | `0x10` / `0x11` | USB3,DP / DP,USB3 | no/**yes** | **yes** | **USB3** |
| TBT | `0x0` / `0x1` | USB4,USB4 / USB4,USB4 | no/no | no | DUMMY (not USB4 -- see below) |
| USB4 | `0x0` / `0x1` | USB4,USB4 / USB4,USB4 | no/no | no | USB4 (unimplemented, falls back) |
| DP | `0x14` / `0x14` | DP,DP / DP,DP | no/no | **yes** | DUMMY |

Two deliberately-asymmetric facts, preserved exactly (and asserted by
`test_mode_config_dp_asymmetric_swap` in the host tests):

- **USB3-only intentionally programs the "other" lane as DP**, not OFF
  (`atc.c` comment at 681-684: 20 Gbps / USB3+USB3 is not supported by this
  silicon). The only USB3 vs USB3+DP difference is `enable_dp_aux`.
- **The swapped `DP` mode entry is not a mirror of the normal one**:
  `crossbar_dp_both_pma` and `set_swap` are both forced `false` in the
  swapped case (`atc.c:774-781`), and `crossbar_dp_single_pma` changes from
  `UNK100` to `UNK008`. This looks deliberate in `atc.c` (there's no
  comment explaining why) and this patch does not second-guess it.

**The important, sobering finding for this task's actual bug:** a plain
non-alt-mode USB attach on real hardware does **not** go through
`APPLE_ATCPHY_MODE_USB2`. Linux's typec-mux consumer
(`atcphy_mux_set`, `atc.c:2089-2156`) maps `TYPEC_STATE_USB` --
i.e. an ordinary attach with no alt-mode negotiated, which covers the
overwhelming majority of plugged devices including pure-USB2 ones -- to
**`APPLE_ATCPHY_MODE_USB3`**, running the full pipehandler BIST dance.
`APPLE_ATCPHY_MODE_USB2` is reserved for one narrow case: a USB4-capable
partner that explicitly negotiates `Enter_USB` with `EUDO_USB_MODE_USB2`
(`atc.c:2100-2117`). See sec 8 for what this means for the "smallest safe
fix" this patch actually wires up.

---

## 5. What is implemented

### 5.1 `src/atcphy_core.{h,c}` -- pure logic, host-tested, zero MMIO

Following this repo's established pattern
(`src/tps6598x_host_policy.{h,c}` + `tests/usb/test_tps6598x_host_policy.c`):
an `ATCPHY_CORE_HOST_TEST` build switch selects `stdint.h` typedefs instead
of `"types.h"`, and the file never calls `read32`/`write32`/`udelay`
directly. Contents:

1. The full T6020 register map (sec 2), as `#define`s with inline
   citations.
2. The mode × orientation table (sec 4) and a reverse pipehandler-mux
   decoder (`atcphy_pipe_mux_decode`).
3. The DisplayPort link-rate table (`ATCPHY_DP_RATE_{RBR,HBR,HBR2,HBR3}`,
   wire codes, kHz, and the per-rate AUSPLL frequency-descriptor constants
   from `dp_lr_config[]`, `atc.c:787-831`).
4. The 28-entry `atc-phy,t6020` tunable vocabulary
   (`atcphy_tunable_table[]`), now carrying **ADT-confirmed** absolute
   offsets (sec 2.1) instead of merely `kboot_atc.c`-transcribed ones, plus
   `atcphy_tunable_validate()` (byte-for-byte the same checks as
   `kboot_atc.c:206-256`: 12-byte records, `size==32`, 4-byte alignment,
   `offset+4 <= block_size`) and `atcphy_tunable_apply_one()` (the exact
   `apple_tunable_apply` RMW: `new = (old & ~mask) | value`).
5. A declarative op-list model (`atcphy_seq_op_t` /
   `WRITE`/`SET`/`CLEAR`/`MASK`/`DELAY_US`/`POLL_SET`/`POLL_CLEAR`) and a
   pure interpreter, `atcphy_seq_apply()`, that takes caller-supplied
   read/write/delay function pointers -- host tests use a small in-memory
   fake register file (`tests/atcphy/test_atcphy_core.c`); `src/atcphy.c`
   supplies `read32`/`write32`/`udelay`.
6. Every fixed (parameter-free) sequence from `atcphy_configure()`
   transcribed as a static op-list table, each op citing its `atc.c` line:
   usb2-PHY power on/off, ATCPHY core "small"/"big" power-domain wake/sleep,
   the `ACIOPHY_CFG0`/`ACIOPHY_SLEEP_CTRL` override dance, CIO3PLL clock
   enable, the two `AUSPLL_FSM_CTRL`/`APB_CMD_OVERRIDE` unconditional
   writes, the final `PHY_RESET_N` release, pipehandler→dummy, the
   pipehandler→USB3 host-mode BIST dance, and lpdptx AUX enable/disable.
7. Two parameterized builders (mode+orientation → crossbar/lane_mode/misc
   ops; DP rate → AUSPLL FREQ_DESC A/B/C ops), since these depend on a
   runtime argument and can't be static tables.

**11 test groups, ~35 assertions**, covering: the mode table's structural
invariants (OFF/USB2 never touch the pipe mux or enable DP AUX -- the
exact property `src/atcphy.c`'s safety gate depends on), the USB3
swap/mirror behavior, the DP asymmetric-swap quirk, pipe-mux
encode/decode against m1n1's own whole-register constants, DP rate
round-trips, tunable vocabulary lookups and offset cross-checks against
sec 2.1, tunable blob validation (empty/bad-length/bad-record/good, with
the exact failing record index reported), the RMW arithmetic, fuse
policy, the op-list interpreter's every op kind including POLL success
and POLL timeout-with-failing-index, block-scoping assertions on every
fixed sequence (e.g. `pipehandler_dummy` only ever touches the
`PIPEHANDLER` block; `usb2_power_on` only touches `USB2PHY`), and the two
parameterized builders replayed through the interpreter against a fake
register file with the resulting register *content* checked (not just
"it ran"): OFF-mode crossbar reads back `0xa`/`0xb` by orientation and
`LANE_MODE` reads back all-OFF; USB3-swapped sets `MISC.LANE_SWAP` and
mirrors the lane fields; DP rate builds produce a `FREQ_DESC_A` whose
target field matches the table and a `DTC_VREG_BYPASS` bit matching
`plla_clkout_vreg_bypass`.

```
$ sh tests/atcphy/run-host-tests.sh
atcphy_core tests: PASS
atcphy_core tests: PASS     # (ASan/UBSan build)
```

Both the plain build and an `-fsanitize=address,undefined` build are run
by the script, matching `tests/usb/run-host-tests.sh`'s pattern; both are
green. Wired into `.github/workflows/test.yml` alongside the existing
`tests/pcie` step.

### 5.2 `src/atcphy.{h,c}` -- m1n1 MMIO glue

- `atcphy_get_regs(idx, atcphy_regs_t*)`: resolves the five windows from
  the live ADT using the `reg[]` indices established in sec 2.1
  (`atc-phy%u` `reg[0]`/`reg[3]`/`reg[20]`/`reg[24]` for
  usb2phy/core/lpdptx/axi2af; `usb-drd%u` `reg[3]` for pipehandler), via
  `adt_path_offset_trace` + `adt_get_reg` -- the same primitives
  `usb.c`'s own `usb_drd_get_regs` uses. **Never resolves `usb-drd`
  `reg[0]`/`reg[2]` (the actual dwc3 register file) at all.**
- `atcphy_set_orientation(idx, flipped)`: the safe, orientation-only entry
  point the task brief asked for by name. Always programs
  `(ATCPHY_MODE_USB2, flipped)` -- pipehandler stays on `DUMMY`,
  unconditionally, every time. See sec 6 for exactly why this one is safe
  to call and sec 8 for whether it's *sufficient*.
- `atcphy_apply_mode(idx, mode, flipped, allow_pipe_switch, dp_rate_valid,
  dp_rate)`: the general entry point covering the whole mode table,
  orchestrated to mirror `atcphy_configure()`'s exact op order (power-on →
  global tunables → mode-scoped lane tunables → AUSPLL FSM override →
  CFG0/SLEEP_CTRL dance → DP AUX enable if needed → CIO3PLL enable → lane
  config → PHY_RESET_N release → pipehandler → optional DP rate). Tunables
  are read **directly from the live ADT by name** and applied via
  read-modify-write to the resolved absolute offset -- this sidesteps a
  known local-tree-vs-upstream FDT property-name mismatch
  (`apple,tunable-common` vs `apple,tunable-common-a`/`-b`, flagged by the
  sibling repo's own header comment) entirely, since this patch never
  goes through the FDT bridge.

---

## 6. The safety gate: why `allow_pipe_switch` exists

The task brief is explicit: *"Windows owns XHC1/XHC2 at runtime... Do not
design anything that re-initialises DWC3 or xHCI behind a running
Windows... flag clearly if your design needs a DWC3 touch."*

`atcphy_apply_mode()` refuses to run at all if the target mode's
`pipe_state` is `ATCPHY_PIPE_STATE_USB3` (i.e. USB3 or USB3+DP) unless the
caller explicitly passes `allow_pipe_switch=true`:

```c
if (cfg->pipe_state == ATCPHY_PIPE_STATE_USB3 && !allow_pipe_switch) {
    printf("atcphy%u: mode %d needs a pipehandler PIPE-mux switch; "
           "refusing without allow_pipe_switch=true ...\n", idx, (int)mode);
    return -1;
}
```

**No call site in this patch ever passes `allow_pipe_switch=true`.**
`atcphy_set_orientation()` -- the only function anything could plausibly
wire up automatically -- hardcodes `ATCPHY_MODE_USB2`, whose `pipe_state`
is always `DUMMY`, so the gate is structurally unreachable from it.

Why this matters concretely: `pipehandler` is a *different* MMIO window
from the dwc3 core/apple registers (sec 2.1's structural guarantee), so
switching it is not literally "touching DWC3." But it reprograms the PIPE
interface DWC3's SuperSpeed side is wired through, while a running guest's
xHCI driver believes that interface is permanently parked on the "dummy"
backend (which is m1n1's current, working, hardware-proven state). Flipping
it live, under a bound driver, is exactly the class of operation the task
brief's `0x144 BUGCODE_USB3_DRIVER` warning is about. This patch **compiles
and host-tests** the USB3 BIST sequence (`atcphy_seq_pipehandler_usb3_host_bist`,
faithfully transcribed including the "already locked, unlock first" guard
from `atcphy_pipehandler_check`, `atc.c:956-973`) because the task's
"go full" scope asked for it to exist and be correct, but it is **not
reachable from any automatic path in this tree**, and this session did not
attempt to determine on hardware whether it is safe to run under a live
guest -- that experiment (the sibling doc's own "L2", "mode switch under a
running inbox xHCI... unproven anywhere") is explicitly out of reach under
this session's hardware ban.

**If this driver is ever extended to call `atcphy_apply_mode()` with
`allow_pipe_switch=true` from an automatic path (e.g. a future HPM-event
handler), that is the moment this stops being "PHY/mux-only" in the sense
the task brief cares about, and it should not happen without: (a) a
hardware session with recovery available, and (b) the L2 experiment run
first with a fail-closed revert path.** This is the one loud flag this
document is required to raise, raised.

---

## 7. What is explicitly NOT implemented, and why

- **Per-lane DP analog calibration** (`atcphy_dp_configure_lane`,
  `atc.c:1283-1493`, ~300 lines / ~100 register-field `#define`s of
  blind TX/RX SHM pokes with no semantics beyond XNU debug names, run once
  per DP-lane on every link-rate change). `atcphy_build_dp_rate_ops()`
  programs the AUSPLL frequency descriptors and waits for lock, but
  **stops there** -- a DP link brought up by this patch alone is not
  expected to train. Rationale: this sequence is (a) the single largest
  remaining block of unported code by a wide margin, (b) genuinely
  unverifiable offline (it's calibration, not policy -- there's no
  "correct" test other than a live PHY), and (c) low-value for *this*
  machine specifically: the sibling doc's own exhaustive grep of the
  pinned Asahi DT tree found **no device tree anywhere that wires a
  Type-C receptacle's DP lanes to a `dcpext`** on j31x/j41x boards -- only
  the fixed HDMI path (`atcphy3`) gets one. So even a fully-calibrated
  USB-C DP-alt PHY on this machine has nowhere to send pixels without
  separate, unrelated display-crossbar work this patch doesn't touch
  either. DP-altmode was explicitly out of scope in the original brief for
  exactly this kind of reason; the "go full" scope update asked for DP
  lane *configuration* (crossbar/lane_mode/AUSPLL), which is implemented,
  and this document names the one piece still missing rather than silently
  shipping a DP mode that can never actually light up an external
  Type-C monitor on this hardware.
- **USB4/Thunderbolt PIPE backend.** `ATCPHY_PIPE_STATE_USB4` exists in the
  mode table (crossbar/lane_mode values are real, transcribed from
  `atc.c`) but `atcphy_apply_mode()` falls back to the dummy pipehandler
  sequence for it with a loud `printf`, **matching upstream Linux's own
  behavior** (`atc.c:1133-1136`: `"ATCPHY_PIPEHANDLER_STATE_USB4 not
  implemented; falling back to USB2"`). No open-source reference for the
  real USB4 PIPE handshake exists anywhere. ACIO/NHI (the Thunderbolt
  router/coprocessor) is untouched entirely -- no register model exists in
  any open source for it, matching the sibling doc's own "not a realistic
  target" verdict.
- **Continuous, interrupt-driven re-triggering on every physical plug
  event while Windows owns the port.** This is the literal meaning of
  "wire it to the HPM event path" for a driver that is supposed to react
  to *every* re-plug, not just the one at boot. m1n1 in this configuration
  is a resident EL2 hypervisor for the whole Windows session (this
  project's own `m2-pro-windows-boot-chain` memory note), so it is
  architecturally *possible* for it to keep servicing a routed HPM
  interrupt after handoff -- but doing that means routing a new physical
  IRQ (the shared `pinctrl_ap` line, GPIO 44) to EL2, adding an
  `hv_exc.c` handler, and running I2C from EL2 context concurrently with
  whatever the guest is doing on the same fabric. This project's own
  memory carries five separate prior incidents in exactly this area
  (`native-aic-gsiv-vs-gic-limits`, `aic-line-mask-publish-race`,
  `hardware-debug-session-safety`, the fail-closed-by-design IPI note,
  and the M2 Pro boot-chain note itself). Building that blind, under an
  absolute hardware ban, with no way to test it, is not a responsible use
  of this session -- it would be exactly the kind of unverified,
  high-blast-radius change this project's history says not to make. **This
  is the honest, named gap**, not a silently-dropped requirement: see sec
  8.3 for what this patch does instead.

---

## 8. Does this fix the measured bug? An honest assessment

### 8.1 The hypothesis this patch was asked to implement

"Nothing programs ATCPHY lane/orientation muxing; program it on attach and
USB2 hotplug will work." This patch implements exactly that as
`atcphy_set_orientation()`.

### 8.2 A harder look, done in this session

Two observations complicate a confident "yes, this fixes it":

1. **The crossbar was never touched at the time of the one attach that
   *did* work either** (sec 3: m1n1 has never written a `core`-window
   register, ever, in any configuration). If an unprogrammed crossbar were
   the direct, sole cause of `CCS=0`, the very first attach should have
   failed too, since nothing about the crossbar's state changes between
   "first attach" and "later attach" -- m1n1 doesn't touch it either time.
   Something *else* is different about the first attach (most likely:
   it happens immediately after `usb_phy_handoff_host()`'s USB2-PHY
   reset/role-relatch/VBUS-force dance, while every later attach happens
   with the USB2 PHY already settled and Windows fully owning the port).
2. **On real hardware, per sec 4, a plain attach does not even use
   `APPLE_ATCPHY_MODE_USB2`** -- Linux promotes essentially every attach
   (including plain USB2 devices) to `APPLE_ATCPHY_MODE_USB3`, running the
   full pipehandler BIST dance, and only reserves `MODE_USB2` for a narrow
   USB4-negotiated fallback. So the "safe" fix this patch auto-wires
   (`ATCPHY_MODE_USB2`, crossbar `0xa`/`0xb`, pipehandler untouched) is
   **not literally what Apple's own reference driver does for the
   scenario being debugged** -- it's the PHY-side state Apple's driver
   uses in a different, narrower case. The fix that *would* match Apple's
   reference behavior for a generic attach (switch pipehandler to USB3
   BIST-host mode) is exactly the one this patch refuses to run
   automatically (sec 6), because doing so under a running guest's xHCI is
   the specific thing the task brief prohibits without a live, recoverable
   hardware session.

### 8.3 What this patch actually is, stated plainly

- A real, previously-100%-missing piece of the boot chain
  (`core`-window power-on + crossbar/lane_mode programming) is now
  implemented, host-tested, and safe to call at any time including live
  under Windows.
- It is **plausible but not proven** that leaving the `core` window
  entirely unprogrammed (its literal, current, hardware-proven state)
  produces a marginal condition that degrades after the first
  enumeration cycle -- e.g. because the `ATCPHY_POWER_CTRL` "small"/"big"
  sleep domains were never explicitly woken, so the crossbar/lane_mode
  registers may be running on ungated-but-uncalibrated clocks, or because
  some downstream consumer of `ATCPHY_MISC.RESET_N` (never set by m1n1)
  gates behavior this session doesn't have visibility into.
- The **alternative hypothesis this session could not rule out**: the
  observed "one clean attach, then permanently invisible" pattern is a
  xHCI/ACPI/Windows-driver-side latch (port-change-event masking, a
  `_PS0`/`_PS3` power-state assumption, or a VBUS-debounce state that
  m1n1's one-shot `SIG_VBUS` force at handoff time doesn't refresh) that
  lives entirely outside this tree and this patch does not touch. Nothing
  in this session's evidence rules this out, and the m1n1-only "safe" fix
  cannot address it if true.
- **The honest bottom line: this patch is the right next experiment to
  run, not a proven fix.** The correct way to falsify/confirm sec 8.2's
  concern is a live MMIO trace across exactly one boot → first-attach →
  detach → second-attach cycle, with and without `atcphy_set_orientation()`
  wired into the boot path, comparing `CCS` behavior both ways. That
  experiment needs the hardware session this task explicitly forbade.

---

## 9. Deliverable-4 status: HPM event path

Per the task's own priority ordering ("only if 1-3 are solid"): 1-3 are
solid (sec 5); deliverable 4 is **partially done, and the part that is not
done is named rather than faked**.

What exists: `atcphy_apply_mode()`/`atcphy_set_orientation()` are complete,
tested, callable entry points that take an orientation and program the
PHY accordingly -- the "reprogram the mux" half of "an attach reprograms
the mux." What does **not** exist in this patch: any code path that reads
the HPM's `STATUS`/`DATA_STATUS` registers and calls into these functions.
m1n1's own `src/tps6598x.c` currently implements only `CMD1`/`DATA1`
(4CC command execution), `INT_EVENT1`/`INT_MASK1`/`INT_CLEAR1` (IRQ
mask/ack, used only to quiesce the HPM during proxy operation), and
`POWER_STATE`/`SYSTEM_CONFIG` -- **it never reads `STATUS` at all**, so
there is currently no plug/orientation *sensor* in this tree for an event
handler to consume. Adding one (a `tps6598x_read_status()` +
orientation-bit decode, following this same session's own
hardware-measured `STATUS` byte0 layout from sec 0: bit0 `PLUG_PRESENT`,
bit4 `PLUG_UPSIDE_DOWN`) is a small, low-risk addition that was judged
lower priority than getting the PHY-side logic right and thoroughly
tested, given the session's time budget; it is not implemented here.

Even with that sensor added, **continuously reacting to it while Windows
owns the port requires the EL2-resident interrupt wiring named as an
explicit gap in sec 7** -- a one-shot read at boot/handoff time (before
Windows starts) can only prime whatever is plugged in at that instant, not
react to a later hot-replug, which is the actual bug being chased. Given
that limitation, this session chose not to wire even the boot-time-only
version into `usb.c`, to avoid shipping a call site that looks like it
addresses the bug but structurally cannot (a one-shot prime at handoff
time cannot affect any attach after boot, which is exactly the failure
mode in sec 0). The honest state is: **the entry points exist and are
tested; nothing in this tree calls them yet; the reason is architectural
(no live sensor, no resident IRQ path), not an oversight.**

---

## 10. Build and test

```sh
# Host tests (pure logic, no cross-compiler needed):
sh tests/atcphy/run-host-tests.sh
# -> "atcphy_core tests: PASS" x2 (plain + ASan/UBSan)

# Real-target compile (this repo's actual toolchain/flags):
gmake BUILD_DIR=build -j4   # or `make`; see note below
```

**Build note, unrelated to this patch, found while verifying it:** a clean
`make`/`gmake` in this checkout currently fails at link time with
`undefined symbol: _binary_font_bin_start` etc. -- `data/font.bin` and
`data/font_retina.bin` are missing from this checkout (`data/bootlogo_*.bin`
are present). This reproduces identically with this patch's changes
stashed out (`git stash push` + rebuild + `git stash pop`, verified in this
session), so it predates and is unrelated to this work -- it is `fb.c`'s
boot-splash font linking, nothing to do with USB. Also: plain `/usr/bin/make`
on this macOS host is GNU Make 3.81 (2006), which silently mis-expands
`-I$(BUILD_DIR)` in `BASE_CFLAGS` (a `:=`-immediate-expansion-before-`BUILD_DIR`
-is-defined ordering issue, also pre-existing and also unrelated to this
patch) when `chainload.c` is the very first object built in a from-scratch
tree; `gmake BUILD_DIR=build` (Homebrew's GNU Make 4.4.1, `brew --prefix make`)
works around it by pre-setting the variable. **Neither issue is touched by
this patch** -- confirmed by reproducing both with the patch's files
stashed out.

What *was* verified clean in this session: every object file in
`OBJECTS` (added `atcphy.o atcphy_core.o` to the `Makefile` list next to
the existing `usb.o`/`tps6598x.o` entries) compiles with the project's
real `clang --target=aarch64-linux-gnu` invocation and its real `CFLAGS`
(`-Wall -Wundef -Werror=strict-prototypes -Werror=implicit-function-declaration
-Werror=implicit-int -Wsign-compare -Wunused-parameter ... -mgeneral-regs-only
-march=armv8.2-a`), including `atcphy.o`/`atcphy_core.o` themselves --
**zero warnings, zero errors** from either file (spot-checked against the
several dozen pre-existing warnings in unrelated files like `hv_vgic.c`/
`dcp_iboot.c`/`dlmalloc/malloc.c`, none of which this patch touches or
introduces).

---

## 11. Honest summary for the reader in a hurry

| Deliverable (task's priority order) | Status |
|---|---|
| 1. Design note with register map + provenance | **This document.** |
| 2. Host-testable core, unit-tested | **Done.** `src/atcphy_core.{h,c}`, 11 test groups, green plain + ASan/UBSan. |
| 3. m1n1 MMIO glue + `atcphy_set_orientation(port, flipped)` | **Done.** `src/atcphy.{h,c}`, compiles clean for the real target. Not yet called from anywhere (sec 9). |
| 4. Wire to the HPM event path | **Not done**, named explicitly (sec 9): no `STATUS` reader exists yet, and a boot-time-only prime can't fix a post-boot-replug bug, so wiring a call site that can't work was judged worse than not wiring one. |
| "Go full" additions (USB3 lane mux, PIPE BIST, AUSPLL/CIO3PLL, DP crossbar/rate) | **Designed + host-tested + compiles.** Deliberately never auto-called (sec 6) because doing so live is the exact DWC3-adjacent risk the brief prohibits without a recoverable hardware session. |
| Per-lane DP analog calibration | **Not implemented**, named (sec 7) -- and would be useless on this machine's DT topology even if it were (sec 7). |
| USB4/Thunderbolt | **Not implemented**, matches upstream's own non-implementation, named (sec 7). |
| Does this fix the measured `CCS=0` bug? | **Unknown.** Plausible, not proven; a real alternative hypothesis (xHCI/ACPI-side state) was not ruled out; the experiment that would settle it needs the hardware session this task forbade (sec 8). |

Every register offset in this document that is not independently
cross-checked against the live ADT capture (sec 2.1's table) or against
m1n1's own already-deployed code (sec 3) should be read as **grade B: a
plausible name from RE'd XNU debug output, unverified on this hardware.**
That is most of the bit-level *semantics* (what a given bit actually does,
as opposed to where it lives) in sections 2.2 and 5. Nothing in this patch
was assumed correct without a citation; nothing was executed on hardware.

---

## 12. Session-2 additions (2026-07-30): build wiring, proxy interface, power-off

A follow-up session (same hardware ban in force; still nothing
hardware-proven) verified this patch's transcription against
`atc.c @ 030248d39b40` line-by-line for the mode table, `dp_lr_config[]`,
`atcphy_usb2_power_on/off`, `atcphy_power_on/off`, the full
`atcphy_configure()` ordering, both pipehandler sequences (incl. lock/
unlock/check), `atcphy_configure_lanes`, `atcphy_enable/disable_dp_aux`,
and `atcphy_dp_configure` -- **no divergences found** -- and then completed
the integration work:

### 12.1 Build wiring (salvaged from the abandoned stash)

The `Makefile` (`atcphy.o atcphy_core.o` in `OBJECTS`) and
`.github/workflows/test.yml` (host-test step) changes from the previous
session's `stash@{0}` are now applied and committed.

**Sec 10's build note is now stale, in a good way:** the "missing
`data/font.bin`" link failure was misdiagnosed. `font/font.bin` and
`font/font_retina.bin` exist in-tree; the real problem was *stale
`build/*.o` blobs produced by an earlier GNU Make 3.81 run*, whose objcopy
invocation embedded the wrong symbol prefix
(`_binary_build_bootlogo_48_bin_start` instead of
`_binary_bootlogo_48_bin_start`). Deleting
`build/{bootlogo_*,font*}.o` and rebuilding with Homebrew's
`gmake BUILD_DIR=build` links a complete `build/m1n1.bin` (verified in this
session, 2026-07-30). Plain `/usr/bin/make` (3.81) remains unusable for a
from-scratch build (sec 10's `-I$(BUILD_DIR)` expansion issue stands).

### 12.2 `atcphy_power_off()` -- upstream-faithful OFF + recovery path

`atcphy_apply_mode(idx, ATCPHY_MODE_OFF, ...)` keeps the PHY powered and
parks the lanes (it runs the power-on + OFF-mode-table path). Upstream's
`atcphy_configure(MODE_OFF)` instead short-circuits to `atcphy_power_off`
(atc.c:1733-1737). The new `atcphy_power_off(idx)` provides that
upstream-faithful state: usb2 PHY off (atc.c:1616-1635) -> unconditional DP
AUX disable (atc.c:1642,1267-1281) -> core big/small domain sleep
(atc.c:1637-1667), matching `atcphy_probe_finalize`'s reset order
(atc.c:2242-2247) EXCEPT that it deliberately skips (a) the dwc3
reset-assert via pipehandler `AON_GEN` (a DWC3 touch, banned by sec 6's
rule) and (b) the lock-less pipehandler->dummy mux write
(`atcphy_setup_pipehandler`, atc.c:1141-1161). This is the
recover-to-baseline command for hardware experiments.

### 12.3 SoC-level PMGR power (new, glue-only)

`atcphy_apply_mode`/`atcphy_power_off` now call `pmgr_adt_power_enable` for
`/arm-io/atc-phyN` + `/arm-io/usb-drdN` before touching MMIO -- the same
SoC-wide enable `usb_phy_bringup` already does at boot (usb.c:151-161), so
this is normally a no-op; it exists so proxy-driven calls can't fault on a
powered-down aperture. This is distinct from the ATCPHY-internal
POWER_CTRL/POWER_STAT state machine (sec 2.2).

### 12.4 Proxy interface

New opcode group in `src/proxy.h`/`src/proxy.c` (chosen to not collide with
the concurrently-developed wireless opcodes; `src/wireless_handoff.c` and
`proxyclient/m1n1/proxy.py` were NOT touched):

| Opcode | Value | Args | Maps to |
|---|---|---|---|
| `P_ATCPHY_APPLY_MODE` | `0x1500` | idx, mode, flipped, allow_pipe_switch, dp_rate_valid, dp_rate | `atcphy_apply_mode()` |
| `P_ATCPHY_SET_ORIENTATION` | `0x1501` | idx, flipped | `atcphy_set_orientation()` |
| `P_ATCPHY_POWER_OFF` | `0x1502` | idx | `atcphy_power_off()` |
| `P_ATCPHY_GET_REG_BASE` | `0x1503` | idx, block (`atcphy_block_t`) | `atcphy_reg_base()` |

`P_ATCPHY_GET_REG_BASE` exists purely so the Python side can cross-check
its own ADT resolution against the C driver's before poking anything (this
project's "verify offsets, don't trust decodes" rule, applied to itself).

### 12.5 Proxyclient module + experiment

- `proxyclient/m1n1/atcphy.py`: `ATCPHY(u, port)` -- window resolution with
  mandatory C-side cross-check, `dump()`/`state()` (read-only decode of
  crossbar/lane_mode/power/misc/pipehandler-mux/usb2phy; never reads the
  dwc3 register file), `set_orientation()`, `apply_mode()`, `power_off()`,
  plus `hpm_status()`/`hpm_orientation()` (SMBus block read of the HPM
  `STATUS` register over `m1n1.hw.i2c`; byte0 bit layout is this project's
  own grade-A hardware measurement from sec 0; the register *number* 0x1A
  is the TI/tipd convention, flagged as not independently re-verified).
- `proxyclient/experiments/atcphy_probe.py`: CLI driver
  (`dump`/`hpm`/`orient`/`usb3`/`off`/`power-off`, default port 2 = the
  broken right-side port). `usb3` refuses to run without
  `--allow-pipe-switch` and is for proxy-only sessions (no guest running),
  matching sec 6's gate.

Because these live in new files and use `M1N1Proxy.request()` generically,
no concurrently-edited file was modified.

### 12.6 What the next hardware session should run (proxy-only, no guest)

1. `python3 proxyclient/experiments/atcphy_probe.py dump` -- capture the
   virgin, never-programmed state of atc-phy2 (this alone answers part of
   sec 8.2's open question about what state the crossbar actually idles in).
2. `... hpm` with a device plugged into the right port -- confirms the
   STATUS read path + orientation decode against the earlier session's
   measurement.
3. `... orient` -- the safe fix candidate; watch the m1n1 console for the
   per-op failure report if any POLL times out (POWER_STAT is the first
   register whose *semantics* this patch bets on -- sec 8.3).
4. `... usb3 --allow-pipe-switch` -- the full USB3 lane/PIPE bring-up, then
   plug a SuperSpeed device and check enumeration from whatever xHCI host
   is later started (or re-dump and record the BIST/lock state).
5. `... power-off` then re-`dump` -- verifies the recovery path so step 4
   can be retried cleanly.
6. Only after 1-5 behave: consider wiring `atcphy_set_orientation(2, ...)`
   into the boot path ahead of a Windows boot, and only then re-litigate
   sec 8's "does this fix the measured CCS=0 bug" question on real data.
