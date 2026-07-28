# windows-native-aic: native AIC passthrough + timer-FIQ reflector

Branch: `windows-native-aic`. Base: `windows` (vGIC-emulation design, `ENABLE_VGIC_MODULE`).

**Status: ACTIVE HARDWARE BRING-UP.** Native AIC2 CONFIG handoff, four-core
startup, timer reflection, and real Fast-IPI sends have all executed on a J414s
M2 Pro. Windows has progressed into storage bring-up, but a successful boot is
not yet claimed. Sections that still describe an unvalidated mechanism are
retained as explicit open questions rather than historical assumptions.

**Toolchain note (this DOES change the starting premise for future work on this
patch):** the task that produced this patch was framed as "cannot be built on this
macOS host, no aarch64 toolchain." That turned out to be wrong for *this* checkout: a
Homebrew LLVM + lld toolchain is present (`brew --prefix llvm`, `brew --prefix lld`),
and `make -j4` in this repo builds `build/m1n1.bin` successfully with
`ENABLE_NATIVE_AIC_PASSTHROUGH` defined -- zero errors. This was cross-checked by
diffing the compiler warning list between a build with the flag on and one with it off:
the *only* differences are three `-Wunused-function` warnings for
`handle_vgic_dist_access`/`handle_vgic_redist_access`/`handle_vgic_its_access` in
`hv_vgic.c`, which is exactly and only the expected consequence of no longer installing
those three `hv_map_hook()` calls. No new warnings, no errors, no change anywhere else.
**This validates C syntax, types, and macro/`#ifdef` correctness only.** It says nothing
about register semantics, timing, or runtime behavior -- those remain entirely
M1-gated, per the checklist at the bottom of this document.

## 1. Before / after architecture

**Before (`windows` branch, `ENABLE_VGIC_MODULE`, no `ENABLE_NATIVE_AIC_PASSTHROUGH`):**

- `HCR_EL2.{IMO,FMO}` both set (`src/hv.c`). Every physical IRQ *and* FIQ traps to EL2.
- `hv_vgicv3_init()` (`src/hv_vgic.c`) allocates and MMIO-hooks (`hv_map_hook()`) a full
  software GICv3 distributor + per-CPU redistributors + a stub ITS at fake addresses
  (`DIST_BASE_*`/`REDIST_BASE_*`/`ITS_BASE_*`). The guest sees a synthetic GICv3, not
  AIC.
- `hv_exc_irq()` (`src/hv_exc.c`) is the physical-IRQ handler: on every trapped AIC IRQ
  it calls `aic_ack()`, then translates the result into a GICv3 list-register injection
  (`hv_vgic3_inject_irq()`) so the guest's (also emulated) GICv3 CPU interface delivers
  it as a virtual IRQ. A UEFI `ArmGicDxe`-class driver, or a generic Windows GIC HAL
  extension, is expected to drive this.
- The timer FIQ is reflected the same way: `hv_update_fiq()` masks the physical
  CNTP/CNTV FIQ source and injects a GICv3 list-register virtual IRQ at a hardcoded
  vINTID (17 for CNTP, 18 for CNTV), falling back to a per-CPU software queue
  (`timer_queue`) when no list register is free, drained later from `hv_exc_irq()`'s
  GICv3-maintenance-interrupt branch.

**After (`ENABLE_NATIVE_AIC_PASSTHROUGH`, this patch):**

- `HCR_EL2.IMO` is cleared; `HCR_EL2.FMO` stays set. Physical AIC IRQs go straight to
  the guest at EL1 with zero EL2 involvement. Physical FIQs (timer, PMU, Fast-IPI) still
  trap to EL2.
- `hv_vgicv3_init()` no longer installs the three `hv_map_hook()` calls. The guest sees
  no synthetic GICv3 at all. It sees the *real* AIC MMIO, which was never hooked by
  anything in this tree in the first place (see &sect;3).
- `hv_exc_irq()` is not expected to fire for ordinary AIC IRQs anymore (that's the whole
  point of clearing IMO). It is left intact rather than deleted, with a new fail-closed
  fallback replacing the old "translate into a vGIC injection" tail, in case that
  assumption is ever wrong on real hardware (see &sect;6, OQ-3).
- The timer FIQ is reflected as an ordinary **per-CPU AIC software-generated IRQ**
  (`aic_set_sw()`), not a GICv3 list-register injection. This is a design pivot from
  the plan this patch started with (see &sect;4) -- superseded mid-implementation by a
  sharper design from the branch owner specifically to close a re-arm/coalescing gap in
  the naive "mask, inject, done" version.
- The GICv3 virtual-CPU-interface / list-register machinery (`hv_vgicv3_enable_virtual_
  interrupts()`, `hv_vgicv3_init_list_registers()`, `hv_vgic3_inject_irq()` and friends)
  is left running, unchanged, per-core. It is no longer used by the timer. It is kept
  only because the pre-existing, vestigial `ICC_SGI1R_EL1` SGI-emulation trap-and-emulate
  (`src/hv_exc.c`, `case SYSREG_ISS(ICC_SGI1R_EL1)`) still uses it, and a genuinely
  native-AIC Windows guest is not expected to exercise that path at all (see &sect;5).

## 2. HCR_EL2 change

`src/hv.c`, `hv_init()`, primary site (secondary cores inherit this value verbatim via
`hv_secondary_info.hcr` / `hv_init_secondary()`, see the comments at both call sites):

```
HCR_API | HCR_APK | HCR_TEA | HCR_RW | HCR_TSC | HCR_TID3 | HCR_AMO |
/* HCR_IMO intentionally omitted */
HCR_FMO | HCR_VM
```

Bit facts, cited from `src/arm_cpu_regs.h`:
- `HCR_IMO = BIT(4)` (line 177) -- routes physical Group 1 IRQ to EL2 when set.
- `HCR_FMO = BIT(3)` (line 178) -- routes physical FIQ to EL2 when set.
- `HCR_AMO = BIT(5)` (line 176) -- routes physical SError to EL2 (unchanged, stays set).

Clearing IMO is the entire mechanism by which "the guest drives AIC directly" is
achieved: it is a single bit, and nothing else in the patch does the routing work --
everything else (unhooking the vGIC MMIO, the AIC-IRQ fail-closed fallback) exists to
make sure the *rest* of the system is consistent with that one bit being clear.
`HCR_TID3` is untouched (still traps ID-group-3 registers so `ID_AA64PFR0_EL1` can be
faked for GICv3-CPU-interface detection, see the comment at
`src/hv_exc.c`'s `case SYSREG_ISS(ID_AA64PFR0_EL1)`).

## 3. AIC MMIO passthrough: nothing to change

Item 2 of the originating task asked for "make sure the real AIC MMIO is left mapped
straight through to the guest (no hooks)." This is already true, unconditionally, and
required no code change:

- The only place in this tree that ever calls `hv_map_hook()` against AIC's own MMIO
  base is `src/hv_aic.c`'s `hv_trace_irq()` (an opt-in host-debugger IRQ tracer), which
  is only ever invoked from `src/proxy.c:502` in response to an explicit proxy command
  from the *host* debug tool -- never from the boot path, never automatically.
- The vGIC distributor/redistributor/ITS hooks (`src/hv_vgic.c`, now gated out under
  this flag) were installed at entirely separate, synthetic addresses (`DIST_BASE_*`
  etc.) chosen to be "unoccupied MMIO space" -- they never targeted AIC's real MMIO
  region to begin with, in either the old or new design.
- Guest access to physical MMIO that has no explicit `hv_map_hook()`/`hv_map_sw()`
  falls through to whatever bulk stage-2 identity mapping the *host-side* boot tooling
  (the Python proxyclient driving `hv_map`/`hv_map_hw`, `src/proxy.c:479`) established
  before `hv_start()` was called -- that is outside this repository's C source and
  outside this patch's scope.

## 4. Timer-FIQ reflector: design decision and re-arm handshake

### 4.1 What changed mid-implementation

The task that produced the first draft of this patch specified: "keep the GICv3
*virtual CPU interface* alive for ONLY the timer PPI (inject via `ICH_LR<n>_EL2`) ...
and flag the alternative (fully AIC-native timer delivery) as an open question." That
was the initial plan and was partially implemented (reusing `hv_vgic3_inject_irq()`
with vINTID 17/18) before the branch owner sharpened the requirement mid-task: the
reflector must not "fire-and-return" -- it needs an explicit per-CPU mask/pending/re-arm
handshake with proper coalescing, and delivery must be via a **reserved per-CPU AIC
software IRQ**, exposed to Windows as an ordinary clock vector by its AIC HAL
extension, not a GIC PPI. This document reflects the *final* implemented design (AIC
software IRQ), not the initial GIC-hybrid plan; the GIC-hybrid alternative and why it
was dropped is discussed in &sect;4.5.

This turned out to also resolve several problems the GIC-hybrid design would have had
(discovered while implementing it, see &sect;4.5), so the pivot is a net simplification,
not just a change of taste.

### 4.2 The handshake, per physical-timer source (CNTP and CNTV, independently, per-CPU)

Implemented in `hv_update_fiq()` (`src/hv_exc.c`), called from every EL2 exception exit
(`hv_exc_exit()`, itself called from the sync-trap fast and slow paths, `hv_exc_fiq()`,
and `hv_exc_serr()`):

1. **Physical timer expires -> FIQ to EL2.** (Already true before this patch;
   `HCR_EL2.FMO` stays set specifically so this keeps happening -- see &sect;2.)
2. **EL2 masks/suppresses that timer source.** `reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
   VM_TMR_FIQ_ENA_ENA_P)` (or `_ENA_V` for CNTV). Register cited at
   `src/cpu_regs.h:736-738`: `SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2 = sys_reg(3,5,15,1,3)`,
   `VM_TMR_FIQ_ENA_ENA_P = BIT(1)`, `VM_TMR_FIQ_ENA_ENA_V = BIT(0)`. This exact
   primitive already existed in the pre-`windows-native-aic` code (it briefly masked the
   source ahead of a `// TODO: proper injection`); it is now load-bearing, not a
   stopgap -- this is precisely the step that prevents the failure mode the branch
   owner called out: "if the architectural timer stays asserted, EL2 takes another FIQ
   before EL1 services the reflected IRQ."
3. **EL2 atomically marks `TIMER_REFLECTION_PENDING` (per-CPU).** Implemented as
   `PERCPU(timer_p_reflection_pending)` / `PERCPU(timer_v_reflection_pending)` in the new
   `struct hv_pcpu_data` fields (`src/hv_exc.c`). "Atomically" here means "as a plain
   read-modify-write" -- these fields, like the pre-existing `ipi_pending`/`pmc_pending`
   in the same struct, are only ever touched by the *owning* core's own EL2 code path,
   so there is no concurrent writer to race with; no `__atomic_*` builtin is used, matching
   the existing style of that struct.
4. **EL2 triggers a RESERVED per-CPU AIC software IRQ.** `aic_set_sw(HV_TIMER_P_SWIRQ(smp_id()),
   true)` (or `HV_TIMER_V_SWIRQ()`). `aic_set_sw()` is m1n1's own wrapper (`src/aic.c:192-202`)
   around AIC's `AIC_SW_SET`/`AIC_SW_CLR` registers (`src/aic_regs.h:12-13`); a software-posted
   event on IRQ number N goes through the *identical* ack/mask/priority/affinity machinery as a
   real HW interrupt on that same number (`aic_ack()` returns it via `AIC_EVENT` with
   `AIC_EVENT_TYPE_HW`, `aic_regs.h:50` -- there is no separate "type" for software-triggered vs.
   hardware-triggered once posted). Because `HCR_EL2.IMO` is clear, this reaches the guest at EL1
   directly, exactly like a real peripheral IRQ, which is what makes it FIQ-bugcheck-safe.
5. **EL1 handles the ordinary Windows clock vector; Windows programs the next
   deadline.** Outside m1n1's control -- this is the guest's own AppleAic-equivalent HAL
   extension's ordinary interrupt-servicing + timer-reprogramming path.
6. **Timer is unmasked.** See &sect;4.3 -- the chosen design is PASS-THROUGH.

The per-source counters `PERCPU(timer_p_fiq_count)` / `PERCPU(timer_v_fiq_count)` are a
monotonically-incrementing "expiration counter" (never decremented by EL2), incremented
on every observed physical expiration regardless of whether a reflection is already
outstanding. See &sect;4.4 for why this exists and what it does and does not guarantee.

### 4.3 Re-arm design: PASS-THROUGH chosen, TRAPPED/COOPERATIVE documented as the alternative

The two options as framed by the branch owner:

1. **PASS-THROUGH**: EL1 keeps direct timer-register access; EL2 sets the mask on FIQ
   entry; rely on the normal Windows timer-programming sequence to clear the mask while
   setting the next deadline. Thinnest, but only correct if Windows reliably writes the
   control register on every reprogramming path.
2. **TRAPPED/COOPERATIVE**: the timer HAL path uses an HVC (or the timer-register
   accesses are trapped) so EL2 explicitly programs + unmasks after receiving the next
   deadline. Deterministic ownership, more invasive.

**Implemented: PASS-THROUGH**, via the `else` branch in `hv_update_fiq()`'s per-source
handling: on every EL2 exit, re-read the live `CNTx_CTL_EL02` register; if it no longer
reads ISTATUS-asserted, re-set `VM_TMR_FIQ_ENA_ENA_{P,V}` and clear
`*_reflection_pending`. This is not a fixed-interval poll -- `hv_update_fiq()` runs on
*every* EL2 exit (every trapped sysreg access, every FIQ, every SError), so in practice
the re-arm check happens far more often than once per guest timer period.

Why PASS-THROUGH and not TRAPPED/COOPERATIVE, concretely:

- `mrs(CNTP_CTL_EL02)`/`mrs(CNTV_CTL_EL02)` are read **unconditionally** at this exact
  call site in the pre-existing (pre-`windows-native-aic`) code, regardless of whether
  `hv_has_ecv` is true or false (`src/hv.c:127-140`, the FEAT_ECV detection). That means
  this read is already relied upon, by code that predates this patch, to reflect live
  timer truth in both configurations:
  - **ECV supported** (`hv_has_ecv == true`, `CNTHCTL_EL1NVPCT`/`CNTHCTL_EL1TVT` set):
    the guest's accesses to the architectural `CNTP_CTL_EL0`/`CNTP_CVAL_EL0`/`CNTP_TVAL_EL0`
    registers are *already* trapped and relayed synchronously
    (`SYSREG_MAP(SYS_CNTP_CTL_EL0, SYS_CNTP_CTL_EL02)` etc. in `hv_handle_msr_unlocked()`,
    unconditional, not gated by this flag). Every one of those trapped writes exits
    through `hv_exc_sync`'s fast path, which calls `hv_update_fiq()` right there -- so in
    this mode, PASS-THROUGH's re-check happens *synchronously, on the reprogramming
    write itself*. This is effectively as good as TRAPPED/COOPERATIVE, for free, without
    a second code path.
  - **ECV not supported** (`hv_has_ecv == false`): `CNTHCTL_EL1PTEN` grants EL1 direct,
    untrapped access to the real physical timer registers (`src/hv.c:136-137`). The
    guest's reprogramming write does **not** trap at all here. Re-arm can only happen
    the next time `hv_update_fiq()` runs for some *other* reason. This is the genuinely
    thinner case the branch owner flagged, and it is **not resolved by this patch** --
    see the M1-VALIDATION CHECKLIST, item T3.

Given the ECV case gets TRAPPED/COOPERATIVE-equivalent behavior "for free" through the
existing SYSREG_MAP trap-and-emulate, and the non-ECV case is the only one where the
distinction matters, this patch does not add a second, ECV-independent trapping
mechanism (e.g. forcing `CNTHCTL_EL1PTEN = 0` unconditionally to make *all* hardware
always take the TRAPPED/COOPERATIVE path). That would be a more invasive change
touching timer behavior for every guest OS this hypervisor supports, not just Windows,
and is left as a documented fallback if M1 tracing (checklist item T3) shows the
non-ECV pass-through re-arm is not prompt enough in practice.

### 4.4 Coalescing (required, not a boolean)

Per-CPU, per-source `u64 timer_{p,v}_fiq_count` in `struct hv_pcpu_data`
(`src/hv_exc.c`) is a monotonic count of "how many times EL2 has observed and masked a
new physical expiration," incremented on every detected expiration regardless of
whether a reflection is already outstanding.

What this counter does **not** do: it is not delivered to the guest, and the guest is
never told "there were N expirations, not 1." AIC's software-pending bit
(`AIC_SW_SET`/`AIC_SW_CLR`) is a **level**, not a FIFO slot -- a second `aic_set_sw(irq,
true)` call while the first is still un-acked is a harmless no-op at the hardware level,
and this patch deliberately does not even issue that redundant call (gated by
`*_reflection_pending`). So if two physical expirations occur before the guest services
the first reflected IRQ, the guest observes **one** AIC interrupt, not two.

This is safe **conditionally**, not by construction: it relies on Windows' clock
interrupt service routine recomputing elapsed/next-deadline time from the live counter
register (`CNTPCT_EL0`) rather than assuming "exactly one tick has elapsed" per
interrupt taken -- i.e. the "recompute from the counter and compare value" rule the
branch owner offered as the alternative to literal per-tick counting. This is standard
practice for virtualized/NTP-disciplined OS timekeeping and is architecturally how
`CNTx_CTL_EL0.ISTATUS` itself works (a live comparison result, not a latched edge-count),
but **whether Windows' specific AppleAic-driven clock ISR behaves this way is
unconfirmed** -- it is the single most important thing the first M1 trace needs to
establish (checklist item T1/T2). The `timer_{p,v}_fiq_count` counters exist specifically
so that trace can compare "how many physical expirations actually happened" against
"how many ticks Windows' timekeeping believes occurred," to catch silent time loss if
the recompute assumption turns out to be false.

### 4.5 Why the GIC-list-register design was dropped

Implementing the original GIC-hybrid plan (inject the timer via `ICH_LR<n>_EL2`, vINTID
17/18, reusing `hv_vgic3_inject_irq()`) surfaced three concrete problems specific to
combining it with `HCR_EL2.IMO = 0`:

1. **The GICv3 virtual-CPU-interface maintenance interrupt's routing relative to
   `HCR_EL2.IMO` on Apple Silicon is unverified** (see OQ-3, &sect;6). On textbook ARM
   systems the maintenance interrupt is architecturally a physical interrupt subject to
   the same `IMO`/`FMO` routing as any other -- clearing IMO could mean it stops
   trapping to EL2 at all, which would silently break `hv_exc_irq()`'s
   list-register-cleanup housekeeping (the `type == 0` branch). This project's own
   existing code has a bare `//maintenance IRQ?` with a question mark at that exact
   check (`src/hv_exc.c`), i.e. even the original author of that logic wasn't certain.
2. **`timer_queue` (the LR-injection overflow path) had exactly one drain point**
   (`hv_exc_irq()`'s maintenance-interrupt branch), which is the same code whose
   liveness OQ-3 puts in question. If physical IRQ (and, possibly, the maintenance
   interrupt with it) genuinely stops trapping to EL2, an overflowed timer virq could be
   stranded forever with no other retry point in the old design.
3. **The vINTID numbers (17, 18) don't match SBSA convention** (PPI 30/27 for
   CNTP/CNTV), which is fine only if a to-be-written GTDT explicitly matches them --
   another moving part to get right and keep in sync, entirely avoided once the timer
   is delivered as an ordinary AIC IRQ described the same way as every other AIC
   interrupt.

The AIC-software-IRQ design sidesteps all three: it does not touch list registers, does
not depend on the maintenance interrupt at all, and describes the timer to ACPI the
same uniform way as every other AIC-routed interrupt (see &sect;5). The GICv3
virtual-CPU-interface machinery (`hv_vgicv3_enable_virtual_interrupts()`,
`hv_vgic3_inject_irq()`, etc.) is left in the tree, unused by the timer, only because
the pre-existing `ICC_SGI1R_EL1` SGI emulation still references it (see &sect;7).

## 5. ACPI implication (revised for the AIC-software-IRQ design)

The original task framing (before the mid-task pivot) called for a hybrid ACPI
description: CSRT for AIC, plus a minimal MADT/GTDT describing the timer as a GIC PPI.
**That is no longer the design.** With the timer delivered as an ordinary AIC software
IRQ:

- **Peripherals**, including the reflected timer, are described uniformly as AIC-routed
  interrupts -- whatever ACPI mechanism (CSRT and/or a native-AIC-aware MADT-equivalent)
  describes AIC to Windows' native AIC HAL extension should describe the reserved
  per-CPU timer software IRQ numbers (`HV_TIMER_P_SWIRQ(cpu)`/`HV_TIMER_V_SWIRQ(cpu)`,
  `src/hv_exc.c`) the same way it describes any other AIC IRQ.
- **GTDT** (Generic Timer Description Table) conventionally carries a `GSIV` field
  identifying the architectural timer's interrupt, normally expected to be a GIC PPI
  number in the 16-31 range on a standard ARM platform. Under this design, that field
  would need to hold the reserved AIC software IRQ number instead -- **whether Windows'
  ACPI/HAL layer accepts and correctly routes a GTDT GSIV that is not in the
  conventional GIC-PPI range, resolving it through whichever interrupt controller HAL
  extension currently owns that GSIV (AppleAic, not a GIC), is UNVERIFIED.** This is a
  real open question about Windows HAL internals, not a register fact this codebase can
  settle -- see OQ-4.
- No GIC-describing ACPI structures (minimal MADT GICC/GICD/GICR entries) should be
  needed at all in the fully-AIC-native design, since the guest never touches the GICv3
  CPU interface for anything Windows is expected to use.

## 6. PMU / Fast-IPI FIQ handling (item 4)

Both are FIQ-class physical interrupt sources that keep trapping to EL2 because
`HCR_EL2.FMO` stays set (they are not fixed by clearing `IMO`, unlike ordinary AIC
IRQs). PMU remains fail-closed. Fast IPI now has a hardware-tested native-AIC
transport described below.

**PMU** (`src/hv_exc.c`, `hv_exc_fiq()`, the `SYS_IMP_APL_PMCR0` handling): unchanged.
The physical source is masked (IACT + IMODE cleared) on FIQ, and
`PERCPU(pmc_pending)` is only ever exposed to the guest via the pre-existing trapped
read of `SYS_IMP_APL_PMCR0` -- nothing wakes the guest to look at it, before or after
this patch. Noted as a low-risk future option, not implemented: `cpu_regs.h:461`
defines `PMCR0_IMODE_AIC` (route the PMU interrupt through AIC as an ordinary IRQ)
alongside `PMCR0_IMODE_FIQ`; nothing in this codebase currently requests `IMODE_AIC`.
Switching the guest-facing `SYS_PMCR_EL0` emulation to request it would let PMU
interrupts bypass EL2 the same way ordinary peripherals do -- unvalidated, out of scope
for this pass.

**Fast-IPI** (`SYS_IMP_APL_IPI_RR_LOCAL_EL1`/`IPI_RR_GLOBAL_EL1`/`IPI_SR_EL1`):
CPU-local, bypasses AIC entirely, and is also used by m1n1's own EL2 rendezvous.
Guest RR writes are trapped, tagged before the physical send, and relayed as real
Fast IPIs. On the target, EL2 acknowledges the physical edge and turns only a
guest-tagged arrival into a synthetic AIC `EVENT_TYPE_IPI` wake driven by
`HCR.IMO|HCR.VI`.

The synthetic wake is transactional. `PERCPU(ipi_pending)` contains distinct
`DELIVERABLE` and `INFLIGHT` generations. Reading EVENT moves one generation to
`INFLIGHT`; it does not retire it. The Windows AIC HAL keeps the corresponding
IPI class active until its controller EOI and writes trapped `IPI_SR_EL1` there.
That write commits only `INFLIGHT`, preserving a newer `DELIVERABLE` send. Each
new transaction places a seven-bit generation in EVENT bits 30:24. EVENT bit 31
marks real NT IPI work migrated from the GIC startup carrier; the HAL dispatches
that classless origin as class 0, while explicitly retiring an ordinary
classless redundant Fast-IPI edge as spurious. Retries reuse the exact raw
token, so the HAL's comparison rejects an EOI delayed from an older Active
lifetime. If no EOI arrives, the local architectural-counter clock
rate-limits retries to at least 250 ms apart and continues until completion;
the diagnostic retry count saturates instead of abandoning the transaction.
This closes the observed failure in which NT retained its KPCR pending-vector
bit and KPRCB request node after HAL and m1n1 had both destructively cleared
their wake state.

## 7. What was disabled vs. kept

**Disabled** (gated behind `#ifndef ENABLE_NATIVE_AIC_PASSTHROUGH` / `#ifdef`, not
deleted):
- `HCR_EL2.IMO` (`src/hv.c`).
- The three `hv_map_hook()` calls in `hv_vgicv3_init()` for the GICD/GICR/ITS synthetic
  MMIO regions (`src/hv_vgic.c`).
- `hv_exc_irq()`'s translation of a real AIC HW/IPI event into a `hv_vgic3_inject_irq()`
  call (`src/hv_exc.c`) -- replaced with a fail-closed log-and-mask fallback.
- The GICv3 list-register-based timer reflection in `hv_update_fiq()` (both the
  `hv_vgic3_inject_irq()` call and the `timer_queue` overflow push) -- replaced with the
  AIC-software-IRQ handshake.

**Kept, unchanged:**
- `HCR_EL2.FMO`, `HCR_EL2.TID3`.
- `hv_vgicv3_init()`'s in-memory distributor/redistributor/ITS struct
  allocation+initialization (now unused, kept to minimize diff).
- `hv_vgicv3_enable_virtual_interrupts()` / `hv_vgicv3_init_list_registers()` (per-core
  vCPU-interface enablement) -- still called from `hv_start()`/`hv_init_secondary()`,
  no longer used by the timer, kept for the vestigial `ICC_SGI1R_EL1` path.
- The `ICC_SGI1R_EL1` SGI trap-and-emulate (`src/hv_exc.c`) and its
  `sgi_queue`/list-register drain in `hv_exc_irq()`'s maintenance branch and in
  `hv_exc_fiq()`'s Fast-IPI-arrival handling -- unchanged, now understood to be
  vestigial for a genuinely-native-AIC guest (see &sect;6).
- `ID_AA64PFR0_EL1`'s GICv3-CPU-interface-present bit forcing, and the rest of the
  `HCR_TID3` ID-register pass-through table (`src/hv_exc.c`) -- unchanged; whether
  anything in the native-AIC boot path still needs it is OQ-5.
- All PMU virtualization (`SYS_PMCR_EL0`, `PMCNTEN*`, `PMOVS*`, etc.) -- entirely
  untouched by this patch.

**New:**
- `struct hv_pcpu_data`: `timer_p_fiq_count`, `timer_v_fiq_count`,
  `timer_p_reflection_pending`, `timer_v_reflection_pending` (`src/hv_exc.c`).
- `HV_TIMER_SWIRQ_BASE`/`HV_TIMER_P_SWIRQ()`/`HV_TIMER_V_SWIRQ()` macros
  (`src/hv_exc.c`).
- `hv_timer_reflect_init()` (`src/hv_exc.c`, declared in `src/hv.h`), called once from
  `hv_init()` (`src/hv.c`).
- The fail-closed fallback in `hv_exc_irq()` (`src/hv_exc.c`).

## 8. Open questions

- **OQ-1 (placement):** `HV_TIMER_SWIRQ_BASE = aic->nr_irq - (2 * MAX_CPUS)` reserves
  the topmost `2 * MAX_CPUS` (48) implemented AIC HW IRQ numbers for the timer reflector. This is a
  common convention (real peripherals are enumerated from the ADT starting near the
  bottom) but has **not** been cross-checked against any specific chip's actual
  interrupt map. Confirm nothing else claims these numbers before trusting this on real
  hardware.
- **OQ-1b (affinity, likely more serious than OQ-1):** `aic_set_affinity()`
  (`src/aic.c:216-219`) is a **no-op unless `aic->version == 1`**. `aic->version` comes
  from the ADT `"aic,N"` compatible string (`src/aic.c:159-168`) and is chip-dependent;
  every chip this project actually targets beyond the M1 base (T8103/T8112, AIC1) --
  M1 Pro/Max/Ultra, M2 Pro/Max/Ultra, M5 Pro, etc. -- uses AIC2 or AIC3, where per-IRQ
  target routing lives in a differently-encoded `AIC23_IRQ_CFG_TARGET` field
  (`src/aic_regs.h:28`) whose value semantics are **not documented anywhere in this
  tree** and were not reverse-engineered for this patch (guessing them would violate
  the "never guess a register/bit" rule this task was given under). Concretely: on
  AIC2/AIC3 hardware, `hv_timer_reflect_init()`'s `aic_set_affinity()` calls currently do
  nothing, and the reserved per-CPU timer SW IRQs are left at whatever AIC's own
  default routing is for a freshly-claimed IRQ number -- **unknown, and not validated to
  land on the expected core.** This is the single most likely "the timer reflector
  doesn't actually work correctly" failure mode on real target hardware and should be
  the first thing an M1 trace checks (checklist item T4).
- **OQ-2 (transactional Fast IPI, &sect;6):** confirm on hardware that every emitted
  synthetic EVENT is followed by a HAL EOI commit during clean SMP startup and
  that the bounded retry counter remains zero. A nonzero retry is recovery
  evidence and must be correlated with NT's KPCR+0xCC and KPRCB request queue.
- **OQ-3 (maintenance interrupt routing, &sect;4.5 point 1):** does the GICv3
  virtual-CPU-interface maintenance interrupt trap to EL2 independent of
  `HCR_EL2.IMO`, or does it share the same physical-IRQ routing gate as ordinary AIC
  IRQs on Apple Silicon specifically? This determines whether `hv_exc_irq()` (the
  physical-IRQ vector) is ever entered again at all under this patch, and thus whether
  its fail-closed fallback (&sect;7) is dead code or a real safety net.
- **OQ-4 (GTDT GSIV outside the GIC-PPI range, &sect;5):** does Windows' ACPI/HAL layer
  accept a GTDT `GSIV` field that resolves through a non-GIC interrupt controller HAL
  extension (AppleAic) rather than a literal GIC PPI? This is a Windows-HAL-internals
  question, not something this codebase can answer; cross-reference
  `woa-hal-interrupt-re.md`/`native-aic-hal-re.md` in the drivers RE repo and confirm on
  real hardware.
- **OQ-5 (`ID_AA64PFR0_EL1`.GIC relevance):** is forcing the "GICv3 CPU interface
  present" bit still necessary for anything in a fully-AIC-native Windows boot path, now
  that the timer is no longer GIC-PPI-delivered? Left unchanged rather than guessed at
  either way (see `src/hv_exc.c`, `case SYSREG_ISS(ID_AA64PFR0_EL1)`).

## 9. J414S HARDWARE BRING-UP STATUS (2026-07-26)

The path has now run on an M2 Pro Mac14,9 (`apple,j414s`) with the current m1n1
RAM-chainloaded over the older resident proxy.  Observed hardware evidence:

- Mu starts with native-AIC passthrough active and reaches Windows Boot Manager.
- After waking the correct HPM and handing USB1's PHY/controller to the guest in
  host mode, Mu enumerates a Satechi NVMe enclosure as high-speed USB mass storage,
  validates its GPT/FAT ESP, and loads `EFI/BOOT/BOOTAA64.EFI` and `bootmgfw.efi`.
  SuperSpeed operation is not yet proven; the successful enumeration is USB2.
- Windows reaches its kernel transition and executes PMUv3/PSCI and feature-register
  probes.  An EL2 undefined exception caused by operandless `TLBI VMALLE1OS` was
  fixed by explicitly issuing `TLBI VMALLE1IS` with the reserved `XZR` operand.
- The next captured failure was Windows recovery status `0xc000000d`, "Fatal error
  transitioning to the operating system."  Mu simultaneously rejected
  `ExitBootServices()` because runtime descriptors were not 64 KiB aligned.  The raw
  guest load address and T602x runtime code/data bins have been corrected; live
  verification of that correction is pending the next powered target run.

The checklist below therefore distinguishes what the hardware run has already
established from the remaining native-AIC correctness work.  Reaching the kernel is
not evidence that timer reflection, per-CPU affinity, IPIs, or peripheral interrupts
are fully correct.

### 2026-07-27 checkpoint

Later J414s runs advanced substantially beyond the first recovery failure:

- Keeping the x18/KPCR repair active after the AIC2 `CONFIG` handoff eliminated the
  repeatable `IRQL_NOT_LESS_OR_EQUAL (0xA)` at `KfRaiseIrql+4`. A four-E-core control
  configuration then ran for minutes with the Windows logo/spinner and no bugcheck.
- Windows can populate `TPIDR_EL1` before both of its exception stacks are usable.
  The carrier now requires writable `PanicStackBase` and `InterruptStackBase` ranges
  before delivering an SGI. This allows the first Avalanche core (logical CPU 4) to
  acknowledge and EOI its startup INTID 0; the earlier gate on only the panic stack
  entered `KxSwitchStackAndPlayInterrupt` with an unsafe interrupt stack.
- A software pending/active carrier is required for the short GIC compatibility
  window. The hardware ICH LR path worked on Blizzard but left the same interrupt
  pending indefinitely on Avalanche. HCR.VI now asserts only while a software queue
  has a priority-eligible, non-active Group-1 entry. The first Windows AIC2 `CONFIG`
  enable still removes this carrier permanently; normal operation remains native AIC.
- CPU4 receives exactly one startup SGI from CPU0, returns INTID 0 from IAR, and EOIs
  it. At EOI the SGI queue is empty and the coalesced count is zero. Windows sends no
  second SGI to CPU4, so the remaining stall is not a lost carrier notification.
- Normalizing `VPIDR_EL2` so Avalanche reports the Blizzard MIDR was tested and made
  no behavioral difference. All other trapped architectural feature registers were
  already identical across the two core types; that experiment was removed.
- A 5 kHz EL2 sampler found CPU4 alive after EOI with stable x18/SP, looping at kernel
  offsets `0x4fc4`, `0x13a88`, and `0x13a98` until the BSP times out and tears down the
  temporary KPCR mapping. The next hardware step is to capture/disassemble this loop
  and identify its waited-on value. The sampler itself was diagnostic-only and is not
  retained in the checkpoint commit.

The last session ended with stale USB CDC device nodes: neither proxy endpoint
answered NOPs after a chainload re-enumeration race. A physical cable replug or proxy
restart is required before that next capture; this is an external test-access blocker,
not a new guest failure.

## 10. M1-VALIDATION CHECKLIST

Most of the interrupt-correctness analysis above remains design reasoning from m1n1's
source plus the RE docs in `apple_silicon_nt_drivers/docs/`.  The limited J414s evidence
in section 9 does not satisfy the timer/IPI/peripheral acceptance tests below.

**T0 -- sanity / does it boot at all**
- [ ] Boots a Windows guest (or, as a cheaper first step, m1n1's own test harness /a
      minimal EL1 payload that just spins reading `PMCR_EL0`/`CNTPCT_EL0`) under
      `hv_start()` with `ENABLE_NATIVE_AIC_PASSTHROUGH` defined, without an immediate
      SError/panic from `hv_exc_serr()`/`hv_panic()`.
- [ ] No FIQ-class exception is ever observed at EL1 in the guest (confirms
      `HCR_EL2.FMO` masking + AIC-software-IRQ reflection is actually working, and that
      nothing else is leaking a raw FIQ through) -- this is the literal "no FIQ
      bugcheck" acceptance criterion from the originating task.

**T1 -- timer tick arrives, at all**
- [ ] The guest's clock ISR (on the reserved `HV_TIMER_P_SWIRQ(cpu)`/
      `HV_TIMER_V_SWIRQ(cpu)` AIC IRQ number(s)) fires at least once per CPU.
- [ ] `PERCPU(timer_p_fiq_count)`/`PERCPU(timer_v_fiq_count)` (add a debug dump path,
      e.g. via the m1n1 proxy console) increments at the expected physical timer
      frequency.

**T2 -- the FIRST M1 trace, required before trusting PASS-THROUGH re-arm (&sect;4.3)**
- [ ] Record **every** guest system-register write to `CNTP_CTL_EL0`/`CNTP_CVAL_EL0`/
      `CNTP_TVAL_EL0` (and the `CNTV_*` equivalents) around a single clock interrupt,
      from FIQ-entry to the guest's next `WFI`/idle. Determine: (a) does Windows always
      rewrite the control/compare register on every reprogramming path, or are there
      optimized paths that only touch e.g. `TVAL`; (b) is there any path where Windows
      leaves the timer disabled/masked from the guest's perspective without an
      intervening m1n1 EL2 entry to notice via `hv_update_fiq()`'s poll.
- [ ] Cross-check against `hv_has_ecv` (`src/hv.c:127`) on the actual target chip: is
      ECV support detected, and does that match the ECV-vs-non-ECV re-arm-timing
      analysis in &sect;4.3?
- [ ] If T2 shows the non-ECV pass-through re-arm is not prompt enough (guest goes idle
      after reprogramming without another EL2 entry before the new deadline), the
      documented fallback is: force `CNTHCTL_EL1PTEN = 0` unconditionally (make the ECV
      trapped-and-emulated path the only path, regardless of hardware ECV support) --
      i.e. retroactively adopt TRAPPED/COOPERATIVE. Not implemented in this patch.

**T3 -- coalescing correctness (&sect;4.4)**
- [ ] Deliberately induce a second physical expiration while a reflection is still
      outstanding (e.g. by holding the guest off-core briefly via the m1n1 debugger).
      Confirm `timer_{p,v}_fiq_count` advanced by 2 (or more) while the guest still only
      observed one AIC interrupt, AND confirm Windows' timekeeping does not lose the
      elapsed time (i.e. the "recompute from the live counter" assumption in &sect;4.4
      holds). If it does not hold, this design needs a way to communicate the count to
      the guest -- not implemented here.

**T4 -- per-CPU affinity (OQ-1b, likely the highest-risk item in this whole patch)**
- [ ] On the actual target chip, determine `aic->version` (`src/aic.c`, printed at AIC
      init: `"AIC: Version %d @ ..."`). If it is 2 or 3 (expected for anything newer
      than M1 base), confirm via tracing/logging which physical CPU actually receives
      the reflected timer SW IRQ for a given source CPU's expiration. If it does not
      match, `aic_set_affinity()` needs a real AIC2/AIC3 implementation (requires
      reverse-engineering `AIC23_IRQ_CFG_TARGET`'s value encoding, `aic_regs.h:28` --
      not done in this patch) before this design is correct on that hardware.

**T5 -- IPI path**
- [x] Confirm the hardware HAL executes Apple Fast-IPI RR instructions and that
      tagged arrivals become native-AIC synthetic EVENTs on all four E cores.
- [ ] Validate the transactional EOI build with no debugger mutation. Capture
      per-CPU send/tag/EVENT/commit counters plus `DELIVERABLE`, `INFLIGHT`, and
      retry count. Clean startup requires NT's vector-pending bit and request
      queue to drain without a manual `pcpu[].ipi_pending` write.

**T6 -- maintenance-interrupt routing (OQ-3)**
- [ ] Confirm whether `hv_exc_irq()` (the physical-IRQ vector, `src/hv_exc.c`) is ever
      entered at all once the guest is running with `HCR_EL2.IMO = 0`. If it is, capture
      what triggered it (the new fail-closed fallback logs loudly -- see &sect;7) and
      determine whether it was a genuine AIC HW/IPI event (meaning the IMO=0 assumption
      is violated somewhere) or the GICv3 maintenance interrupt (meaning OQ-3 resolves
      to "still routes independent of IMO," which is only relevant to the now-vestigial
      SGI path, &sect;7, and otherwise harmless).

**T7 -- peripheral IRQ passthrough (the actual point of this whole patch)**
- [ ] Confirm ordinary AIC-routed peripheral IRQs (anything other than the timer/IPI)
      reach the guest directly with no EL2 involvement / no measurable added latency
      versus bare metal, and that Windows' native AIC HAL extension can mask/unmask/
      ack/EOI them via AIC's real MMIO without faulting.
