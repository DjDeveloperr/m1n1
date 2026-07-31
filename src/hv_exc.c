/* SPDX-License-Identifier: MIT */

#include "hv.h"
#include "assert.h"
#include "cpu_regs.h"
#include "exception.h"
#include "smp.h"
#include "string.h"
#include "uart.h"
#include "uartproxy.h"
#include "utils.h"
#include "hv_vgic.h"
#include "aic.h"
#include "aic_regs.h"
#include "adt.h"

#define TIME_ACCOUNTING
//
// m1n1_windows change: when the vGIC is running in the guest - timer interrupts by virtue of coming from the generic timer are
// still going to come as FIQs to EL2 - we'll need to divert those to the guest as *IRQs* (to prevent Windows from crashing as it treats FIQs as
// errors).
//
extern bool vgic_inited;
extern spinlock_t bhl;

#define _SYSREG_ISS(_1, _2, op0, op1, CRn, CRm, op2)                                               \
    (((op0) << ESR_ISS_MSR_OP0_SHIFT) | ((op1) << ESR_ISS_MSR_OP1_SHIFT) |                         \
     ((CRn) << ESR_ISS_MSR_CRn_SHIFT) | ((CRm) << ESR_ISS_MSR_CRm_SHIFT) |                         \
     ((op2) << ESR_ISS_MSR_OP2_SHIFT))
#define SYSREG_ISS(...) _SYSREG_ISS(__VA_ARGS__)

#define PERCPU(x) pcpu[mrs(TPIDR_EL2)].x
#define PERCPU_N(x, y) pcpu[x].y

/*
 * Guest Fast-IPI transport state.  DELIVERABLE is also used by the legacy
 * vGIC path, so these common state bits must remain available when native AIC
 * passthrough is compiled out.
 *
 * Under native AIC, the physical Fast-IPI latch is only a wake edge; it is
 * not proof that Windows reached KiIpiServiceRoutine. EVENT accept moves a
 * generation to INFLIGHT, and the HAL's controller EOI writes IPI_SR_EL1 to
 * commit it. DELIVERABLE may coexist with INFLIGHT when a newer send arrives.
 */
#define HV_GUEST_IPI_DELIVERABLE      BIT(0)
#define HV_GUEST_IPI_INFLIGHT         BIT(1)
#define HV_GUEST_IPI_RETRY_ARMED      BIT(2)
#define HV_GUEST_IPI_GENERATION       GENMASK(9, 3)
#define HV_GUEST_IPI_INFLIGHT_CARRIER BIT(10)
#define HV_GUEST_IPI_EVENT_TOKEN      GENMASK(10, 3)
#define HV_GUEST_IPI_RETRY_COUNT      GENMASK(14, 11)
#define HV_GUEST_IPI_START_TIME       GENMASK(30, 15)
#define HV_GUEST_IPI_QUEUED_CARRIER   BIT(31)
#define HV_GUEST_IPI_OUTSTANDING      (HV_GUEST_IPI_DELIVERABLE | HV_GUEST_IPI_INFLIGHT)
#define HV_GUEST_IPI_RETRY_COUNT_MAX  MASK(4)
#define HV_GUEST_IPI_CLOCK_SHIFT      18
#define HV_GUEST_IPI_CLOCK_MASK       MASK(16)

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
//
// windows-native-aic timer-FIQ reflector: reserved per-CPU AIC software IRQ numbers.
// See docs/windows-native-aic.md "Timer re-arm handshake" for the full design and
// "Open questions" OQ-1 for why this exact placement is UNVERIFIED.
//
// AIC's own software-triggered-interrupt facility (aic_set_sw() in aic.c, backed by
// the AIC_SW_SET/AIC_SW_CLR registers at aic_regs.h:12-13) lets software post a
// pending event on *any* AIC HW IRQ number, which then goes through the exact same
// ack/mask/affinity machinery as a real peripheral interrupt (aic_ack() returns it via
// AIC_EVENT with AIC_EVENT_TYPE_HW, aic_regs.h:50, indistinguishable from a real
// device). Because HCR_EL2.IMO is clear under this config (see hv.c), an event posted
// this way reaches the guest at EL1 with zero further EL2 involvement, exactly like a
// real peripheral IRQ -- this is what makes it usable as a bugcheck-safe (IRQ, not
// FIQ) timer-tick reflector.
//
// This reserves 2 * MAX_CPUS consecutive IRQ numbers at the TOP of AIC's implemented
// HW IRQ space (aic->nr_irq, populated by aic_init() before the hypervisor starts) -- one CNTP
// ("P") and one CNTV ("V") number per possible CPU, so a given core's timer FIQ always
// reflects to that SAME core (via aic_set_affinity(), see hv_timer_reflect_init()
// below) rather than an arbitrary one.
//
// TODO/M1-VALIDATION (OQ-1): reserving the topmost numbers is a common convention for
// software-only IRQs precisely because real peripherals are enumerated from ADT
// starting near the bottom, but this has NOT been cross-checked against any chip's
// actual interrupt map -- confirm on real hardware (watch for an unexpected event at
// this number, or use hv_trace_irq()/the proxy IRQ tracer) that nothing else claims
// these numbers before trusting this.
//
#define HV_TIMER_SWIRQ_BASE      (aic->nr_irq - (2 * MAX_CPUS))
#define HV_TIMER_P_SWIRQ(cpu)    (HV_TIMER_SWIRQ_BASE + (cpu))
#define HV_TIMER_V_SWIRQ(cpu)    (HV_TIMER_SWIRQ_BASE + MAX_CPUS + (cpu))
#define HV_TIMER_REFLECT_CALL_MAGIC 0x4e54414943ULL /* "NTAIC" */
#define HV_TIMER_CTL_ENABLE      BIT(0)
#define HV_TIMER_CTL_IMASK       BIT(1)
/* Standard GIC PPIs published by the Windows startup-carrier GTDT. */
#define HV_GIC_TIMER_P_INTID     30
#define HV_GIC_TIMER_V_INTID     27

/*
 * Windows ARM64 uses KPCR+0x24d8 (KPRCB.PanicStackBase) for synchronous
 * kernel exceptions and KPCR+0x24e0 (KPRCB.InterruptStackBase) in
 * KxSwitchStackAndPlayInterrupt. During AP startup TPIDR_EL1 becomes non-zero
 * before both stacks are guaranteed writable. Treating TPIDR_EL1 alone as
 * proof that the AP can receive a carrier SGI lets Windows take an exception
 * or interrupt on an uninitialised stack; the resulting nested trap corrupts
 * the PRCB and is later reported as CRITICAL_STRUCTURE_CORRUPTION (0x109).
 *
 * Keep this offset next to the carrier workaround rather than pretending it
 * is architectural.  The checks are read-only and fail closed: both ends of
 * the space consumed by KiKernelStackException must translate writable before
 * m1n1 observes x18 or drains an SGI to the AP.
 */
#define HV_WINDOWS_PANIC_STACK_SLOT_OFFSET 0x24d8
#define HV_WINDOWS_INTERRUPT_STACK_SLOT_OFFSET 0x24e0
#define HV_WINDOWS_PANIC_STACK_RESERVE     0x700
#endif

struct hv_pcpu_data {
    u32 ipi_queued;
    u32 ipi_pending;
    u32 pmc_pending;
    u64 pmc_irq_mode;
    u64 exc_entry_pmcr0_cnt;
    u64 guest_pmuserenr;
#ifdef ENABLE_VGIC_MODULE
    virq_queue_t irq_queue;
    virq_queue_t sgi_queue;
    virq_queue_t timer_queue;
    /* One pending bit per architectural SGI INTID (0..15). */
    u32 sgi_queued_mask;
    u32 sgi_coalesced;
#endif
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    //
    // windows-native-aic timer-FIQ reflector state, one instance per physical timer
    // source (P = CNTP, the non-secure EL1 physical timer; V = CNTV, the virtual
    // timer), per CPU. Only ever read/written by the owning core's own EL2 code (in
    // hv_update_fiq(), called from that core's own FIQ/sync/SError exit paths), same
    // as ipi_pending/pmc_pending above, so plain (non-atomic) read-modify-write is
    // sufficient -- there is no cross-core writer.
    //
    // *_fiq_count is a monotonic "expiration counter": incremented every time EL2
    // observes+masks a new physical FIQ condition for that source, and never
    // decremented by EL2. It is NOT what makes coalescing safe by itself (see the
    // "Coalescing" discussion in docs/windows-native-aic.md) -- it exists so an M1
    // trace can compare "how many physical expirations occurred" against "how many
    // reflected AIC IRQs the guest actually took", and as a hook for a future
    // TRAPPED/COOPERATIVE re-arm design if the pass-through one turns out to be
    // insufficient.
    //
    // *_reflection_pending is true from the moment EL2 masks the source and posts the
    // AIC software IRQ until EL2 next observes (via the live CNTx_CTL_ELx2 register)
    // that the guest reprogrammed a future deadline. It exists so a second expiration
    // that arrives while a reflection is already outstanding does not re-post
    // aic_set_sw() redundantly (harmless either way, since AIC's SW-pending bit is a
    // level, not a FIFO -- see the doc) and so the M1-VALIDATION CHECKLIST has
    // something concrete to assert on.
    //
    u64  timer_p_fiq_count;
    u64  timer_v_fiq_count;
    bool timer_p_reflection_pending;
    bool timer_v_reflection_pending;
    bool timer_p_event_unread;
    bool timer_v_event_unread;
    bool native_doorbell_posted;
    bool carrier_timer_ready;
    bool carrier_x18_zero_logged;
    bool carrier_stack_defer_logged;
    bool carrier_stack_ready;
    bool carrier_vi_logged;
    bool carrier_irq_active;
    /*
     * Set once this CPU has completed a full carrier IAR->EOI cycle.
     * That proves VBAR, KPCR, stack switching and PMR discipline all work.
     * After it, the exception-stack gate must not veto delivery: Windows
     * parks PanicStackBase/InterruptStackBase at 0 while a dispatch is in
     * flight -- which is exactly when an AP self-requests its software
     * interrupt via ICC_SGI1R_EL1. Re-checking the slots then deadlocks the
     * AP forever: NT will not restore them until the interrupt it is waiting
     * for is delivered. Observed on J414s CPU 4 (first Avalanche core),
     * 2026-07-27. This closes a carrier-progress hole; it is distinct from
     * the later NT scheduler 0xA caused by an invalid HAL LocalUnitId.
     */
    bool carrier_delivery_proven;
    u32 carrier_active_intid;
    u32 carrier_iar_count;
    u32 carrier_eoi_count;
    /*
     * Monotonic, read-only diagnostic counters for the guest Fast-IPI path.
     * Keep these at the tail so the established offsets of ipi_queued,
     * ipi_pending, native_doorbell_posted and the carrier counters do not
     * change. The 0x800-byte aligned structure has exactly 16 tail bytes.
     */
    u32 guest_ipi_send_count;
    u32 guest_ipi_tag_take_count;
    u32 guest_ipi_event_emit_count;
    u32 guest_ipi_commit_count;
#endif
} ALIGNED(64);

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
static_assert(sizeof(struct hv_pcpu_data) == 0x800,
              "native-AIC pcpu debug ABI changed");
#endif

struct hv_pcpu_data pcpu[MAX_CPUS];

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
/*
 * A native level IRQ can reach EL2 during the short interval in which a
 * synthetic timer/IPI wake has HCR.IMO asserted.  Keep this outside
 * hv_pcpu_data: that structure's exact 0x800-byte layout is a debugger ABI.
 * While set, doorbell synchronization must not restore IMO until EL1 reads
 * AIC EVENT and thereby auto-masks the real source.  m1n1's private tick stays
 * armed meanwhile so the proxy remains serviceable without stealing the IRQ.
 */
static bool native_irq_rearm_deferred[MAX_CPUS];
static u32 native_irq_bounce_count[MAX_CPUS];
#endif

void hv_exit_guest(void) __attribute__((noreturn));

static u64 stolen_time = 0;
static u64 exc_entry_time;
extern u64 hv_cpus_in_guest;
extern int hv_pinned_cpu;
extern int hv_want_cpu;

static bool time_stealing = true;

void init_vgic_irq_queues(void) {
#ifdef ENABLE_VGIC_MODULE
    for (int i = 0; i < MAX_CPUS; i++) {
        virq_queue_init(&PERCPU_N(i, irq_queue));
        virq_queue_init(&PERCPU_N(i, sgi_queue));
        virq_queue_init(&PERCPU_N(i, timer_queue));
        __atomic_store_n(&PERCPU_N(i, sgi_queued_mask), 0, __ATOMIC_RELAXED);
        __atomic_store_n(&PERCPU_N(i, sgi_coalesced), 0, __ATOMIC_RELAXED);
        PERCPU_N(i, guest_pmuserenr) = 0;
    }
#endif
}

#ifdef ENABLE_VGIC_MODULE
/*
 * A GIC SGI is a pending state, not an edge counter.  Windows can write the
 * same ICC_SGI1R target repeatedly while that SGI is already queued.  Keeping
 * every write in a FIFO replays stale IPIs after EOI and can corrupt scheduler
 * state during processor startup.  Keep at most one queued instance per
 * target/INTID; once the target removes it from the queue, a new write can
 * become the active+pending state in hv_vgic3_inject_irq().
 */
static bool hv_sgi_queue_push(int cpu, const virq_t *pending)
{
    if (cpu < 0 || cpu >= MAX_CPUS || pending->vintid >= 16)
        return false;

    u32 bit = (u32)BIT(pending->vintid);
    if (__atomic_fetch_or(&PERCPU_N(cpu, sgi_queued_mask), bit,
                          __ATOMIC_ACQ_REL) & bit) {
        __atomic_fetch_add(&PERCPU_N(cpu, sgi_coalesced), 1,
                           __ATOMIC_RELAXED);
        return false;
    }

    if (virq_queue_push(&PERCPU_N(cpu, sgi_queue), pending))
        return true;

    __atomic_fetch_and(&PERCPU_N(cpu, sgi_queued_mask), ~bit,
                       __ATOMIC_RELEASE);
    return false;
}

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
/*
 * Windows starts issuing Apple Fast-IPIs while some processors are still in
 * the short GIC startup-carrier phase.  Do not consume those sends into the
 * native EVENT transaction before the first Windows AIC2 CONFIG enable: the
 * readiness bit is global, HCR.VI is CPU-local, and an AP can otherwise take
 * the transport FIQ just before readiness changes, return with no doorbell,
 * mask FIQs in KiInitializeKernel, and never enter EL2 again to arm it.
 *
 * Keep the request as carrier SGI 0 during that window.  If CONFIG becomes
 * ready before the target handles it, hv_carrier_migrate_sgis_to_native()
 * converts the queued SGI into the native transactional EVENT.  If not, the
 * existing carrier path delivers it directly.  Either ordering therefore
 * leaves a CPU-local HCR.VI armed before the target returns to EL1.
 */
static bool hv_guest_ipi_queue_pre_config_carrier(int cpu)
{
    if (!hv_native_aic_windows_active() ||
        hv_native_aic_windows_ready())
        return false;

    virq_t pending = {
        .vintid = 0,
        .priority = hv_vgic3_get_priority_cpu(cpu, 0),
        .active = false,
        .pending = true,
        .hw_status = false,
        .hw_irq = 0,
    };

    if (hv_sgi_queue_push(cpu, &pending))
        smp_send_ipi(cpu);
    return true;
}
#endif

static bool hv_sgi_queue_pop(virq_t *pending)
{
    if (!virq_queue_pop(&PERCPU(sgi_queue), pending))
        return false;

    if (pending->vintid < 16) {
        u32 bit = (u32)BIT(pending->vintid);
        __atomic_fetch_and(&PERCPU(sgi_queued_mask), ~bit, __ATOMIC_RELEASE);
    }
    return true;
}
#endif

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
//
// windows-native-aic: reserve and unmask the per-CPU AIC software IRQs used by the
// timer-FIQ reflector (see the HV_TIMER_P_SWIRQ()/HV_TIMER_V_SWIRQ() comment above and
// docs/windows-native-aic.md). Must run after init_vgic_irq_queues() and after
// aic_init() has run (so `aic->nr_irq` is valid) -- both are already true at
// this function's one call site in hv_init() (hv.c).
//
// aic_set_affinity()/aic_set_mask() write global (not per-calling-core) AIC MMIO state
// (aic.c:216-219, aic.c:204-215 -- the same registers hv_vgic.c's guest-facing GICD
// emulation already pokes on behalf of the guest when vGIC mode is active), so it is
// correct to configure every CPU's reserved IRQ from here, on the boot CPU, once.
//
void hv_timer_reflect_init(void)
{
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        PERCPU_N(cpu, ipi_queued) = 0;
        PERCPU_N(cpu, ipi_pending) = 0;
        PERCPU_N(cpu, timer_p_reflection_pending) = false;
        PERCPU_N(cpu, timer_v_reflection_pending) = false;
        PERCPU_N(cpu, timer_p_event_unread) = false;
        PERCPU_N(cpu, timer_v_event_unread) = false;
        PERCPU_N(cpu, native_doorbell_posted) = false;
        PERCPU_N(cpu, carrier_timer_ready) = false;
        PERCPU_N(cpu, carrier_x18_zero_logged) = false;
        PERCPU_N(cpu, carrier_stack_defer_logged) = false;
        PERCPU_N(cpu, carrier_stack_ready) = false;
        PERCPU_N(cpu, carrier_vi_logged) = false;
        PERCPU_N(cpu, carrier_irq_active) = false;
        PERCPU_N(cpu, carrier_delivery_proven) = false;
        PERCPU_N(cpu, carrier_active_intid) = 0x3ff;
        PERCPU_N(cpu, carrier_iar_count) = 0;
        PERCPU_N(cpu, carrier_eoi_count) = 0;
        PERCPU_N(cpu, guest_ipi_send_count) = 0;
        PERCPU_N(cpu, guest_ipi_tag_take_count) = 0;
        PERCPU_N(cpu, guest_ipi_event_emit_count) = 0;
        PERCPU_N(cpu, guest_ipi_commit_count) = 0;
        native_irq_rearm_deferred[cpu] = false;
        native_irq_bounce_count[cpu] = 0;
        aic_set_sw(HV_TIMER_P_SWIRQ(cpu), false);
        aic_set_sw(HV_TIMER_V_SWIRQ(cpu), false);
    }
    reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
            VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);
    printf("HV: windows-native-aic: holding timer FIQ until Mu AIC is ready\n");
}

void hv_timer_native_enable(void)
{
    PERCPU(timer_p_reflection_pending) = false;
    PERCPU(timer_v_reflection_pending) = false;
    PERCPU(timer_p_event_unread) = false;
    PERCPU(timer_v_event_unread) = false;
    PERCPU(native_doorbell_posted) = false;
    hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
    reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
            VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);
}

void hv_timer_reflect_enable(void)
{
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        aic_set_sw(HV_TIMER_P_SWIRQ(cpu), false);
        aic_set_sw(HV_TIMER_V_SWIRQ(cpu), false);
        PERCPU_N(cpu, carrier_timer_ready) = false;
        PERCPU_N(cpu, native_doorbell_posted) = false;
    }
    PERCPU(timer_p_reflection_pending) = false;
    PERCPU(timer_v_reflection_pending) = false;
    PERCPU(timer_p_event_unread) = false;
    PERCPU(timer_v_event_unread) = false;
    PERCPU(native_doorbell_posted) = false;
    reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
            VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);
    printf("HV: windows-native-aic: enabled FIQ-to-AIC-EVENT timer bridge\n");
}

void hv_timer_reflect_hold(void)
{
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        aic_set_sw(HV_TIMER_P_SWIRQ(cpu), false);
        aic_set_sw(HV_TIMER_V_SWIRQ(cpu), false);
        PERCPU_N(cpu, timer_p_reflection_pending) = false;
        PERCPU_N(cpu, timer_v_reflection_pending) = false;
        PERCPU_N(cpu, timer_p_event_unread) = false;
        PERCPU_N(cpu, timer_v_event_unread) = false;
        PERCPU_N(cpu, native_doorbell_posted) = false;
        PERCPU_N(cpu, carrier_timer_ready) = false;
    }
    hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
    reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
            VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);
    printf("HV: windows-native-aic: holding timer bridge until Windows enables AIC2\n");
}

void hv_carrier_retire_active_sgis(void)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    for (int cpu = 0; cpu < MAX_CPUS; cpu++) {
        u32 intid = __atomic_load_n(&pcpu[cpu].carrier_active_intid,
                                   __ATOMIC_ACQUIRE);

        /*
         * CONFIG changes the interrupt-controller callbacks globally.  An AP
         * can already have accepted a startup-carrier SGI when the BSP makes
         * that change; its ISR then completes through the native AIC callback
         * and no GIC EOIR is issued.  Leaving the software carrier active in
         * that case suppresses HCR.VI forever on the AP.  The SGI has already
         * reached Windows, so retire only accepted SGIs here.  Other carrier
         * interrupt classes remain fail-closed for diagnosis.
         */
        if (__atomic_load_n(&pcpu[cpu].carrier_irq_active,
                            __ATOMIC_ACQUIRE) && intid < 16) {
            __atomic_store_n(&pcpu[cpu].carrier_active_intid, 0x3ff,
                             __ATOMIC_RELEASE);
            __atomic_store_n(&pcpu[cpu].carrier_irq_active, false,
                             __ATOMIC_RELEASE);
            printf("HV: windows-native-aic: retired active carrier SGI %u "
                   "on CPU %d at CONFIG handoff\n", intid, cpu);
        }
    }
#endif
}

static bool hv_guest_ipi_doorbell_pending(void)
{
    u32 state = PERCPU(ipi_pending);

    return ((state & HV_GUEST_IPI_DELIVERABLE) &&
            !(state & HV_GUEST_IPI_INFLIGHT)) ||
           (state & HV_GUEST_IPI_RETRY_ARMED);
}

static u32 hv_guest_ipi_clock_now(void)
{
    return (u32)(mrs(CNTPCT_EL0) >> HV_GUEST_IPI_CLOCK_SHIFT) &
           HV_GUEST_IPI_CLOCK_MASK;
}

static u32 hv_guest_ipi_stamp(u32 state)
{
    state &= ~HV_GUEST_IPI_START_TIME;
    state |= FIELD_PREP(HV_GUEST_IPI_START_TIME,
                        hv_guest_ipi_clock_now());
    return state;
}

static bool hv_guest_ipi_begin_delivery(u32 *event_token)
{
    u32 state = PERCPU(ipi_pending);
    u32 next_generation;
    bool carrier;

    if (event_token == NULL)
        return false;

    if ((state & HV_GUEST_IPI_INFLIGHT) &&
        (state & HV_GUEST_IPI_RETRY_ARMED)) {
        /* Re-emit the same uncommitted generation. */
        state &= ~HV_GUEST_IPI_RETRY_ARMED;
    } else if ((state & HV_GUEST_IPI_DELIVERABLE) &&
               !(state & HV_GUEST_IPI_INFLIGHT)) {
        /*
         * Begin a new transaction. Bits 30:24 of the synthetic EVENT carry a
         * seven-bit generation, while bit 31 records that this wake came from
         * a GIC-carrier SGI migrated across CONFIG. The origin bit lets the
         * HAL distinguish real class-0 carrier work from an ordinary
         * redundant Fast-IPI edge whose class bitmap is already empty.
         */
        carrier = (state & HV_GUEST_IPI_QUEUED_CARRIER) != 0;
        next_generation =
            (FIELD_GET(HV_GUEST_IPI_GENERATION, state) + 1) & MASK(7);
        state &= ~(HV_GUEST_IPI_DELIVERABLE |
                   HV_GUEST_IPI_QUEUED_CARRIER |
                   HV_GUEST_IPI_RETRY_ARMED |
                   HV_GUEST_IPI_RETRY_COUNT |
                   HV_GUEST_IPI_START_TIME |
                   HV_GUEST_IPI_EVENT_TOKEN);
        state |= HV_GUEST_IPI_INFLIGHT |
                 FIELD_PREP(HV_GUEST_IPI_GENERATION, next_generation);
        if (carrier)
            state |= HV_GUEST_IPI_INFLIGHT_CARRIER;
    } else {
        return false;
    }

    PERCPU(ipi_pending) = hv_guest_ipi_stamp(state);
    PERCPU(guest_ipi_event_emit_count)++;
    *event_token = FIELD_GET(HV_GUEST_IPI_EVENT_TOKEN, state);
    return true;
}

static bool hv_guest_ipi_commit_delivery(void)
{
    u32 state = PERCPU(ipi_pending);

    if (!(state & HV_GUEST_IPI_INFLIGHT))
        return false;

    /* Preserve a newer DELIVERABLE generation across this EOI commit. */
    state &= ~(HV_GUEST_IPI_INFLIGHT |
               HV_GUEST_IPI_INFLIGHT_CARRIER |
               HV_GUEST_IPI_RETRY_ARMED |
               HV_GUEST_IPI_RETRY_COUNT |
               HV_GUEST_IPI_START_TIME);
    PERCPU(ipi_pending) = state;
    PERCPU(guest_ipi_commit_count)++;
    return true;
}

static void hv_guest_ipi_retry_tick(void)
{
    u32 state;
    u32 retries;
    u32 now;
    u32 started;
    u32 elapsed;
    u32 delay;

    if (!hv_native_aic_windows_ready())
        return;

    state = PERCPU(ipi_pending);
    if (!(state & HV_GUEST_IPI_INFLIGHT) ||
        (state & HV_GUEST_IPI_RETRY_ARMED))
        return;

    now = hv_guest_ipi_clock_now();
    started = FIELD_GET(HV_GUEST_IPI_START_TIME, state);
    elapsed = (now - started) & HV_GUEST_IPI_CLOCK_MASK;
    delay = (u32)((mrs(CNTFRQ_EL0) / 4) >> HV_GUEST_IPI_CLOCK_SHIFT);
    if (delay == 0)
        delay = 1;
    if (elapsed < delay)
        return;

    /*
     * A non-interruptible AP receives m1n1's one-Hz slow tick even while EL1
     * is parked in WFI.  The boot CPU ticks at 5 kHz, so use the architectural
     * counter rather than tick count and wait at least 250 ms between attempts.
     * The stored 17-bit coarse timestamp wraps after tens of minutes on J414s;
     * unsigned modular subtraction is unambiguous for this sub-second delay.
     * Retry frequency is bounded, but the transport is never abandoned while
     * NT still owes an EOI. Saturate the diagnostic count rather than recreating
     * the original permanent INFLIGHT-with-no-doorbell hang after a fixed cap.
     */
    retries = FIELD_GET(HV_GUEST_IPI_RETRY_COUNT, state);
    if (retries < HV_GUEST_IPI_RETRY_COUNT_MAX)
        retries++;
    state &= ~HV_GUEST_IPI_RETRY_COUNT;
    state |= FIELD_PREP(HV_GUEST_IPI_RETRY_COUNT, retries) |
             HV_GUEST_IPI_RETRY_ARMED;
    PERCPU(ipi_pending) = hv_guest_ipi_stamp(state);
}

static void hv_native_aic_doorbell_sync(void)
{
    bool pending = hv_guest_ipi_doorbell_pending() ||
                   PERCPU(timer_p_event_unread) ||
                   PERCPU(timer_v_event_unread);
    bool carrier_active = hv_native_aic_windows_ready() &&
                          PERCPU(carrier_irq_active);
    u64 hcr;

    if (!hv_native_aic_windows_ready())
        return;

    if (native_irq_rearm_deferred[smp_id()]) {
        /*
         * Do not recreate IMO in the exception that released a real native
         * IRQ to EL1.  A level source is still asserted until Windows reads
         * AIC EVENT; immediate re-arm would bounce straight back to EL2 and
         * livelock both the guest and m1n1's proxy tick.
         */
        PERCPU(native_doorbell_posted) = false;
        hcr = mrs(HCR_EL2);
        if (hcr & (HCR_IMO | HCR_VI))
            hv_write_hcr(hcr & ~(HCR_IMO | HCR_VI));
        return;
    }

    hcr = mrs(HCR_EL2);
    if (carrier_active) {
        /*
         * CONFIG is global, but another CPU can still be between its trapped
         * carrier IAR and EOIR when CPU 0 enables AIC2.  Keep TALL1 enabled so
         * that accepted interrupt's EOIR still traps, but stop routing native
         * physical IRQs to EL2.  Otherwise a pending AIC source immediately
         * re-enters hv_exc_irq() forever because EL2 deliberately does not
         * acknowledge it.  A native synthetic wake accumulated meanwhile
         * remains level-pending in software and is armed by the EOIR exit.
         */
        PERCPU(native_doorbell_posted) = false;
        if (hcr & (HCR_IMO | HCR_VI))
            hv_write_hcr(hcr & ~(HCR_IMO | HCR_VI));
        return;
    }

    if (pending) {
        PERCPU(native_doorbell_posted) = true;
        /*
         * AIC2 has no routable software-HW-IRQ facility: Linux likewise uses
         * Fast IPIs rather than AIC1's MMIO IPI block.  Assert a CPU-local
         * virtual IRQ only as the wakeup edge.  Windows still acknowledges
         * its source through the trapped native AIC EVENT register below;
         * no GIC list register or distributor state is involved.
         */
        if ((hcr & (HCR_IMO | HCR_VI)) != (HCR_IMO | HCR_VI))
            hv_write_hcr(hcr | HCR_IMO | HCR_VI);
    } else {
        PERCPU(native_doorbell_posted) = false;
        if (hcr & (HCR_IMO | HCR_VI))
            hv_write_hcr(hcr & ~(HCR_IMO | HCR_VI));
    }
}
#endif

static void _hv_exc_proxy(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type,
                          void *extra)
{
    int from_el = FIELD_GET(SPSR_M, ctx->spsr) >> 2;

    hv_wdt_breadcrumb('P');

    /*
     * Get all the CPUs into the HV before running the proxy, to make sure they all exit to
     * the guest with a consistent time offset.
     */
    if (time_stealing)
        hv_rendezvous();

    u64 entry_time = mrs(CNTPCT_EL0);

    ctx->elr_phys = hv_translate(ctx->elr, false, false, NULL);
    ctx->far_phys = hv_translate(ctx->far, false, false, NULL);
    ctx->sp_phys = hv_translate(from_el == 0 ? ctx->sp[0] : ctx->sp[1], false, false, NULL);
    ctx->extra = extra;

    struct uartproxy_msg_start start = {
        .reason = reason,
        .code = type,
        .info = ctx,
    };

    hv_wdt_suspend();
    int ret = uartproxy_run(&start);
    hv_wdt_resume();

    switch (ret) {
        case EXC_RET_HANDLED:
            hv_wdt_breadcrumb('p');
            if (time_stealing) {
                u64 lost = mrs(CNTPCT_EL0) - entry_time;
                stolen_time += lost;
            }
            break;
        case EXC_EXIT_GUEST:
            hv_rendezvous();
            spin_unlock(&bhl);
            hv_exit_guest(); // does not return
        default:
            printf("Guest exception not handled, rebooting.\n");
            print_regs(ctx->regs, 0);
            flush_and_reboot(); // does not return
    }
}

static void hv_maybe_switch_cpu(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type,
                                void *extra)
{
    while (hv_want_cpu != -1) {
        if (hv_want_cpu == smp_id()) {
            hv_want_cpu = -1;
            _hv_exc_proxy(ctx, reason, type, extra);
        } else {
            // Unlock the HV so the target CPU can get into the proxy
            spin_unlock(&bhl);
            while (hv_want_cpu != -1)
                sysop("dmb sy");
            spin_lock(&bhl);
        }
    }
}

void hv_exc_proxy(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type, void *extra)
{
    /*
     * Wait while another CPU is pinned or being switched to.
     * If a CPU switch is requested, handle it before actually handling the
     * exception. We still tell the host the real reason code, though.
     */
    while ((hv_pinned_cpu != -1 && hv_pinned_cpu != smp_id()) || hv_want_cpu != -1) {
        if (hv_want_cpu == smp_id()) {
            hv_want_cpu = -1;
            _hv_exc_proxy(ctx, reason, type, extra);
        } else {
            // Unlock the HV so the target CPU can get into the proxy
            spin_unlock(&bhl);
            while ((hv_pinned_cpu != -1 && hv_pinned_cpu != smp_id()) || hv_want_cpu != -1)
                sysop("dmb sy");
            spin_lock(&bhl);
        }
    }

    /* Handle the actual exception */
    _hv_exc_proxy(ctx, reason, type, extra);

    /*
     * If as part of handling this exception we want to switch CPUs, handle it without returning
     * to the guest.
     */
    hv_maybe_switch_cpu(ctx, reason, type, extra);
}

void hv_set_time_stealing(bool enabled, bool reset)
{
    time_stealing = enabled;
    if (reset)
        stolen_time = 0;
}

void hv_add_time(s64 time)
{
    stolen_time -= (u64)time;
}

static void hv_carrier_observe_x18(struct exc_info *ctx)
{
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    if (ctx == NULL || !hv_native_aic_windows_active() ||
        (FIELD_GET(SPSR_M, ctx->spsr) >> 2) != 1)
        return;

    /*
     * TPIDR_EL1 may carry Windows-private low-bit tags while x18 is the
     * page-aligned KPCR.  Live J414s captures show, for example,
     * TPIDR_EL1=...1002 paired with x18=...1000.  Never copy those tags into
     * the architectural x18 alias.
     */
    u64 guest_pcr = mrs(TPIDR_EL1) & ~0xfffULL;
    if (guest_pcr == 0)
        return;

    if (ctx->regs[18] == 0 && !PERCPU(carrier_x18_zero_logged)) {
        printf("HV: windows-native-aic: observed zero guest x18 on CPU %d "
               "with tag-stripped TPIDR_EL1=0x%lx at ELR=0x%lx "
               "CYC_OVRD=0x%lx\n",
               smp_id(), guest_pcr, ctx->elr, mrs(SYS_IMP_APL_CYC_OVRD));
        PERCPU(carrier_x18_zero_logged) = true;
    }

    /*
     * Do not cache this translation. Windows changes AP page tables during
     * bring-up and can tear the temporary KPCR mapping down after a startup
     * timeout. A once-valid PanicStackBase is therefore not proof that a later
     * carrier exception is safe.
     */
    u64 stack_slot = hv_translate(guest_pcr + HV_WINDOWS_PANIC_STACK_SLOT_OFFSET,
                                  false, false, NULL);
    u64 panic_stack = stack_slot ? read64(stack_slot) : 0;
    u64 interrupt_stack_slot =
        hv_translate(guest_pcr + HV_WINDOWS_INTERRUPT_STACK_SLOT_OFFSET,
                     false, false, NULL);
    u64 interrupt_stack = interrupt_stack_slot ? read64(interrupt_stack_slot) : 0;
    bool panic_stack_ready = panic_stack >= HV_WINDOWS_PANIC_STACK_RESERVE &&
                             hv_translate(panic_stack - 8, false, true, NULL) != 0 &&
                             hv_translate(panic_stack - HV_WINDOWS_PANIC_STACK_RESERVE,
                                          false, true, NULL) != 0;
    bool interrupt_stack_ready =
        interrupt_stack >= HV_WINDOWS_PANIC_STACK_RESERVE &&
        hv_translate(interrupt_stack - 16, false, true, NULL) != 0 &&
        hv_translate(interrupt_stack - HV_WINDOWS_PANIC_STACK_RESERVE,
                     false, true, NULL) != 0;
    bool stack_ready = panic_stack_ready && interrupt_stack_ready;
    if (!stack_ready) {
        if (PERCPU(carrier_stack_ready)) {
            printf("HV: windows-native-aic: carrier CPU %d lost KPCR/panic-stack "
                   "mapping (TPIDR=0x%lx panic=0x%lx interrupt=0x%lx "
                   "ELR=0x%lx ESR=0x%lx FAR=0x%lx proven=%d)\n",
                   smp_id(), guest_pcr, panic_stack, interrupt_stack, ctx->elr,
                   ctx->esr, ctx->far, PERCPU(carrier_delivery_proven));
        } else if (!PERCPU(carrier_stack_defer_logged)) {
            printf("HV: windows-native-aic: deferring carrier CPU %d; "
                   "exception stacks are not writable (TPIDR=0x%lx "
                   "panic=0x%lx interrupt=0x%lx)\n",
                   smp_id(), guest_pcr, panic_stack, interrupt_stack);
            PERCPU(carrier_stack_defer_logged) = true;
        }
        PERCPU(carrier_stack_ready) = false;
        return;
    }

    if (!PERCPU(carrier_stack_ready)) {
        printf("HV: windows-native-aic: carrier CPU %d exception stacks ready "
               "(panic=0x%lx interrupt=0x%lx)\n",
               smp_id(), panic_stack, interrupt_stack);
        PERCPU(carrier_stack_ready) = true;
    }

#else
    (void)ctx;
#endif
}

static void hv_carrier_drain_pending(struct exc_info *ctx)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    hv_carrier_observe_x18(ctx);
#else
    (void)ctx;
#endif
}

enum hv_carrier_queue {
    HV_CARRIER_QUEUE_NONE,
    HV_CARRIER_QUEUE_SGI,
    HV_CARRIER_QUEUE_TIMER,
    HV_CARRIER_QUEUE_IRQ,
};

static enum hv_carrier_queue hv_carrier_select_pending(virq_t *selected)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    enum hv_carrier_queue best_queue = HV_CARRIER_QUEUE_NONE;
    virq_t candidate;

#define CONSIDER_CARRIER_QUEUE(queue, which)                                  \
    do {                                                                      \
        if (virq_queue_peek((queue), &candidate) &&                           \
            (best_queue == HV_CARRIER_QUEUE_NONE ||                           \
             candidate.priority < selected->priority)) {                      \
            *selected = candidate;                                            \
            best_queue = (which);                                             \
        }                                                                     \
    } while (0)

    CONSIDER_CARRIER_QUEUE(&PERCPU(sgi_queue), HV_CARRIER_QUEUE_SGI);
    CONSIDER_CARRIER_QUEUE(&PERCPU(timer_queue), HV_CARRIER_QUEUE_TIMER);
    CONSIDER_CARRIER_QUEUE(&PERCPU(irq_queue), HV_CARRIER_QUEUE_IRQ);
#undef CONSIDER_CARRIER_QUEUE
    return best_queue;
#else
    (void)selected;
    return HV_CARRIER_QUEUE_NONE;
#endif
}

static u32 hv_carrier_do_iar1(void)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    if (PERCPU(carrier_irq_active))
        return 0x3ff;

    virq_t selected = {.priority = 0xff};
    enum hv_carrier_queue queue = hv_carrier_select_pending(&selected);
    bool popped = false;
    switch (queue) {
        case HV_CARRIER_QUEUE_SGI:
            popped = hv_sgi_queue_pop(&selected);
            break;
        case HV_CARRIER_QUEUE_TIMER:
            popped = virq_queue_pop(&PERCPU(timer_queue), &selected);
            break;
        case HV_CARRIER_QUEUE_IRQ:
            popped = virq_queue_pop(&PERCPU(irq_queue), &selected);
            break;
        default:
            break;
    }
    if (!popped)
        return 0x3ff;

    PERCPU(carrier_irq_active) = true;
    PERCPU(carrier_active_intid) = selected.vintid;
    return selected.vintid;
#else
    return 0x3ff;
#endif
}

static void hv_carrier_do_eoir1(u32 intid)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    if (PERCPU(carrier_irq_active) &&
        PERCPU(carrier_active_intid) == intid) {
        PERCPU(carrier_irq_active) = false;
        PERCPU(carrier_active_intid) = 0x3ff;
        PERCPU(carrier_delivery_proven) = true;
    }
#else
    (void)intid;
#endif
}

static bool hv_guest_ipi_take_tag(void)
{
    if (!__atomic_exchange_n(&PERCPU(ipi_queued), false, __ATOMIC_ACQUIRE))
        return false;

    /* Do not overwrite an older INFLIGHT transaction. */
    PERCPU(ipi_pending) |= HV_GUEST_IPI_DELIVERABLE;
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    PERCPU(guest_ipi_tag_take_count)++;
#endif
    return true;
}

/*
 * A Windows GIC-carrier SGI can race the global AIC2 CONFIG write.  The old
 * post-CONFIG FIQ path popped such an SGI into an ICH LR after the local CPU
 * interface had been disabled.  That made every hypervisor pending field
 * read zero while NT's per-source IPI node remained queued forever.
 *
 * Once native AIC is ready, an architectural SGI no longer needs its GIC
 * INTID: Windows' own per-processor IPI queue is authoritative.  Preserve one
 * level wake for any queued SGI and let the native EVENT hook return the
 * Apple Fast-IPI token.  Coalescing is correct because one KiIpiInterrupt
 * drains the target's NT queue.
 */
static void hv_carrier_migrate_sgis_to_native(void)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    if (!hv_native_aic_windows_ready())
        return;

    virq_t pending;
    u32 migrated = 0;
    while (hv_sgi_queue_pop(&pending))
        migrated++;

    if (migrated != 0) {
        PERCPU(ipi_pending) |= HV_GUEST_IPI_DELIVERABLE |
                               HV_GUEST_IPI_QUEUED_CARRIER;
        printf("HV: windows-native-aic: migrated %u carrier SGI(s) "
               "to native wake on CPU %d\n", migrated, smp_id());
    }
#endif
}

static bool hv_carrier_irq_pending(void)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    if (!hv_native_aic_windows_active() || hv_native_aic_windows_ready() ||
        !hv_vgic3_get_igrpen1() ||
        (!PERCPU(carrier_stack_ready) && !PERCPU(carrier_delivery_proven)))
        return false;

    /*
     * J414s' Blizzard cores assert a virtual IRQ for a pending ICH LR, but the
     * first Avalanche core can leave the same LR pending indefinitely. Keep
     * the short-lived carrier's pending/active state in software and drive the
     * IRQ line with HCR.VI while a deliverable Group-1 entry is queued.
     *
     * HCR.VI bypasses the virtual CPU interface's priority filter, so mirror
     * the relevant VMCR checks here. A second interrupt is not deliverable
     * until EOIR clears the software active state.
     */
    if (PERCPU(carrier_irq_active))
        return false;

    u8 pmr = (mrs(ICH_VMCR_EL2) >> 24) & 0xff;
    virq_t selected = {.priority = 0xff};
    if (hv_carrier_select_pending(&selected) != HV_CARRIER_QUEUE_NONE &&
        selected.priority < pmr)
        return true;
#endif
    return false;
}

static void hv_update_fiq(struct exc_info *ctx)
{ 
    u64 hcr = mrs(HCR_EL2);
    bool fiq_pending = false;

    hv_carrier_observe_x18(ctx);
    hv_carrier_drain_pending(ctx);

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    if (hv_native_aic_windows_active() && hv_native_aic_windows_ready()) {
        hv_carrier_migrate_sgis_to_native();
        /* CONFIG is global, but HCR is per-CPU. Retire the startup carrier on
         * every AP at its first post-handoff EL2 entry.  An AP already between
         * carrier IAR and EOIR must retain TALL1 until the EOIR trap clears its
         * software-active state. */
        if (!PERCPU(carrier_irq_active) &&
            ((hcr & HCR_IMO) || mrs(ICH_HCR_EL2) != 0)) {
            hv_native_aic_enter_cpu();
            hcr = mrs(HCR_EL2);
        }
        /*
         * Windows owns AIC directly, but Apple wires the architectural timers
         * to FIQ.  Coalesce an asserted timer, suppress its physical FIQ, and
         * assert HCR.VI.  The HAL then takes an ordinary IRQ and reads the AIC
         * EVENT hook, which returns the native Apple source value 2 or 3.
         */
        if (mrs(CNTP_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
            fiq_pending = true;
            reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
            if (!PERCPU(timer_p_reflection_pending)) {
                PERCPU(timer_p_fiq_count)++;
                PERCPU(timer_p_reflection_pending) = true;
                PERCPU(timer_p_event_unread) = true;
            }
        } else {
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
            PERCPU(timer_p_reflection_pending) = false;
            PERCPU(timer_p_event_unread) = false;
        }

        if (mrs(CNTV_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
            fiq_pending = true;
            reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
            if (!PERCPU(timer_v_reflection_pending)) {
                PERCPU(timer_v_fiq_count)++;
                PERCPU(timer_v_reflection_pending) = true;
                PERCPU(timer_v_event_unread) = true;
            }
        } else {
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
            PERCPU(timer_v_reflection_pending) = false;
            PERCPU(timer_v_event_unread) = false;
        }
        hv_native_aic_doorbell_sync();
    } else if (hv_native_aic_windows_active()) {
        /*
         * Windows calibrates its architectural clock before the AIC HAL
         * extension enables CONFIG.  During that narrow window, reflect the
         * architected GTDT timer PPIs 30/27 through empty startup-carrier LRs. A timer write
         * and ICC_IGRPEN1=1 are both required, so stale Mu state cannot create
         * an IRQ storm.  This path disappears when CONFIG switches to AIC2.
         */
        PERCPU(timer_p_event_unread) = false;
        PERCPU(timer_v_event_unread) = false;

        if (!PERCPU(carrier_timer_ready) || !hv_vgic3_get_igrpen1() ||
            ctx == NULL || ctx->regs[18] == 0) {
            reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
                    VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);
            PERCPU(timer_p_reflection_pending) = false;
            PERCPU(timer_v_reflection_pending) = false;
        } else {
            if (mrs(CNTP_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
                fiq_pending = true;
                reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
                if (!PERCPU(timer_p_reflection_pending) &&
                    !PERCPU(carrier_irq_active)) {
                    PERCPU(timer_p_fiq_count)++;
                    PERCPU(timer_p_reflection_pending) = true;
                    virq_t pending = {
                        .vintid = HV_GIC_TIMER_P_INTID,
                        .priority = hv_vgic3_get_priority(HV_GIC_TIMER_P_INTID),
                        .pending = true,
                    };
                    virq_queue_push(&PERCPU(timer_queue), &pending);
                }
            } else {
                PERCPU(timer_p_reflection_pending) = false;
                reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
            }

            if (mrs(CNTV_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
                fiq_pending = true;
                reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
                if (!PERCPU(timer_v_reflection_pending) &&
                    !PERCPU(carrier_irq_active)) {
                    PERCPU(timer_v_fiq_count)++;
                    PERCPU(timer_v_reflection_pending) = true;
                    virq_t pending = {
                        .vintid = HV_GIC_TIMER_V_INTID,
                        .priority = hv_vgic3_get_priority(HV_GIC_TIMER_V_INTID),
                        .pending = true,
                    };
                    virq_queue_push(&PERCPU(timer_queue), &pending);
                }
            } else {
                PERCPU(timer_v_reflection_pending) = false;
                reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
            }
        }
    } else if (hv_native_aic_mu_timer_active()) {
        /*
         * Mu's native FIQ exception return corrupts its exception frame on
         * J414s.  Keep FIQ at EL2 and post the timer through Mu's reserved
         * real-AIC software IRQ ABI instead.  Ordinary device IRQs remain
         * physical AIC pass-through and no GIC state is exposed.
         */
        if (mrs(CNTP_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
            fiq_pending = true;
            reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
            if (!PERCPU(timer_p_reflection_pending)) {
                PERCPU(timer_p_fiq_count)++;
                PERCPU(timer_p_reflection_pending) = true;
                aic_set_sw(HV_TIMER_P_SWIRQ(smp_id()), true);
            }
        } else if (!PERCPU(timer_p_reflection_pending)) {
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
        }

        if (mrs(CNTV_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
            fiq_pending = true;
            reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
            if (!PERCPU(timer_v_reflection_pending)) {
                PERCPU(timer_v_fiq_count)++;
                PERCPU(timer_v_reflection_pending) = true;
                aic_set_sw(HV_TIMER_V_SWIRQ(smp_id()), true);
            }
        } else if (!PERCPU(timer_v_reflection_pending)) {
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
        }
    }
#else
    if (mrs(CNTP_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        fiq_pending = true;
        reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);

        //TODO: proper injection
#ifdef ENABLE_VGIC_MODULE
        if(hv_vgic3_get_free_lr() != -1){
            hv_vgic3_inject_irq(
                17,                         //vintid
                hv_vgic3_get_priority(17),  //priority
                false,                      //active
                true,                       //pending
                false,                      //hw_status
                0                           //hw_irq
            );
        }
        else{
            virq_t pending = {
                .vintid = 17,
                .priority = 0x20,
                .active = false,
                .pending = true,
                .hw_status = false,
                .hw_irq = 0,
            };
            virq_queue_push(&PERCPU(timer_queue), &pending);
        }
#endif
    } else {
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
    }

    if (mrs(CNTV_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        fiq_pending = true;
        reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);

        //TODO: proper injection
#ifdef ENABLE_VGIC_MODULE
        if(hv_vgic3_get_free_lr() != -1){
            hv_vgic3_inject_irq(
                18,                         //vintid
                hv_vgic3_get_priority(18),  //priority
                false,                      //active
                true,                       //pending
                false,                      //hw_status
                0                           //hw_irq
            );
        }
        else{
            virq_t pending = {
                .vintid = 18,
                .priority = 0x20,
                .active = false,
                .pending = true,
                .hw_status = false,
                .hw_irq = 0,
            };
            virq_queue_push(&PERCPU(timer_queue), &pending);
        }
#endif
    } else {
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
    }
#endif

    fiq_pending |= (PERCPU(ipi_pending) & HV_GUEST_IPI_OUTSTANDING) ||
                   PERCPU(pmc_pending);

    sysop("isb");
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    bool carrier_pending = hv_carrier_irq_pending();
    bool native_pending = hv_native_aic_windows_ready() &&
                          (hv_guest_ipi_doorbell_pending() ||
                           PERCPU(timer_p_event_unread) ||
                           PERCPU(timer_v_event_unread));
    hcr = mrs(HCR_EL2);
    if (carrier_pending && !PERCPU(carrier_vi_logged)) {
        printf("HV: windows-native-aic: carrier VI armed CPU %d ELR=0x%lx "
               "SPSR=0x%lx TPIDR=0x%lx\n",
               smp_id(), ctx ? ctx->elr : 0, ctx ? ctx->spsr : 0,
               mrs(TPIDR_EL1));
        PERCPU(carrier_vi_logged) = true;
    }
    if (hv_native_aic_windows_ready() && PERCPU(carrier_irq_active)) {
        if (hcr & (HCR_IMO | HCR_VI))
            hv_write_hcr(hcr & ~(HCR_IMO | HCR_VI));
    } else if (native_pending) {
        if ((hcr & (HCR_IMO | HCR_VI)) != (HCR_IMO | HCR_VI))
            hv_write_hcr(hcr | HCR_IMO | HCR_VI);
    } else if (carrier_pending) {
        if (!(hcr & HCR_VI))
            hv_write_hcr(hcr | HCR_VI);
    } else if (hcr & HCR_VI) {
        hcr &= ~HCR_VI;
        if (hv_native_aic_windows_ready() && !PERCPU(carrier_irq_active))
            hcr &= ~HCR_IMO;
        hv_write_hcr(hcr);
    }
#elif !defined(ENABLE_VGIC_MODULE)
    if ((hcr & HCR_VF) && !fiq_pending) {
        hv_write_hcr(hcr & ~HCR_VF);
    } else if (!(hcr & HCR_VF) && fiq_pending) {
        hv_write_hcr(hcr | HCR_VF);
    }
#endif
}

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
bool hv_native_aic_event_read(u64 raw_event, u64 *event)
{
    u32 type = FIELD_GET(AIC_EVENT_TYPE, raw_event);
    u32 die = FIELD_GET(AIC_EVENT_DIE, raw_event);
    u32 irq = FIELD_GET(AIC_EVENT_NUM, raw_event);
    u32 flat_irq = die * aic->max_irq + irq;
    u32 ipi_event_token;
    bool reserved = type == AIC_EVENT_TYPE_HW &&
                    flat_irq >= HV_TIMER_SWIRQ_BASE &&
                    flat_irq < HV_TIMER_SWIRQ_BASE + (2 * MAX_CPUS);

    if (!hv_native_aic_windows_ready() || event == NULL)
        return false;

    if (raw_event != 0 && !reserved) {
        /*
         * A real native AIC source won arbitration ahead of the synthetic
         * wakeup.  Reading EVENT has now auto-masked that level source, so it
         * is finally safe to restore IMO for a queued synthetic wake.
         */
        native_irq_rearm_deferred[smp_id()] = false;
        hv_native_aic_doorbell_sync();
        return false;
    }

    /*
     * A reserved Mu timer reflector can race the ExitBootServices/Windows
     * CONFIG transition on another CPU and remain latched after the bulk
     * SW_CLEAR. These implementation-private IRQ numbers must never escape
     * to the Windows controller: HalBeginSystemInterrupt treats the unknown
     * line as a fatal controller result (0x5c/0x203). Consume the stale token
     * before returning a pending processor-local source, or a spurious EVENT.
     */
    if (reserved) {
        aic_set_sw(flat_irq, false);
        aic_set_mask(flat_irq, true);
        printf("HV: windows-native-aic: discarded stale reserved EVENT IRQ %u on CPU %d\n",
               flat_irq, smp_id());
    }

    PERCPU(native_doorbell_posted) = false;

    /*
     * Fast IPIs are FIQ-class and are consumed by m1n1 for its own EL2
     * coordination.  Guest-originated sends are tagged by ipi_queued; reflect
     * only those arrivals as the architectural AIC IPI EVENT expected by the
     * native Windows controller callback.  This is an AIC EVENT/IRQ delivery,
     * not a vGIC list-register injection.
     */
    if (hv_guest_ipi_begin_delivery(&ipi_event_token)) {
        *event = FIELD_PREP(AIC_EVENT_DIE, ipi_event_token) |
                 FIELD_PREP(AIC_EVENT_TYPE, AIC_EVENT_TYPE_IPI) |
                 AIC_EVENT_IPI_OTHER;
    /* Windows uses the virtual timer in the proven QEMU AIC path. */
    } else if (PERCPU(timer_v_event_unread)) {
        *event = 3;
        PERCPU(timer_v_event_unread) = false;
    } else if (PERCPU(timer_p_event_unread)) {
        *event = 2;
        PERCPU(timer_p_event_unread) = false;
    } else {
        *event = 0;
    }

    hv_native_aic_doorbell_sync();
    return true;
}

/*
 * A guest timer write is the reliable completion signal for a reflected tick.
 * Merely polling ISTATUS in hv_update_fiq() is racy: immediately after an MSR
 * write the old asserted status can still be observed, and if EL1 then returns
 * without another trap the reflector remains suppressed forever.  Rearm the
 * physical source at the trapped write itself.  hv_update_fiq() runs on the
 * same exception exit and will suppress/post it again if the new deadline is
 * already expired.
 */
static void hv_timer_reflect_guest_rearm(bool physical, bool control_write, u64 value,
                                         bool guest_cpu_ready)
{
    sysop("isb");

    if (!hv_native_aic_windows_active()) {
        /*
         * TimerDxe first writes CTL to disable/mask the timer, then registers
         * all callbacks, and only at the end writes ENABLE=1 with IMASK clear.
         * That final control write is the proof that native FIQ can be released.
         */
        if (control_write && (value & HV_TIMER_CTL_ENABLE) &&
            !(value & HV_TIMER_CTL_IMASK)) {
            hv_native_aic_timer_ready();
        } else if (hv_native_aic_mu_timer_active()) {
            if (physical) {
                PERCPU(timer_p_reflection_pending) = false;
                aic_set_sw(HV_TIMER_P_SWIRQ(smp_id()), false);
                reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
            } else {
                PERCPU(timer_v_reflection_pending) = false;
                aic_set_sw(HV_TIMER_V_SWIRQ(smp_id()), false);
                reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
            }
        }
        return;
    }

    if (!hv_native_aic_windows_ready()) {
        /*
         * Readiness is CPU-local.  A BSP timer write must never arm a newly
         * entered AP: Windows has not installed that AP's KPCR/x18 yet, and an
         * early carrier tick would crash in KfRaiseIrql at x18 + 0x38.
         */
        /*
         * A timer write alone is too early on a Windows AP.  The AP bootstrap
         * programs its timer before installing the KPCR in x18; delivering an
         * LR in that interval enters KfRaiseIrql with x18 == 0 and bugchecks at
         * [x18 + 0x38].  Only arm this CPU's carrier after a trapped write also
         * proves that Windows has established its per-CPU kernel context.
         */
        /*
         * Keep the compatibility clock on the BSP.  Windows AP startup can
         * transiently populate x18 while programming its timer and then clear
         * it again before the first local tick.  SGIs still bring every AP
         * online through the carrier; local timer delivery begins only after
         * the native AIC handoff.
         */
        PERCPU(carrier_timer_ready) = guest_cpu_ready && smp_id() == boot_cpu_idx;
        PERCPU(timer_p_event_unread) = false;
        PERCPU(timer_v_event_unread) = false;
        hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
        if (physical) {
            PERCPU(timer_p_reflection_pending) = false;
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
        } else {
            PERCPU(timer_v_reflection_pending) = false;
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
        }
        return;
    }

    if (physical) {
        PERCPU(timer_p_reflection_pending) = false;
        PERCPU(timer_p_event_unread) = false;
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
    } else {
        PERCPU(timer_v_reflection_pending) = false;
        PERCPU(timer_v_event_unread) = false;
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
    }
    hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
    hv_native_aic_doorbell_sync();
}
#endif

#define SYSREG_MAP(sr, to)                                                                         \
    case SYSREG_ISS(sr):                                                                           \
        if (is_read)                                                                               \
            regs[rt] = _mrs(sr_tkn(to));                                                           \
        else                                                                                       \
            _msr(sr_tkn(to), regs[rt]);                                                            \
        return true;

#define SYSREG_PASS(sr)                                                                            \
    case SYSREG_ISS(sr):                                                                           \
        if (is_read)                                                                               \
            regs[rt] = _mrs(sr_tkn(sr));                                                           \
        else                                                                                       \
            _msr(sr_tkn(sr), regs[rt]);                                                            \
        return true;

/*
 * Per-CPU one-shot ID-register doorbell.
 *
 * HCR_EL2.TID3 traps the whole AArch64 ID group 3 space and this file answers
 * all of it at EL2, so nothing in that space reaches the proxy client any more.
 * The Windows debug module in the driver repo needs *some* deterministic point
 * at which it is called with a live guest context, per CPU, in order to install
 * its checkpoints; before this it abused the sheer volume of unhandled ID reads
 * as a clock, which is exactly what made boots slow.
 *
 * So forward the *first* ID group 3 read executed on each physical CPU to the
 * proxy, and answer every subsequent one at EL2.  That is a bounded MAX_CPUS
 * serial round trips for an entire boot instead of one per instruction, and it
 * is an ordering guarantee rather than a timing hope: a CPU cannot run guest
 * code without first running the kernel's own per-CPU feature detection, which
 * reads these registers.  The host therefore always gets a callback on a CPU
 * before that CPU can execute anything the host wants to intercept.
 *
 * ID_AA64PFR0_EL1 is deliberately NOT a doorbell.  Its case ORs in the GICv3
 * sysreg-interface bit, while the proxy services a trapped read by issuing its
 * own remote mrs and overwriting regs[rt]; forwarding it would silently discard
 * that bit.  Every other encoding here is a plain pass-through whose proxy
 * answer is bit-identical to the EL2 answer, so forwarding one is unobservable
 * to the guest.
 */
static bool hv_id_doorbell_rung[MAX_CPUS];

static bool hv_ring_id_doorbell(void)
{
    int cpu = smp_id();

    if (cpu < 0 || cpu >= MAX_CPUS)
        return false;
    if (hv_id_doorbell_rung[cpu])
        return false;

    hv_id_doorbell_rung[cpu] = true;
    return true;
}

/*
 * Identical to SYSREG_PASS, except that the first access on each CPU falls
 * through to hv_exc_proxy() so the host sees one deterministic callback per CPU.
 */
#define SYSREG_PASS_DOORBELL(sr)                                                                   \
    case SYSREG_ISS(sr):                                                                           \
        if (is_read && hv_ring_id_doorbell())                                                      \
            return false;                                                                          \
        if (is_read)                                                                               \
            regs[rt] = _mrs(sr_tkn(sr));                                                           \
        else                                                                                       \
            _msr(sr_tkn(sr), regs[rt]);                                                            \
        return true;

static bool hv_handle_msr_unlocked(struct exc_info *ctx, u64 iss)
{
    u64 reg = iss & (ESR_ISS_MSR_OP0 | ESR_ISS_MSR_OP2 | ESR_ISS_MSR_OP1 | ESR_ISS_MSR_CRn |
                     ESR_ISS_MSR_CRm);
    u64 rt = FIELD_GET(ESR_ISS_MSR_Rt, iss);
    bool is_read = iss & ESR_ISS_MSR_DIR;

    u64 *regs = ctx->regs;

    regs[31] = 0;

    switch (reg) {
        SYSREG_PASS(SYS_IMP_APL_CORE_NRG_ACC_DAT);
        SYSREG_PASS(SYS_IMP_APL_CORE_SRM_NRG_ACC_DAT);
        /* Architectural timer, for ECV */
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
#define SYSREG_MAP_REFLECT(sr, to, physical, control)                                               \
    case SYSREG_ISS(sr):                                                                           \
        if (is_read) {                                                                             \
            regs[rt] = _mrs(sr_tkn(to));                                                           \
        } else {                                                                                   \
            _msr(sr_tkn(to), regs[rt]);                                                            \
            hv_timer_reflect_guest_rearm(physical, control, regs[rt], regs[18] != 0);              \
        }                                                                                          \
        return true;

        SYSREG_MAP_REFLECT(SYS_CNTV_CTL_EL0, SYS_CNTV_CTL_EL02, false, true)
        SYSREG_MAP_REFLECT(SYS_CNTV_CVAL_EL0, SYS_CNTV_CVAL_EL02, false, false)
        SYSREG_MAP_REFLECT(SYS_CNTV_TVAL_EL0, SYS_CNTV_TVAL_EL02, false, false)
        SYSREG_MAP_REFLECT(SYS_CNTP_CTL_EL0, SYS_CNTP_CTL_EL02, true, true)
        SYSREG_MAP_REFLECT(SYS_CNTP_CVAL_EL0, SYS_CNTP_CVAL_EL02, true, false)
        SYSREG_MAP_REFLECT(SYS_CNTP_TVAL_EL0, SYS_CNTP_TVAL_EL02, true, false)
#undef SYSREG_MAP_REFLECT
#else
        SYSREG_MAP(SYS_CNTV_CTL_EL0, SYS_CNTV_CTL_EL02)
        SYSREG_MAP(SYS_CNTV_CVAL_EL0, SYS_CNTV_CVAL_EL02)
        SYSREG_MAP(SYS_CNTV_TVAL_EL0, SYS_CNTV_TVAL_EL02)
        SYSREG_MAP(SYS_CNTP_CTL_EL0, SYS_CNTP_CTL_EL02)
        SYSREG_MAP(SYS_CNTP_CVAL_EL0, SYS_CNTP_CVAL_EL02)
        SYSREG_MAP(SYS_CNTP_TVAL_EL0, SYS_CNTP_TVAL_EL02)
#endif
        /* Spammy stuff seen on t600x p-cores */
        /* These are PMU/PMC registers */
        SYSREG_PASS(sys_reg(3, 2, 15, 12, 0));
        SYSREG_PASS(sys_reg(3, 2, 15, 13, 0));
        SYSREG_PASS(sys_reg(3, 2, 15, 14, 0));
        SYSREG_PASS(sys_reg(3, 2, 15, 15, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 7, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 8, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 9, 0));
        SYSREG_PASS(sys_reg(3, 1, 15, 10, 0));
        /* Noisy traps */
        SYSREG_PASS(SYS_IMP_APL_HID4)
        SYSREG_PASS(SYS_IMP_APL_EHID4)
        /* We don't normally trap these, but if we do, they're noisy */
        SYSREG_PASS(SYS_IMP_APL_GXF_STATUS_EL1)
        SYSREG_PASS(SYS_IMP_APL_CNTVCT_ALIAS_EL0)
        SYSREG_PASS(SYS_IMP_APL_TPIDR_GL1)
        SYSREG_MAP(SYS_IMP_APL_SPSR_GL1, SYS_IMP_APL_SPSR_GL12)
        SYSREG_MAP(SYS_IMP_APL_ASPSR_GL1, SYS_IMP_APL_ASPSR_GL12)
        SYSREG_MAP(SYS_IMP_APL_ELR_GL1, SYS_IMP_APL_ELR_GL12)
        SYSREG_MAP(SYS_IMP_APL_ESR_GL1, SYS_IMP_APL_ESR_GL12)
        SYSREG_MAP(SYS_IMP_APL_SPRR_PERM_EL1, SYS_IMP_APL_SPRR_PERM_EL12)
        SYSREG_MAP(SYS_IMP_APL_APCTL_EL1, SYS_IMP_APL_APCTL_EL12)
        SYSREG_MAP(SYS_IMP_APL_AMX_CTL_EL1, SYS_IMP_APL_AMX_CTL_EL12)
        /* FIXME:Might be wrong */
        SYSREG_PASS(SYS_IMP_APL_AMX_STATE_T)
        /* pass through PMU handling */
        SYSREG_PASS(SYS_IMP_APL_PMCR1)
        SYSREG_PASS(SYS_IMP_APL_PMCR2)
        SYSREG_PASS(SYS_IMP_APL_PMCR3)
        SYSREG_PASS(SYS_IMP_APL_PMCR4)
        SYSREG_PASS(SYS_IMP_APL_PMESR0)
        SYSREG_PASS(SYS_IMP_APL_PMESR1)
        SYSREG_PASS(SYS_IMP_APL_PMSR)
#ifndef DEBUG_PMU_IRQ
        SYSREG_PASS(SYS_IMP_APL_PMC0)
#endif
        SYSREG_PASS(SYS_IMP_APL_PMC1)
        SYSREG_PASS(SYS_IMP_APL_PMC2)
        SYSREG_PASS(SYS_IMP_APL_PMC3)
        SYSREG_PASS(SYS_IMP_APL_PMC4)
        SYSREG_PASS(SYS_IMP_APL_PMC5)
        SYSREG_PASS(SYS_IMP_APL_PMC6)
        SYSREG_PASS(SYS_IMP_APL_PMC7)
        SYSREG_PASS(SYS_IMP_APL_PMC8)
        SYSREG_PASS(SYS_IMP_APL_PMC9)

        //spammy ntoskrnl regs
        SYSREG_PASS(SYS_IMP_APL_L2C_ERR_STS)

        SYSREG_PASS(sys_reg(2, 0, 0, 1, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 1, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 1, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 1, 7))

        SYSREG_PASS(sys_reg(2, 0, 0, 2, 2))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 2, 7))

        SYSREG_PASS(sys_reg(2, 0, 0, 3, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 3, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 3, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 3, 7))

        SYSREG_PASS(sys_reg(2, 0, 0, 4, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 4, 5))

        SYSREG_PASS(sys_reg(2, 0, 0, 5, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 5, 5))

        //these seem debugging related but looks like windbg/m1n1 debugger still works
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 4))
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 5))
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 6))
        SYSREG_PASS(sys_reg(2, 0, 0, 0, 7))
        SYSREG_PASS(sys_reg(2, 0, 1, 1, 4))

#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
        /*
         * The Apple cores implement the GIC virtualization registers at EL2 but
         * not a directly usable ICC_* interface at EL1.  During Windows' short
         * startup-carrier phase, ICH_HCR_EL2.TALL1 routes these accesses here so
         * they do not become illegal instructions.  No LR receives an interrupt;
         * this is register compatibility only, until Windows enables AIC2.
         */
        case SYSREG_ISS(ICC_IAR1_EL1):
            if(is_read) {
                if (hv_native_aic_windows_active() &&
                    !hv_native_aic_windows_ready() && regs[18] == 0) {
                    regs[rt] = 0x3ff;
                } else {
                    regs[rt] = hv_carrier_do_iar1();
                }
                if (regs[rt] != 0x3ff) {
                    u32 count = ++PERCPU(carrier_iar_count);
                    if (count <= 8)
                        printf("HV: carrier CPU %d IAR #%u INTID %lu x18=0x%lx "
                               "ELR=0x%lx\n",
                               smp_id(), count, regs[rt], regs[18], ctx->elr);
                }
            }
            return true;
        case SYSREG_ISS(ICC_IGRPEN1_EL1):
            if(is_read) {
                regs[rt] = hv_vgic3_get_igrpen1();
                printf("R: ICC_IGRPEN1_EL1: 0x%lx\n", regs[rt]);
            }
            else{
                hv_vgic3_set_igrpen1(regs[rt]);
                printf("W: ICC_IGRPEN1_EL1: 0x%lx\n", regs[rt]);
            }
            return true;
        case SYSREG_ISS(ICC_BPR1_EL1):
            if(is_read) {
                regs[rt] = 0;
                printf("R: ICC_BPR1_EL1: 0x%lx\n", regs[rt]);
            }
            else{
                printf("W: ICC_BPR1_EL1: 0x%lx\n", regs[rt]);
            }
            return true;
        case SYSREG_ISS(ICC_EOIR1_EL1):
            if(is_read) {
                regs[rt] = 0;
            }
            else{
                hv_carrier_do_eoir1(regs[rt] & ICH_LR_VIRTUAL_MASK);
                u32 count = ++PERCPU(carrier_eoi_count);
                if (count <= 8)
                    printf("HV: carrier CPU %d EOI #%u INTID %lu\n",
                           smp_id(), count, regs[rt] & ICH_LR_VIRTUAL_MASK);
            }
            return true;
#endif

#ifdef ENABLE_VGIC_MODULE
        //
        // windows-native-aic: this trap-and-emulate is kept exactly as-is (it's a
        // synchronous sysreg trap, unrelated to HCR_EL2.IMO/FMO physical-interrupt
        // routing, so nothing about this patch disables it), but it is now VESTIGIAL
        // for a genuinely-native-AIC guest. Windows' AppleAic-equivalent HAL extension
        // is expected to generate IPIs via AIC's own mechanism instead (see the Fast-IPI
        // design decision comment in hv_exc_fiq(), item 4 / docs/windows-native-aic.md),
        // never touching this GICv3 CPU-interface sysreg. Left in place because it is
        // harmless (only fires if the guest actually executes this instruction) and
        // removing it isn't necessary to satisfy this patch's scope.
        //
        /* m1n1_windows change - emulate SGIs */
        case SYSREG_ISS(ICC_SGI1R_EL1):
            if(is_read) {
                regs[rt] = 0;
            }
            else{
                u64 sgir_value = regs[rt];
                u32 aff1, aff2, aff3;
                u32 aff0_targets;
                int virq, irm, rs;

                rs = (sgir_value >> ICH_SGI_RS_SHIFT) & ICH_SGI_RS_MASK;
                irm = (sgir_value >> ICH_SGI_IRQMODE_SHIFT) & ICH_SGI_IRQMODE_MASK;
                virq = (sgir_value >> ICH_SGI_IRQ_SHIFT ) & ICH_SGI_IRQ_MASK;
                aff1 = ICH_SGI_AFF1(sgir_value);
                aff2 = ICH_SGI_AFF2(sgir_value);
                aff3 = ICH_SGI_AFF3(sgir_value);
                aff0_targets = sgir_value & ICH_SGI_TARGETLIST_MASK;

                /*
                 * GICR frames are dense but Apple's physical CPU IDs are not:
                 * J414s exposes frames 0..9 for CPU IDs 0..6,8,9,10.  Iterate
                 * the exact frame map so SGIs never target absent CPU7 or omit
                 * physical CPU10.
                 */
                for (u16 frame = 0; frame < hv_vgic3_num_cpus(); frame++) {
                    int cpu = hv_vgic3_cpu_for_frame(frame);
                    if (cpu < 0)
                        continue;
                    if(irm == ICH_SGI_TARGET_OTHERS){
                        if(smp_id() == cpu)
                            continue;
                    } else if(irm == ICH_SGI_TARGET_LIST){
                        if(aff0_targets == 0)
                            return false;
                        u64 mpidr =  smp_get_mpidr(cpu);

                        if(MPIDR_AFF3(mpidr) != aff3)
                            continue;
                        if(MPIDR_AFF2(mpidr) != aff2)
                            continue;
                        if(MPIDR_AFF1(mpidr) != aff1)
                            continue;
                        //
                        // ICC_SGI1R_EL1: TargetList bit n addresses the PE with
                        // Aff0 = (RS * 16) + n. rs was decoded above but never
                        // applied, so any SGI with RS != 0 matched the wrong
                        // cores (BIT(aff0) against an un-shifted 16-bit list).
                        //
                        if((int)(MPIDR_AFF0(mpidr) >> 4) != rs)
                            continue;
                        if(!(aff0_targets & BIT(MPIDR_AFF0(mpidr) & 0xf)))
                            continue;
                    } else{
                        return false;
                    }
                    virq_t pending = { 
                        .vintid = virq, 
                        .priority = hv_vgic3_get_priority_cpu(cpu, virq),
                        .active = false, 
                        .pending = true,
                        .hw_status = false,
                        .hw_irq = 0,
                    };
                    if (hv_sgi_queue_push(cpu, &pending))
                        smp_send_ipi(cpu);
                }
            }
            return true;
#endif
        /* m1n1_windows change - Trap the ARM standard PMU regs */
        case SYSREG_ISS(SYS_PMCR_EL0):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated = 0;
                u64 pmi_mask = PMCR0_IMODE_MASK;
                //
                // Are PMIs enabled? (affects bit 0 of PMCR equivalently)
                //
                if((pmcr0_value & pmi_mask) == (PMCR0_IMODE_FIQ)) {
                    calculated |= PMCR_E;
                }
                //
                // Bits 5:1 are 0 for now,  bits 6 and 7 are on always (long events always on)
                // and bit 9 is checked by PMCR0[20] (since it deals with freeze/overflow)
                //
                if((pmcr0_value & BIT(20)) != 0) {
                    calculated |= PMCR_FZO;
                }
                calculated |= ((BIT(6)) | (BIT(7)));
                printf("HV PMUv3 Redirect: mrs x%ld, PMCR_EL0 = 0x%lx\n", rt, calculated);
                regs[rt] = calculated;
            }
            else {
                //
                // Bits [63:10] will have writes discarded (mostly ARM spec, bit 32 due to lack of support)
                //
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                int cycle_reset_requested = 0;
                //
                // Bit 9 (stop count on overflow) writes affect bit 20 of APL_PMCR0
                //
                if((regs[rt] & BIT(9)) != 0) {
                    pmcr0_value |= BIT(20);
                }
                else {
                    pmcr0_value &= ~(BIT(20));
                }
                
                //
                // Writes to bits [6:3] unsupported since no way of expressing either with Apple PMUs.
                //
                // Writing bit 2 (cycle counter reset) implies setting PMC0 to 0, so check if that's the case.
                //
                if((regs[rt] & PMCR_C) != 0) {
                    cycle_reset_requested = 1;
                }
                //
                // Bit 1 is the same as bit 2 but for event counters, unimplemented for now.
                //
                // Bit 0 controls whether event counters are enabled globally, if this is being set,
                // the closest thing on Apple platforms is the IRQ mode so set that if bit 0 is requested.
                //
                if((regs[rt] & PMCR_E) != 0) {
                    pmcr0_value &= ~(PMCR0_IMODE_MASK);
                    pmcr0_value |= PMCR0_IMODE_FIQ;
                }
                else {
                    pmcr0_value &= ~(PMCR0_IMODE_MASK);
                    pmcr0_value |= PMCR0_IMODE_OFF;
                }
                sysop("isb");
                if(cycle_reset_requested == 1) {
                    pmcr0_value &= ~(BIT(12));
                    pmcr0_value &= ~(BIT(0));
                }
                msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                sysop("isb");
                if(cycle_reset_requested == 1) {
                    sysop("isb");
                    msr(SYS_IMP_APL_PMC0, 0);
                    sysop("isb");
                    pmcr0_value |= BIT(12);
                    pmcr0_value |= BIT(0);
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                }
                printf("HV PMUv3 Redirect (OK): msr PMCR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        SYSREG_MAP(SYS_PMCCNTR_EL0, SYS_IMP_APL_PMC0)
        case SYSREG_ISS(SYS_PMCCFILTR_EL0):
            if(is_read) {
                u64 pmcr1_value = mrs(SYS_IMP_APL_PMCR1);
                u64 calculated_value = 0;
                //
                // If EL0/EL1 counting is disabled, set bit 30 of PMCCFILTR to 1.
                // (This is backwards from how I would've done it but okay I suppose...)
                //
                if((pmcr1_value & BIT(8)) == 0) {
                    calculated_value |= BIT(30);
                }
                if((pmcr1_value & BIT(16)) == 0) {
                    calculated_value |= BIT(31);
                }
                //
                // EL2 counting always happens as far as is known, so bit 27 is always set to 1
                // (bit set to 1 in this case means disable filtering...not sure why it's backwards.)
                //
                calculated_value |= BIT(27);
                printf("HV PMUv3 Redirect: mrs x%ld, PMCCFILTR_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr1_value = mrs(SYS_IMP_APL_PMCR1);
                //
                // If we're being asked to disable counting cycles for a given EL, set the appropriate bit for
                // PMCR1 respectively.
                //
                if((regs[rt] & PMCCFILTR_P) == 0) {
                    pmcr1_value |= BIT(16);
                }
                else {
                    pmcr1_value &= ~(BIT(16));
                }
                if((regs[rt] & PMCCFILTR_U) == 0) {
                    pmcr1_value |= BIT(8);
                }
                else {
                    pmcr1_value &= ~(BIT(8));
                }
                sysop("isb");
                msr(SYS_IMP_APL_PMCR1, pmcr1_value);
                sysop("isb");
                printf("HV PMUv3 Redirect (OK): msr PMCCFILTR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMCEID0_EL0):
            //
            // Unimplemented for now, return 0 for a read, discard writes.
            //
            if(is_read) {
                regs[rt] = 0;
                printf("HV PMUv3 Redirect: mrs x%ld, PMCEID0_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                //
                // Do nothing here.
                //
                printf("HV PMUv3 Redirect (skipped write): msr PMCEID0_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMCEID1_EL0):
            //
            // Unimplemented for now, return 0 for a read, discard writes.
            //
            if(is_read) {
                regs[rt] = 0;
                printf("HV PMUv3 Redirect: mrs x%ld, PMCEID1_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                printf("HV PMUv3 Redirect (skipped write): msr PMCEID1_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMCNTENCLR_EL0):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // check what perf counters are enabled in PMCR0, and reflect that in the returned PMCNTENCLR/PMCNTENSET value
                //
                if((pmcr0_value & BIT(0)) != 0) {
                    calculated_value |= BIT(31);
                }
                printf("HV PMUv3 Redirect: mrs x%ld, PMCNTENCLR_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counters_disable_mask = GENMASK(31,0);
                //
                // check if any counters are being requested to be disabled (cycle counter only for now)
                // and deal with those here.
                //
                if((regs[rt] & counters_disable_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value &= ~(BIT(0));
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    printf("HV PMUv3 Redirect (OK): msr PMCNTENCLR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        case SYSREG_ISS(SYS_PMCNTENSET_EL0):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // check what perf counters are enabled in PMCR0, and reflect that in the returned PMCNTENCLR/PMCNTENSET value
                //
                if((pmcr0_value & BIT(0)) != 0) {
                    calculated_value |= BIT(31);
                }
                printf("HV PMUv3 Redirect: mrs x%ld, PMCNTENSET_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counters_enable_mask = GENMASK(31,0);
                //
                // check if any counters are being requested to be disabled (cycle counter only for now)
                // and deal with those here.
                //
                if((regs[rt] & counters_enable_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value |= BIT(0);
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    printf("HV PMUv3 Redirect (OK): msr PMCNTENSET_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        SYSREG_MAP(SYS_PMEVCNTR0_EL0, SYS_IMP_APL_PMC2)
        // case SYSREG_ISS(SYS_PMEVTYPER0_EL0):
        //     if(is_read) {
        //         printf("mrs(PMEVTYPER0_EL0)\n");
        //         regs[rt] = 0;
        //         int value = mrs(SYS_IMP_APL_PMCR1);
        //         if(value & GENMASK(23, 16)) {
        //             regs[rt] |= BIT(31); //privileged bit
        //         }
        //         if(value & GENMASK(15, 8)) {
        //             regs[rt] |= BIT(30); //user filter bit
        //         }
        //         regs[rt] |= (mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0));
        //     }
        //     else {
        //         int val = mrs(SYS_IMP_APL_PMCR1);
        //         int event = mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0);
        //         if(regs[rt] & PMEVTYPER_P) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): enabling el1 counting of event\n", regs[rt]);
        //             val |= BIT(16);
        //         }
        //         if(regs[rt] & GENMASK(7, 0)) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): setting event\n", regs[rt]);
        //             event |= regs[rt] & GENMASK(7, 0);
        //             msr(SYS_IMP_APL_PMESR0, event);
        //         }
        //         msr(SYS_IMP_APL_PMCR1, val);
        //     }
        //     return true;
        case SYSREG_ISS(SYS_PMINTENCLR_EL1):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // As before, cycle counter only for now. (bit 12 = PMI enabled for cycle counter)
                //
                if((pmcr0_value & BIT(12)) != 0) {
                    calculated_value |= BIT(31);
                }
                printf("HV PMUv3 Redirect: mrs x%ld, PMINTENCLR_EL1 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counter_irqs_disabled_mask = GENMASK(31,0);
                //
                // Cycle counter only for now (bits 19:12 control all PMIs for the PMUs)
                //
                if((regs[rt] & counter_irqs_disabled_mask) != 0) {
                    if((regs[rt] & (BIT(31))) != 0) {
                        pmcr0_value &= ~(BIT(12));
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    printf("HV PMUv3 Redirect (OK): msr PMINTENCLR_EL1, x%ld = 0x%lx\n", rt, regs[rt]);
                }

            }
            return true;
        case SYSREG_ISS(SYS_PMINTENSET_EL1):
            if(is_read) {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 calculated_value = 0;
                //
                // As before, cycle counter only for now. (bit 12 = PMI enabled for cycle counter)
                //
                if((pmcr0_value & BIT(12)) != 0) {
                    calculated_value |= BIT(31);
                }
                printf("HV PMUv3 Redirect: mrs x%ld, PMINTENSET_EL1 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counter_irqs_enabled_mask = GENMASK(31,0);
                //
                // Cycle counter only for now (bits 19:12 control all PMIs for the PMUs)
                //
                if((regs[rt] & counter_irqs_enabled_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value |= BIT(12);
                    }
                    sysop("isb");
                    msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                    sysop("isb");
                    printf("HV PMUv3 Redirect (OK): msr PMINTENSET_EL1, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        case SYSREG_ISS(SYS_PMMIR_EL1):
            //
            // return 0 for now, discard writes.
            //
            if(is_read) {
                regs[rt] = 0;
                printf("HV PMUv3 Redirect: mrs x%ld, PMMIR_EL1 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                printf("HV PMUv3 Redirect (skipped write): msr PMMIR_EL1, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        case SYSREG_ISS(SYS_PMOVSCLR_EL0):
            if(is_read) {
                //
                // Read the state of the PMSR register to see if a PMU has overflowed.
                // Cycle counter only for now.
                //
                u64 pmsr_value = mrs(SYS_IMP_APL_PMSR);
                u64 calculated_value = 0;
                u64 pmu_overflowed_mask = GENMASK(9, 0);
                if((pmsr_value & pmu_overflowed_mask) != 0) {
                    //
                    // bit 0 is for PMC 0
                    //
                    if((pmsr_value & BIT(0)) != 0) {
                        calculated_value |= BIT(31);
                    }
                }
                printf("HV PMUv3 Redirect: mrs x%ld, PMOVSCLR_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                //
                // To clear the overflow bit requires a reset of the PMC (to clear PMSR)
                // so we need to disable the counter and re-enable it with the bit set to 0.
                //
                u64 pmcr0_value = mrs(SYS_IMP_APL_PMCR0);
                u64 counter_overflow_mask = GENMASK(31,0);
                if((regs[rt] & counter_overflow_mask) != 0) {
                    if((regs[rt] & BIT(31)) != 0) {
                        pmcr0_value &= ~(BIT(12));
                        pmcr0_value &= ~(BIT(0));
                        sysop("isb");
                        msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                        sysop("isb");
                        sysop("isb");
                        msr(SYS_IMP_APL_PMC0, 0);
                        sysop("isb");
                        pmcr0_value |= BIT(12);
                        pmcr0_value |= BIT(0);
                        sysop("isb");
                        msr(SYS_IMP_APL_PMCR0, pmcr0_value);
                        sysop("isb");
                    }
                printf("HV PMUv3 Redirect (OK): msr PMOVSCLR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
                }
            }
            return true;
        case SYSREG_ISS(SYS_PMOVSSET_EL0):
            if(is_read) {
                //
                // Read the state of the PMSR register to see if a PMU has overflowed.
                // Cycle counter only for now.
                //
                u64 pmsr_value = mrs(SYS_IMP_APL_PMSR);
                u64 calculated_value = 0;
                u64 pmu_overflowed_mask = GENMASK(9, 0);
                if((pmsr_value & pmu_overflowed_mask) != 0) {
                    //
                    // bit 0 is for PMC 0
                    //
                    if((pmsr_value & BIT(0)) != 0) {
                        calculated_value |= BIT(31);
                    }
                }
                printf("HV PMUv3 Redirect: mrs x%ld, PMOVSSET_EL0 = 0x%lx\n", rt, calculated_value);
                regs[rt] = calculated_value;
            }
            else {
                //
                // For now, don't set the overflow bit. If this needs to change, reuse the code from before.
                //
            }
            return true;
        case SYSREG_ISS(SYS_PMSELR_EL0):
            //for now hardcode to set the cycle counter, this will very likely need to change
            if(is_read) {
                regs[rt] = 31;
                printf("HV PMUv3 Redirect: mrs x%ld, PMSELR_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                printf("HV PMUv3 Redirect (skipped write): msr PMSELR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
            }
            return true;
        //SYSREG_MAP(SYS_PMSWINC_EL0, SYS_IMP_APL_PMC3)
        case SYSREG_ISS(SYS_PMUSERENR_EL0):
            if(is_read) {
                regs[rt] = PERCPU(guest_pmuserenr);
                printf("HV PMUv3 Redirect: mrs x%ld, PMUSERENR_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                PERCPU(guest_pmuserenr) = regs[rt] & ~PMUSERENR_RESERVED;
                printf("HV PMUv3 Redirect (OK): msr PMUSERENR_EL0, x%ld = 0x%lx\n",
                       rt, PERCPU(guest_pmuserenr));
            }
           return true;
#ifdef ENABLE_VGIC_MODULE
        //
        // m1n1_windows change: since we're now going to be setting HCR_EL2.TID3 (to avoid maintaining a fork of ArmGicDxe in the Mu UEFI port)
        // we need to pass through all the other registers except ID_AA64PFR0_EL1 (because that register needs to have the bit OR'ed in that tells UEFI that we support
        // the GICv3 sysreg interface)
        // Note that this only applies if the vGIC is being used, these registers should not be trapped otherwise.
        //
        // windows-native-aic: HCR_EL2.TID3 stays set unconditionally in this mode too
        // (hv.c) and this pass-through/OR-in behavior is left unchanged. The GICv3 CPU
        // interface (ICH_*/ICC_*) is still enabled per-core even though the timer no
        // longer uses it for delivery (see hv_update_fiq(), which now reflects via an
        // AIC software IRQ instead) -- whatever UEFI/HAL logic probes
        // ID_AA64PFR0_EL1.GIC before the native AIC HAL extension takes over may still
        // rely on seeing this bit set. Whether Windows' own timer/HAL bring-up path
        // actually reads this field at all under a fully AIC-native design (as opposed
        // to the originally-planned GIC-PPI-for-the-timer hybrid this bit was added
        // for) is unconfirmed; left as-is rather than guessed at, see
        // docs/windows-native-aic.md.
        //
        SYSREG_PASS_DOORBELL(ID_AA64PFR1_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64DFR0_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64DFR1_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64ISAR0_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64ISAR1_EL1)
        SYSREG_PASS_DOORBELL(SYS_ID_AA64MMFR0_EL1)
        SYSREG_PASS_DOORBELL(SYS_ID_AA64MMFR1_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64AFR0_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64AFR1_EL1)
        //
        // The remainder of the AArch64 ID group 3 space.  TID3 traps all of it,
        // and anything not answered here falls through to hv_exc_proxy(), which
        // costs several serial round trips per instruction: the exception is
        // shipped to the proxy client, which issues its own remote mrs to read
        // the register and then logs the access.  Windows reads these constantly
        // during per-CPU bring-up and driver init, so leaving them unhandled
        // dominates boot time.  ID_AA64MMFR2_EL1 and ID_AA64ISAR2_EL1 were both
        // observed taking that path on ten-CPU boots.
        //
        SYSREG_PASS_DOORBELL(ID_AA64PFR2_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64ZFR0_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64SMFR0_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64ISAR2_EL1)
        SYSREG_PASS_DOORBELL(ID_AA64ISAR3_EL1)
        SYSREG_PASS_DOORBELL(SYS_ID_AA64MMFR2_EL1)
        SYSREG_PASS_DOORBELL(SYS_ID_AA64MMFR3_EL1)
        SYSREG_PASS_DOORBELL(SYS_ID_AA64MMFR4_EL1)
        case SYSREG_ISS(ID_AA64PFR0_EL1):
            if(is_read) {
                //
                // need to OR in bit 24 to the register.
                // the (1 << 24) here makes the guest see that the MSR is reporting the GICv3 sysreg interface
                //
                regs[rt] = _mrs(sr_tkn(ID_AA64PFR0_EL1)) | (1 << 24);
            }
            else {
                //
                // this register is architecturally RO, nothing should *ever* be attempting to write this.
                //
            }
            //
            // Return here rather than falling through to hv_exc_proxy().  The
            // proxy services a trapped read by issuing its own remote mrs over
            // the serial link and overwriting regs[rt], which both discarded
            // the GIC bit this case exists to set and cost several serial round
            // trips per instruction.  ID_AA64PFR0_EL1 is roughly four fifths of
            // all ID-register traffic during Windows' per-CPU bring-up, so this
            // is where the boot time went.
            //
            // This case is never a doorbell (see SYSREG_PASS_DOORBELL): the
            // proxy's answer differs from ours by exactly the GIC bit, so
            // forwarding even one access would hand the guest a value this case
            // exists to prevent.  The other ID encodings are plain pass-throughs
            // and carry the doorbell instead.
            //
            return true;
#endif
        // SYSREG_MAP(SYS_PMXEVCNTR_EL0, SYS_IMP_APL_PMC2)
        // case SYSREG_ISS(SYS_PMXEVTYPER_EL0):
        //     if(is_read) {
        //         printf("mrs(PMXEVTYPER_EL0)\n");
        //         regs[rt] = 0;
        //         int value = mrs(SYS_IMP_APL_PMCR1);
        //         if(value & GENMASK(23, 16)) {
        //             regs[rt] |= BIT(31); //privileged bit
        //         }
        //         if(value & GENMASK(15, 8)) {
        //             regs[rt] |= BIT(30); //user filter bit
        //         }
        //         regs[rt] |= (mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0));
        //     }
        //     else {
        //         int val = mrs(SYS_IMP_APL_PMCR1);
        //         int event = mrs(SYS_IMP_APL_PMESR0) & GENMASK(7, 0);
        //         if(regs[rt] & PMEVTYPER_P) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): enabling el1 counting of event\n", regs[rt]);
        //             val |= BIT(16);
        //         }
        //         if(regs[rt] & GENMASK(7, 0)) {
        //             printf("msr(PMEVTYPER0_EL0, 0x%08lx): setting event\n", regs[rt]);
        //             event |= regs[rt] & GENMASK(7, 0);
        //             msr(SYS_IMP_APL_PMESR0, event);
        //         }
        //         msr(SYS_IMP_APL_PMCR1, val);
        //     }
        //     return true;

        /*
         * Outer-shareable TLBI operations trap on Apple M2 even though the
         * guest legitimately selects them from its architectural feature
         * view.  Re-executing the trapped OS encoding at EL2 takes a nested
         * undefined exception.  Inner-shareable maintenance has the required
         * scope for this single-socket system and is implemented by the CPU,
         * so translate each OS operation to its IS counterpart.
         */
        case SYSREG_ISS(sys_reg(1, 0, 8, 1, 0)): // VMALLE1OS -> VMALLE1IS
            /* VMALLE1* is operandless and must encode Xt as XZR. */
            if (is_read)
                return false;
            sysop("tlbi vmalle1is");
            return true;
        SYSREG_MAP(sys_reg(1, 0, 8, 1, 1), sys_reg(1, 0, 8, 3, 1)) // VAE1OS -> VAE1IS
        SYSREG_MAP(sys_reg(1, 0, 8, 1, 2), sys_reg(1, 0, 8, 3, 2)) // ASIDE1OS -> ASIDE1IS
        SYSREG_MAP(sys_reg(1, 0, 8, 5, 1), sys_reg(1, 0, 8, 2, 1)) // RVAE1OS -> RVAE1IS

        case SYSREG_ISS(SYS_ACTLR_EL1):
            if (is_read) {
                if (cpu_features->actlr_el2)
                    regs[rt] = mrs(SYS_ACTLR_EL12);
                else
                    regs[rt] = mrs(SYS_IMP_APL_ACTLR_EL12);
            } else {
                if (cpu_features->actlr_el2)
                    msr(SYS_ACTLR_EL12, regs[rt]);
                else
                    msr(SYS_IMP_APL_ACTLR_EL12, regs[rt]);
            }
            return true;

        case SYSREG_ISS(SYS_IMP_APL_IPI_SR_EL1):
            if (is_read) {
                regs[rt] = (PERCPU(ipi_pending) & HV_GUEST_IPI_OUTSTANDING) ?
                               IPI_SR_PENDING : 0;
            } else if (regs[rt] & IPI_SR_PENDING) {
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
                if (hv_native_aic_windows_active()) {
                    /*
                     * The native HAL writes IPI_SR from its controller EOI.
                     * That is the first point at which NT has resolved vector
                     * E01 and run the IPI service path, so it is the commit
                     * boundary for exactly one INFLIGHT generation.  An init
                     * or deinit write with no INFLIGHT transaction is benign
                     * and must not discard a queued DELIVERABLE generation --
                     * including during the pre-CONFIG Windows carrier phase.
                     */
                    hv_guest_ipi_commit_delivery();
                    hv_native_aic_doorbell_sync();
                } else
#endif
                {
                    /* Preserve the original physical-register semantics. */
                    PERCPU(ipi_pending) = 0;
                }
            }
            return true;

        /* shadow the interrupt mode and state flag */
        case SYSREG_ISS(SYS_IMP_APL_PMCR0):
            if (is_read) {
                u64 val = (mrs(SYS_IMP_APL_PMCR0) & ~PMCR0_IMODE_MASK) | PERCPU(pmc_irq_mode);
                regs[rt] = val | (PERCPU(pmc_pending) ? PMCR0_IACT : 0);
            } else {
                PERCPU(pmc_pending) = !!(regs[rt] & PMCR0_IACT);
                PERCPU(pmc_irq_mode) = regs[rt] & PMCR0_IMODE_MASK;
                msr(SYS_IMP_APL_PMCR0, regs[rt]);
            }
            return true;

        /*
         * Handle this one here because m1n1/Linux (will) use it for explicit cpuidle.
         * We can pass it through; going into deep sleep doesn't break the HV since we
         * don't do any wfis that assume otherwise in m1n1. However, don't het macOS
         * disable WFI ret (when going into systemwide sleep), since that breaks things.
         */
        case SYSREG_ISS(SYS_IMP_APL_CYC_OVRD):
            if (is_read) {
                regs[rt] = mrs(SYS_IMP_APL_CYC_OVRD);
            } else {
                u64 value = regs[rt];

                if (value & (CYC_OVRD_DISABLE_WFI_RET | CYC_OVRD_FIQ_MODE_MASK))
                    return false;
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
                /*
                 * Windows may program the remaining CYC_OVRD policy after
                 * secondary startup. Keep the native-AIC invariant installed
                 * by hv_configure_guest_wfi_mode(): mode 2 is the Apple
                 * clock-gate-only mode that preserves architectural registers
                 * across WFI. A trapped guest write must not silently restore
                 * mode 0 and reintroduce the x18/KPCR loss.
                 */
                value &= ~CYC_OVRD_WFI_MODE_MASK;
                value |= CYC_OVRD_WFI_MODE(2);
#endif
                msr(SYS_IMP_APL_CYC_OVRD, value);
                sysop("isb");
            }
            return true;
            /* clang-format off */
        /* IPI handling */
        SYSREG_PASS(SYS_IMP_APL_IPI_CR_EL1)
        /* M1RACLES reg, handle here due to silly 12.0 "mitigation" */
        case SYSREG_ISS(sys_reg(3, 5, 15, 10, 1)):
            if (is_read)
                regs[rt] = 0;
            return true;
    }
    return false;
}

static bool hv_handle_smc(struct exc_info *ctx) {
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    if (ctx->regs[0] == HV_TIMER_REFLECT_CALL_MAGIC &&
        (ctx->regs[1] == 17 || ctx->regs[1] == 18)) {
        hv_timer_reflect_guest_rearm(ctx->regs[1] == 17, false, 0, true);
        return true;
    }
#endif
    printf("PSCI SMC DEBUG: handling PSCI request 0x%lx\n", ctx->regs[0]);
    bool handled_smc = hv_handle_psci_smc(ctx);
    return handled_smc;
}

static bool hv_handle_msr(struct exc_info *ctx, u64 iss)
{
    u64 reg = iss & (ESR_ISS_MSR_OP0 | ESR_ISS_MSR_OP2 | ESR_ISS_MSR_OP1 | ESR_ISS_MSR_CRn |
                     ESR_ISS_MSR_CRm);
    u64 rt = FIELD_GET(ESR_ISS_MSR_Rt, iss);
    bool is_read = iss & ESR_ISS_MSR_DIR;

    u64 *regs = ctx->regs;

    regs[31] = 0;

    switch (reg) {
        /* clang-format on */
        case SYSREG_ISS(SYS_IMP_APL_IPI_RR_LOCAL_EL1): {
            assert(!is_read);
            u64 mpidr = (regs[rt] & 0xff) | (mrs(MPIDR_EL1) & 0xffff00);
            for (int i = 0; i < MAX_CPUS; i++)
                if (mpidr == smp_get_mpidr(i)) {
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
                    __atomic_fetch_add(&pcpu[i].guest_ipi_send_count, 1,
                                       __ATOMIC_RELAXED);
                    if (hv_guest_ipi_queue_pre_config_carrier(i))
                        return true;
#endif
                    __atomic_store_n(&pcpu[i].ipi_queued, true,
                                     __ATOMIC_RELEASE);
                    /*
                     * The target can take and clear the physical Fast-IPI as
                     * soon as the system-register write is visible.  Publish
                     * the guest-origin tag first; otherwise the target may
                     * observe the FIQ before this cache line and discard the
                     * Windows IPI as an EL2-only rendezvous.
                     */
                    sysop("dsb sy");
                    msr(SYS_IMP_APL_IPI_RR_LOCAL_EL1, regs[rt]);
                    return true;
                }
            return false;
        }
        case SYSREG_ISS(SYS_IMP_APL_IPI_RR_GLOBAL_EL1):
            assert(!is_read);
            u64 mpidr = (regs[rt] & 0xff) | ((regs[rt] & 0xff0000) >> 8);
            for (int i = 0; i < MAX_CPUS; i++) {
                if (mpidr == (smp_get_mpidr(i) & 0xffff)) {
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
                    __atomic_fetch_add(&pcpu[i].guest_ipi_send_count, 1,
                                       __ATOMIC_RELAXED);
                    if (hv_guest_ipi_queue_pre_config_carrier(i))
                        return true;
#endif
                    __atomic_store_n(&pcpu[i].ipi_queued, true,
                                     __ATOMIC_RELEASE);
                    sysop("dsb sy");
                    msr(SYS_IMP_APL_IPI_RR_GLOBAL_EL1, regs[rt]);
                    return true;
                }
            }
            return false;
#ifdef DEBUG_PMU_IRQ
        case SYSREG_ISS(SYS_IMP_APL_PMC0):
            if (is_read) {
                regs[rt] = mrs(SYS_IMP_APL_PMC0);
            } else {
                msr(SYS_IMP_APL_PMC0, regs[rt]);
                printf("msr(SYS_IMP_APL_PMC0, 0x%04lx_%08lx)\n", regs[rt] >> 32,
                       regs[rt] & 0xFFFFFFFF);
            }
            return true;
#endif
    }

    return false;
}

static void hv_get_context(struct exc_info *ctx)
{
    ctx->spsr = hv_get_spsr();
    ctx->elr = hv_get_elr();
    ctx->esr = hv_get_esr();
    ctx->far = hv_get_far();
    ctx->afsr1 = hv_get_afsr1();
    ctx->sp[0] = mrs(SP_EL0);
    ctx->sp[1] = mrs(SP_EL1);
    ctx->sp[2] = (u64)ctx;
    ctx->cpu_id = smp_id();
    ctx->mpidr = mrs(MPIDR_EL1);

    sysop("isb");
}

static void hv_exc_entry(void)
{
    // Enable SErrors in the HV, but only if not already pending
    if (!(mrs(ISR_EL1) & 0x100))
        sysop("msr daifclr, 4");

    __atomic_and_fetch(&hv_cpus_in_guest, ~BIT(smp_id()), __ATOMIC_ACQUIRE);
    spin_lock(&bhl);
    hv_wdt_breadcrumb('X');
    exc_entry_time = mrs(CNTPCT_EL0);
    /* disable PMU counters in the hypervisor */
    u64 pmcr0 = mrs(SYS_IMP_APL_PMCR0);
    PERCPU(exc_entry_pmcr0_cnt) = pmcr0 & PMCR0_CNT_MASK;
    msr(SYS_IMP_APL_PMCR0, pmcr0 & ~PMCR0_CNT_MASK);
}

static void hv_exc_exit(struct exc_info *ctx)
{
    hv_wdt_breadcrumb('x');
    hv_carrier_observe_x18(ctx);
    hv_update_fiq(ctx);
    /* reenable PMU counters */
    reg_set(SYS_IMP_APL_PMCR0, PERCPU(exc_entry_pmcr0_cnt));
    msr(CNTVOFF_EL2, stolen_time);
    spin_unlock(&bhl);
    hv_maybe_exit();
    __atomic_or_fetch(&hv_cpus_in_guest, BIT(smp_id()), __ATOMIC_ACQUIRE);

    hv_set_spsr(ctx->spsr);
    hv_set_elr(ctx->elr);
    msr(SP_EL0, ctx->sp[0]);
    msr(SP_EL1, ctx->sp[1]);
}

void hv_exc_sync(struct exc_info *ctx)
{
    hv_wdt_breadcrumb('S');
    hv_get_context(ctx);
    bool handled = false;
    u32 ec = FIELD_GET(ESR_EC, ctx->esr);

    switch (ec) {
        case ESR_EC_MSR:
            hv_wdt_breadcrumb('m');
            handled = hv_handle_msr_unlocked(ctx, FIELD_GET(ESR_ISS, ctx->esr));
            break;
        //
        // for Blizzard/Avalanche and later - we need to explicitly check for SMC EC to handle SMCs
        //
        case ESR_EC_SMC:
            hv_wdt_breadcrumb('s');
            handled = hv_handle_smc(ctx);
            break;
        case ESR_EC_IMPDEF:
            hv_wdt_breadcrumb('a');
            if(ctx->afsr1 == 0x1c00000) {
                /**
                 * m1n1_windows change: add SMC handling support.
                 * 
                 * right now the only reason a guest OS would fire an SMC is due to 
                 * requesting a PSCI service.
                */
                handled = hv_handle_smc(ctx);
                break;
            }
            switch (FIELD_GET(ESR_ISS, ctx->esr)) {
                case ESR_ISS_IMPDEF_MSR:
                    handled = hv_handle_msr_unlocked(ctx, ctx->afsr1);
                    break;
            }
            break;
    }

    if (handled) {
        hv_wdt_breadcrumb('#');
        ctx->elr += 4;
        hv_set_elr(ctx->elr);
        hv_update_fiq(ctx);
        hv_wdt_breadcrumb('s');
        return;
    }

    hv_exc_entry();

    switch (ec) {
        case ESR_EC_DABORT_LOWER:
            hv_wdt_breadcrumb('D');
            handled = hv_handle_dabort(ctx);
            break;
        case ESR_EC_MSR:
            hv_wdt_breadcrumb('M');
            handled = hv_handle_msr(ctx, FIELD_GET(ESR_ISS, ctx->esr));
            break;
        case ESR_EC_IMPDEF:
            hv_wdt_breadcrumb('A');
            switch (FIELD_GET(ESR_ISS, ctx->esr)) {
                case ESR_ISS_IMPDEF_MSR:
                    handled = hv_handle_msr(ctx, ctx->afsr1);
                    break;
            }
            break;
    }

    if (handled) {
        hv_wdt_breadcrumb('+');
        ctx->elr += 4;
    } else {
        hv_wdt_breadcrumb('-');
        // VM code can forward a nested SError exception here
        if (FIELD_GET(ESR_EC, ctx->esr) == ESR_EC_SERROR)
            hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_SERROR, NULL);
        else
            hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_SYNC, NULL);
    }

    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('s');
}

void hv_exc_irq(struct exc_info *ctx)
{
    hv_carrier_observe_x18(ctx);
#ifdef ENABLE_VGIC_MODULE
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    /*
     * AIC2 CONFIG is enabled on the last processor to finish the deferred HAL
     * handoff.  Other processors may still have their per-CPU HCR.IMO set.
     * Do not acknowledge their pending AIC EVENT at EL2: clear IMO and return,
     * allowing the still-pending physical IRQ to be taken again by Windows EL1.
     */
    if (hv_native_aic_active()) {
        if (hv_native_aic_windows_ready() &&
            PERCPU(carrier_irq_active)) {
            /*
             * The carrier IAR has already been returned to EL1.  Preserve the
             * software CPU-interface state and TALL1 until its EOIR trap, but
             * release this native physical IRQ to Windows by clearing IMO.
             * Calling hv_native_aic_enter_cpu() here would disable TALL1 and
             * strand carrier_irq_active permanently.
             */
            hv_write_hcr(mrs(HCR_EL2) & ~(HCR_IMO | HCR_VI));
            return;
        }
        hv_native_aic_enter_cpu();
        /*
         * The real native source remains level-asserted until Windows reads
         * AIC EVENT.  Restoring IMO here would take the same IRQ at EL2 again
         * before EL1 executes a single instruction.  Preserve all synthetic
         * pending state until the trapped AIC EVENT read proves that Windows
         * accepted and auto-masked the source.  Keep m1n1's private tick armed
         * so debugger/proxy polling continues while that handoff is pending.
         */
        if (hv_native_aic_windows_ready()) {
            native_irq_rearm_deferred[smp_id()] = true;
            native_irq_bounce_count[smp_id()]++;
            hv_arm_tick(false);
        }
        return;
    }
#endif
    //
    // windows-native-aic: under ENABLE_NATIVE_AIC_PASSTHROUGH, hv.c clears
    // HCR_EL2.IMO specifically so ordinary AIC-routed physical IRQs go straight to the
    // guest at EL1 -- this vector should not fire for them at all anymore. It is left
    // fully intact below (unreachable-in-theory, not deleted) because:
    //  (a) the GICv3 virtual-CPU-interface maintenance interrupt's routing relative to
    //      HCR_EL2.IMO on Apple Silicon specifically is UNVERIFIED here (see
    //      docs/windows-native-aic.md OQ-3) -- on textbook ARM systems it follows the
    //      same physical-IRQ routing controls as any other physical interrupt, which
    //      would mean it stops trapping to EL2 too, but this file's own existing
    //      `type == 0` heuristic below (with its own "?" in the original comment)
    //      shows even the author of this vector was not fully certain how Apple wires
    //      it through AIC; and
    //  (b) if that assumption is wrong, or anything else still forces physical-IRQ
    //      trapping, we must not silently do nothing here -- see the fail-closed
    //      fallback replacing the old AIC-IRQ-to-vGIC-injection tail, below.
    //
    u32 reason = aic_ack();
    u32 irq = FIELD_GET(AIC_EVENT_NUM, reason);
    int type = FIELD_GET(AIC_EVENT_TYPE, reason);

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    /*
     * AIC EVENT state can outlive a RAM chainload.  A timer-reflector SW IRQ
     * that was posted by the previous guest may therefore already be latched
     * when the new startup carrier begins.  It must not be translated into a
     * vGIC LR: unlike a real level interrupt, the startup guest has no native
     * AIC EOI path that can clear the SW-pending bit, so the same event would
     * be acknowledged and injected forever.
     */
    if (!hv_native_aic_active() && irq >= HV_TIMER_SWIRQ_BASE &&
        irq < HV_TIMER_SWIRQ_BASE + (2 * MAX_CPUS)) {
        aic_set_mask(irq, true);
        aic_set_sw(irq, false);
        printf("HV: windows-native-aic: discarded stale startup reflector IRQ %u\n", irq);
        return;
    }
#endif

    u64 misr = mrs(ICH_MISR_EL2);
    u64 eisr = mrs(ICH_EISR_EL2);

    if(type == 0){//maintenance IRQ?
        if(misr != 0 && eisr != 0){
            for(int lr = 0; lr < (int)hv_vgic3_num_lrs(); lr++){
                if(eisr & BIT(lr)){
                    u64 lr_val = hv_vgic3_read_lr(lr);
                    u64 intd = (lr_val >> ICH_LR_VIRTUAL_SHIFT) & ICH_LR_VIRTUAL_MASK;
                    hv_vgic3_write_lr(lr, 0);
                    if(intd > 31)
                        aic_set_mask(intd, false);//TODO: check distributor
                }
            }
        }

        //
        // windows-native-aic: timer_queue has no producer anymore under
        // ENABLE_NATIVE_AIC_PASSTHROUGH -- the timer FIQ reflector (hv_update_fiq(),
        // above) posts a per-CPU AIC software IRQ directly, it no longer pushes onto
        // this queue or injects via list register. This drain is therefore a
        // permanent no-op in that mode; left in place unconditionally rather than
        // #ifndef'd out to minimize the diff and because it is harmless (an always-empty
        // pop loop). It stays fully live (and load-bearing) when
        // ENABLE_NATIVE_AIC_PASSTHROUGH is off, i.e. the original vGIC-distributor mode.
        //
        while(hv_vgic3_get_free_lr() != -1){
            virq_t pending;
            if (!virq_queue_pop(&PERCPU(timer_queue), &pending))
                break;
            hv_vgic3_inject_irq(
                pending.vintid,
                pending.priority,
                pending.active,
                pending.pending,
                pending.hw_status,
                pending.hw_irq
            );
        }
        //
        // windows-native-aic: sgi_queue still has a producer -- the pre-existing,
        // vestigial ICC_SGI1R_EL1 trap-and-emulate below (case SYSREG_ISS(ICC_SGI1R_EL1))
        // pushes onto it regardless of ENABLE_NATIVE_AIC_PASSTHROUGH. Whether this drain
        // point is still reachable to service it depends on the same maintenance-interrupt
        // routing question raised above; hv_exc_fiq()'s Fast-IPI-arrival handling also
        // drains it independently (see there), so this is not the only retry point.
        //
        while(hv_vgic3_get_free_lr() != -1){
            virq_t pending;
            if (!hv_sgi_queue_pop(&pending))
                break;
            hv_vgic3_inject_irq(
                pending.vintid,
                pending.priority,
                pending.active,
                pending.pending,
                pending.hw_status,
                pending.hw_irq
            );
        }
        while(hv_vgic3_get_free_lr() != -1){
            virq_t pending;
            if (!virq_queue_pop(&PERCPU(irq_queue), &pending))
                break;
            hv_vgic3_inject_irq(
                pending.vintid,
                pending.priority,
                pending.active,
                pending.pending,
                pending.hw_status,
                pending.hw_irq
            );
        }
        return;
    }

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    if (hv_native_aic_windows_active() && !hv_native_aic_windows_ready()) {
        virq_t pending = {
            .vintid = irq,
            .priority = hv_vgic3_get_priority(irq),
            .active = false,
            .pending = true,
            .hw_status = false,
            .hw_irq = 0,
        };
        virq_queue_push(&PERCPU(irq_queue), &pending);
        if (ctx->regs[18] == 0 || !PERCPU(carrier_stack_ready))
            printf("HV: windows-native-aic: carrier deferred CPU %d IRQ %u "
                   "until KPCR/panic-stack readiness\n", smp_id(), irq);
    }
    else
#endif
    if(hv_vgic3_get_free_lr() != -1){
        hv_vgic3_inject_irq(
            irq,                         //vintid
            hv_vgic3_get_priority(irq),  //priority
            false,                       //active
            true,                        //pending
            false,                       //hw_status
            0                            //hw_irq
        );
    }
    else{
        virq_t pending = {
            .vintid = irq,
            .priority = hv_vgic3_get_priority(irq),
            .active = false,
            .pending = true,
            .hw_status = false,
            .hw_irq = 0,
        };
        virq_queue_push(&PERCPU(irq_queue), &pending);
    }
#else
    hv_wdt_breadcrumb('I');
    hv_get_context(ctx);
    hv_exc_entry();
    hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_IRQ, NULL);
    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('i');
#endif
}

void hv_exc_fiq(struct exc_info *ctx)
{
    bool tick = false;

    hv_maybe_exit();

    //
    // windows-native-aic: the stale TODO that used to sit here ("inject the FIQ to the
    // guest as an IRQ if vGIC is enabled") is done, but NOT in this function -- the
    // CNTP_CTL_EL0/CNTV_CTL_EL0 reads immediately below are m1n1's OWN internal
    // periodic tick (hv_arm_tick(), HV_TICK_RATE/HV_SLOW_TICK_RATE in hv.c) and the
    // host-debugger HV_VTIMER proxy event, not the guest's virtualized timer. The
    // guest's virtualized CNTP/CNTV (CNTx_CTL_EL02) FIQ-to-AIC-software-IRQ reflection
    // lives in hv_update_fiq() (called via hv_exc_exit() at the bottom of this
    // function, and directly for the non-interruptible-CPU fast path below) -- see
    // docs/windows-native-aic.md "Timer re-arm handshake".
    //

    if (mrs(CNTP_CTL_EL0) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        msr(CNTP_CTL_EL0, CNTx_CTL_ISTATUS | CNTx_CTL_IMASK | CNTx_CTL_ENABLE);
        tick = true;
    }

    int interruptible_cpu = hv_pinned_cpu;
    if (interruptible_cpu == -1)
        interruptible_cpu = boot_cpu_idx;

    /*
     * Recover a guest-origin Fast-IPI tag even when its physical IPI latch was
     * already acknowledged by a racing FIQ.  Every FIQ is a safe owner-CPU
     * recovery point; the second exchange below closes the arrival window
     * around the physical acknowledge.
     */
    bool guest_ipi_taken = hv_guest_ipi_take_tag();
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    /*
     * The target FIQ is the only guaranteed EL2 entry for this Fast-IPI.
     * Post the native virtual-IRQ doorbell immediately after consuming a
     * post-CONFIG guest tag; an AP can mask FIQs as soon as it returns to
     * KiInitializeKernel, so deferring the CPU-local HCR.VI update to a later
     * timer/exception entry can strand an otherwise DELIVERABLE transaction.
     * hv_update_fiq() below remains the level-maintenance path and will keep
     * the same doorbell asserted until EVENT begins the transaction.
     */
    if (guest_ipi_taken && hv_native_aic_windows_ready())
        hv_native_aic_doorbell_sync();
    if (tick)
        hv_guest_ipi_retry_tick();
#endif

    if (smp_id() != interruptible_cpu && !(mrs(ISR_EL1) & 0x40) && hv_want_cpu == -1) {
        // Non-interruptible CPU and it was just a timer tick (or spurious), so just update FIQs
        hv_get_context(ctx);
        hv_update_fiq(ctx);
        hv_arm_tick(true);
        return;
    }

    // Slow (single threaded) path
    hv_wdt_breadcrumb('F');
    hv_get_context(ctx);
    hv_carrier_observe_x18(ctx);
    hv_exc_entry();

    // Only poll for HV events in the interruptible CPU
    if (tick) {
        if (smp_id() == interruptible_cpu) {
            hv_tick(ctx);
            hv_arm_tick(false);
        } else {
            hv_arm_tick(true);
        }
    }

    if (mrs(CNTV_CTL_EL0) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
        msr(CNTV_CTL_EL0, CNTx_CTL_ISTATUS | CNTx_CTL_IMASK | CNTx_CTL_ENABLE);
        hv_exc_proxy(ctx, START_HV, HV_VTIMER, NULL);
    }

    //
    // windows-native-aic PMU-FIQ design decision (item 4, docs/windows-native-aic.md
    // "PMU / Fast-IPI FIQ handling"): fail-closed, unchanged from before this patch.
    // The physical PMU FIQ source is masked here (IACT + IMODE cleared) exactly as it
    // always was, and PERCPU(pmc_pending) is only ever exposed to the guest via the
    // pre-existing trapped MSR read of SYS_IMP_APL_PMCR0 (hv_handle_msr_unlocked(),
    // case SYSREG_ISS(SYS_IMP_APL_PMCR0)) -- nothing wakes the guest up to look at it;
    // there is no vGIC-injection or AIC-software-IRQ path for it either before or
    // after this patch. This is deliberate: the task scope for this transform is the
    // timer only, and inventing a new PMU-interrupt reflection here was explicitly out
    // of scope. Note cpu_regs.h:461 also defines PMCR0_IMODE_AIC (route the PMU
    // interrupt through AIC as an ordinary IRQ instead of FIQ) as an existing hardware
    // option nothing in this codebase currently requests -- switching the guest-PMU
    // emulation (hv_handle_msr_unlocked(), case SYSREG_ISS(SYS_PMCR_EL0)) to request
    // that mode instead of PMCR0_IMODE_FIQ could let PMU interrupts bypass EL2 entirely
    // under this patch's HCR_EL2.IMO=0, the same way ordinary peripheral IRQs do; flagged
    // as a future open question, not implemented here (unvalidated against whatever a
    // guest PMU driver expects).
    //
    u64 reg = mrs(SYS_IMP_APL_PMCR0);
    if ((reg & (PMCR0_IMODE_MASK | PMCR0_IACT)) == (PMCR0_IMODE_FIQ | PMCR0_IACT)) {
#ifdef DEBUG_PMU_IRQ
        printf("[FIQ] PMC IRQ, masking and delivering to the guest\n");
#endif
        reg_clr(SYS_IMP_APL_PMCR0, PMCR0_IACT | PMCR0_IMODE_MASK);
        PERCPU(pmc_pending) = true;
    }

    reg = mrs(SYS_IMP_APL_UPMCR0);
    if (FIELD_GET(UPMCR0_IMODE_T8020, reg) == UPMCR0_IMODE_FIQ &&
        (mrs(SYS_IMP_APL_UPMSR) & UPMSR_IACT)) {
        printf("[FIQ] UPMC IRQ, masking");
        reg_clr(SYS_IMP_APL_UPMCR0, UPMCR0_IMODE_T8020);
        hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_FIQ, NULL);
    }

    /*
     * Apple's Fast IPI is FIQ-class and shared with m1n1's own rendezvous
     * mechanism. Guest IPI_RR writes are therefore trapped, tagged, and
     * relayed as physical Fast IPIs. The target acknowledges the physical
     * edge here, but that edge is only transport: after CONFIG handoff the
     * guest tag becomes a transactional synthetic AIC IPI EVENT driven by
     * HCR.VI. EVENT accept marks it INFLIGHT; the HAL's later IPI_SR write at
     * controller EOI commits it. A local-tick retry can re-emit an uncommitted
     * EVENT without consuming a newer DELIVERABLE generation.
     */
    if (mrs(SYS_IMP_APL_IPI_SR_EL1) & IPI_SR_PENDING) {
        /*
         * Acknowledge the physical edge before consuming the guest tag.  If a
         * sender publishes a new tag while this FIQ is in flight, it either
         * re-raises IPI_SR after this acknowledge or is consumed by the
         * exchange below.  The previous consume-then-ack order could clear a
         * newly-arrived edge while stranding its tag forever.
         */
        msr(SYS_IMP_APL_IPI_SR_EL1, IPI_SR_PENDING);
        sysop("isb");
#ifdef ENABLE_VGIC_MODULE
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
        if (!hv_native_aic_windows_active()) {
#endif
            while(hv_vgic3_get_free_lr() != -1){//another CPU sent an IPI, check the sgi_queue
                virq_t pending;
                if (!hv_sgi_queue_pop(&pending))
                    break;
                hv_vgic3_inject_irq(
                    pending.vintid,
                    pending.priority,
                    pending.active,
                    pending.pending,
                    pending.hw_status,
                    pending.hw_irq
                );
            }
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
        }
#endif
#endif
        hv_guest_ipi_take_tag();
    }

    hv_maybe_switch_cpu(ctx, START_HV, HV_CPU_SWITCH, NULL);

    // Handles guest timers
    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('f');
}

void hv_exc_serr(struct exc_info *ctx)
{
    hv_wdt_breadcrumb('E');
    hv_get_context(ctx);
    hv_exc_entry();
    hv_exc_proxy(ctx, START_EXCEPTION_LOWER, EXC_SERROR, NULL);
    hv_exc_exit(ctx);
    hv_wdt_breadcrumb('e');
}
