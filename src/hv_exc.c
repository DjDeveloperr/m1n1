/* SPDX-License-Identifier: MIT */

#include "hv.h"
#include "assert.h"
#include "cpu_regs.h"
#include "exception.h"
#include "smp.h"
#include "string.h"
#include "uart.h"
#include "uartproxy.h"
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
#endif

struct hv_pcpu_data {
    u32 ipi_queued;
    u32 ipi_pending;
    u32 pmc_pending;
    u64 pmc_irq_mode;
    u64 exc_entry_pmcr0_cnt;
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
    bool carrier_timer_ready;
    bool carrier_x18_repair_logged;
#endif
} ALIGNED(64);

struct hv_pcpu_data pcpu[MAX_CPUS];

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
        PERCPU_N(cpu, timer_p_reflection_pending) = false;
        PERCPU_N(cpu, timer_v_reflection_pending) = false;
        PERCPU_N(cpu, timer_p_event_unread) = false;
        PERCPU_N(cpu, timer_v_event_unread) = false;
        PERCPU_N(cpu, carrier_timer_ready) = false;
        PERCPU_N(cpu, carrier_x18_repair_logged) = false;
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
    }
    PERCPU(timer_p_reflection_pending) = false;
    PERCPU(timer_v_reflection_pending) = false;
    PERCPU(timer_p_event_unread) = false;
    PERCPU(timer_v_event_unread) = false;
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
        PERCPU_N(cpu, carrier_timer_ready) = false;
    }
    hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
    reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2,
            VM_TMR_FIQ_ENA_ENA_P | VM_TMR_FIQ_ENA_ENA_V);
    printf("HV: windows-native-aic: holding timer bridge until Windows enables AIC2\n");
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

static void hv_carrier_repair_x18(struct exc_info *ctx)
{
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    if (ctx == NULL || !hv_native_aic_windows_active() ||
        hv_native_aic_windows_ready() ||
        (FIELD_GET(SPSR_M, ctx->spsr) >> 2) != 1 || ctx->regs[18] != 0)
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

    if (!PERCPU(carrier_x18_repair_logged)) {
        printf("HV: windows-native-aic: repaired carrier CPU %d x18 from "
               "tag-stripped TPIDR_EL1=0x%lx at ELR=0x%lx\n",
               smp_id(), guest_pcr, ctx->elr);
        PERCPU(carrier_x18_repair_logged) = true;
    }
    ctx->regs[18] = guest_pcr;
#else
    (void)ctx;
#endif
}

static void hv_carrier_drain_pending(struct exc_info *ctx)
{
#if defined(ENABLE_VGIC_MODULE) && defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    hv_carrier_repair_x18(ctx);
    if (!hv_native_aic_windows_active() || hv_native_aic_windows_ready() ||
        ctx == NULL || ctx->regs[18] == 0)
        return;

    while (hv_vgic3_get_free_lr() != -1) {
        virq_t pending;
        if (!hv_sgi_queue_pop(&pending))
            break;
        hv_vgic3_inject_irq(pending.vintid, pending.priority, pending.active,
                            pending.pending, pending.hw_status, pending.hw_irq);
    }
    while (hv_vgic3_get_free_lr() != -1) {
        virq_t pending;
        if (!virq_queue_pop(&PERCPU(irq_queue), &pending))
            break;
        printf("HV: windows-native-aic: carrier inject CPU %d IRQ %u with x18=0x%lx\n",
               smp_id(), pending.vintid, ctx->regs[18]);
        hv_vgic3_inject_irq(pending.vintid, pending.priority, pending.active,
                            pending.pending, pending.hw_status, pending.hw_irq);
    }
#else
    (void)ctx;
#endif
}

static void hv_update_fiq(struct exc_info *ctx)
{ 
    u64 hcr = mrs(HCR_EL2);
    bool fiq_pending = false;

    hv_carrier_repair_x18(ctx);

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    if (hv_native_aic_windows_active() && hv_native_aic_windows_ready()) {
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
                    hv_vgic3_get_free_lr() != -1) {
                    PERCPU(timer_p_fiq_count)++;
                    PERCPU(timer_p_reflection_pending) = true;
                    hv_vgic3_inject_irq(HV_GIC_TIMER_P_INTID,
                                        hv_vgic3_get_priority(HV_GIC_TIMER_P_INTID),
                                        false, true, false, 0);
                }
            } else {
                PERCPU(timer_p_reflection_pending) = false;
                reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
            }

            if (mrs(CNTV_CTL_EL02) == (CNTx_CTL_ISTATUS | CNTx_CTL_ENABLE)) {
                fiq_pending = true;
                reg_clr(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_V);
                if (!PERCPU(timer_v_reflection_pending) &&
                    hv_vgic3_get_free_lr() != -1) {
                    PERCPU(timer_v_fiq_count)++;
                    PERCPU(timer_v_reflection_pending) = true;
                    hv_vgic3_inject_irq(HV_GIC_TIMER_V_INTID,
                                        hv_vgic3_get_priority(HV_GIC_TIMER_V_INTID),
                                        false, true, false, 0);
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

    fiq_pending |= PERCPU(ipi_pending) || PERCPU(pmc_pending);

    sysop("isb");
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    bool event_unread = PERCPU(ipi_pending) ||
                        PERCPU(timer_p_event_unread) ||
                        PERCPU(timer_v_event_unread);
    if (hv_native_aic_windows_ready() && event_unread) {
        if (!(hcr & HCR_VI))
            hv_write_hcr(hcr | HCR_VI);
    } else if (hcr & HCR_VI) {
        hv_write_hcr(hcr & ~HCR_VI);
    }
#elif !defined(ENABLE_VGIC_MODULE)
    if ((hcr & HCR_VF) && !fiq_pending) {
        hv_write_hcr(hcr & ~HCR_VF);
    } else if (!(hcr & HCR_VF) && fiq_pending) {
        hv_write_hcr(hcr | HCR_VF);
    }
#endif

    hv_carrier_drain_pending(ctx);
}

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
bool hv_native_aic_event_read(u64 *event)
{
    if (!hv_native_aic_windows_ready() || event == NULL)
        return false;

    /*
     * Fast IPIs are FIQ-class and are consumed by m1n1 for its own EL2
     * coordination.  Guest-originated sends are tagged by ipi_queued; reflect
     * only those arrivals as the architectural AIC IPI EVENT expected by the
     * native Windows controller callback.  This is an AIC EVENT/IRQ delivery,
     * not a vGIC list-register injection.
     */
    if (PERCPU(ipi_pending)) {
        *event = FIELD_PREP(AIC_EVENT_TYPE, AIC_EVENT_TYPE_IPI) |
                 AIC_EVENT_IPI_OTHER;
        PERCPU(ipi_pending) = false;
    /* Windows uses the virtual timer in the proven QEMU AIC path. */
    } else if (PERCPU(timer_v_event_unread)) {
        *event = 3;
        PERCPU(timer_v_event_unread) = false;
    } else if (PERCPU(timer_p_event_unread)) {
        *event = 2;
        PERCPU(timer_p_event_unread) = false;
    } else {
        return false;
    }

    if (!PERCPU(ipi_pending) && !PERCPU(timer_p_event_unread) &&
        !PERCPU(timer_v_event_unread))
        hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
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
    if (!PERCPU(timer_p_event_unread) && !PERCPU(timer_v_event_unread))
        hv_write_hcr(mrs(HCR_EL2) & ~HCR_VI);
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
                    regs[rt] = hv_vgic3_do_iar1();
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
                hv_vgic3_do_eoir1(regs[rt]);
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
                    pmcr1_value &= ~(BIT(16));
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
                regs[rt] = 0;
                printf("HV PMUv3 Redirect: mrs x%ld, PMUSERENR_EL0 = 0x%lx\n", rt, regs[rt]);
            }
            else {
                printf("HV PMUv3 Redirect (skipped write): msr PMUSERENR_EL0, x%ld = 0x%lx\n", rt, regs[rt]);
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
        SYSREG_PASS(ID_AA64PFR1_EL1)
        SYSREG_PASS(ID_AA64DFR0_EL1)
        SYSREG_PASS(ID_AA64DFR1_EL1)
        SYSREG_PASS(ID_AA64ISAR0_EL1)
        SYSREG_PASS(ID_AA64ISAR1_EL1)
        SYSREG_PASS(SYS_ID_AA64MMFR0_EL1)
        SYSREG_PASS(SYS_ID_AA64MMFR1_EL1)
        SYSREG_PASS(ID_AA64AFR0_EL1)
        SYSREG_PASS(ID_AA64AFR1_EL1)
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
            if (is_read)
                regs[rt] = PERCPU(ipi_pending) ? IPI_SR_PENDING : 0;
            else if (regs[rt] & IPI_SR_PENDING)
                PERCPU(ipi_pending) = false;
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
                if (regs[rt] & (CYC_OVRD_DISABLE_WFI_RET | CYC_OVRD_FIQ_MODE_MASK))
                    return false;
                msr(SYS_IMP_APL_CYC_OVRD, regs[rt]);
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
                    pcpu[i].ipi_queued = true;
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
                    pcpu[i].ipi_queued = true;
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
    hv_carrier_repair_x18(ctx);
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
    hv_carrier_repair_x18(ctx);
#ifdef ENABLE_VGIC_MODULE
#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    /*
     * AIC2 CONFIG is enabled on the last processor to finish the deferred HAL
     * handoff.  Other processors may still have their per-CPU HCR.IMO set.
     * Do not acknowledge their pending AIC EVENT at EL2: clear IMO and return,
     * allowing the still-pending physical IRQ to be taken again by Windows EL1.
     */
    if (hv_native_aic_active()) {
        hv_native_aic_enter_cpu();
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

    if (hv_native_aic_windows_active() && !hv_native_aic_windows_ready() &&
        ctx->regs[18] == 0) {
        virq_t pending = {
            .vintid = irq,
            .priority = hv_vgic3_get_priority(irq),
            .active = false,
            .pending = true,
            .hw_status = false,
            .hw_irq = 0,
        };
        virq_queue_push(&PERCPU(irq_queue), &pending);
        printf("HV: windows-native-aic: carrier deferred CPU %d IRQ %u with x18=0\n",
               smp_id(), irq);
    }
    else if(hv_vgic3_get_free_lr() != -1){
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
    hv_carrier_repair_x18(ctx);
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

    //
    // windows-native-aic Fast-IPI design decision (item 4, docs/windows-native-aic.md
    // "PMU / Fast-IPI FIQ handling"). Apple's "Fast IPI" (SYS_IMP_APL_IPI_RR_LOCAL_EL1
    // / IPI_RR_GLOBAL_EL1 / IPI_SR_EL1) is FIQ-class and CPU-local -- it bypasses AIC
    // entirely, so unlike ordinary peripheral interrupts it is NOT fixed by clearing
    // HCR_EL2.IMO; it stays trapped here because HCR_EL2.FMO stays set (hv.c). m1n1
    // ALSO uses this exact mechanism for its own EL2-internal cross-core coordination
    // (smp_send_ipi(), smp.c:417-424, used by hv_rendezvous() and by the
    // ICC_SGI1R_EL1-relay below), so this FIQ source cannot simply be masked off
    // permanently -- m1n1 needs to keep consuming it regardless of what the guest does.
    //
    // A guest write to IPI_RR_LOCAL_EL1/IPI_RR_GLOBAL_EL1 is already trapped and
    // relayed to a real cross-core Fast-IPI unconditionally (hv_handle_msr(), case
    // SYSREG_ISS(SYS_IMP_APL_IPI_RR_LOCAL_EL1)/(..._GLOBAL_EL1)), independent of vGIC
    // or native-AIC-passthrough. What this patch does NOT add is a new *arrival-side*
    // reflection for that path to the guest (i.e. no vGIC injection, no AIC software
    // IRQ, for PERCPU(ipi_pending) specifically) -- per this task's explicit
    // instruction not to invent an untested Fast-IPI reflection. This mirrors the
    // pre-existing behavior: even under the old vGIC-distributor design,
    // PERCPU(ipi_pending) (set below) was only ever exposed via the trapped read of
    // SYS_IMP_APL_IPI_SR_EL1, never injected as a virtual interrupt.
    //
    // DESIGN DECISION: cross-core IPI generation should instead use AIC's OWN
    // software-triggered-interrupt mechanism (AIC_IPI_SEND/AIC_IPI_ACK,
    // aic_regs.h:7-10, delivered via the same AIC_EVENT ack path as ordinary HW
    // interrupts per AIC_EVENT_TYPE_IPI=4, aic_regs.h:51) rather than Apple's CPU Fast
    // IPI registers. An AIC-mediated IPI is IRQ-class through the normal AIC
    // ack/arbitration path, so with HCR_EL2.IMO clear it reaches the guest directly,
    // zero EL2 involvement -- structurally identical to the timer-reflector's use of
    // aic_set_sw() above, just for a real cross-core doorbell instead of a
    // software-synthesized one. Windows' AppleAic-equivalent HAL extension is expected
    // to use this path for HalRequestIpi, not Apple's Fast-IPI system registers. This
    // is a documented design decision, not implemented code (nothing in this patch
    // calls AIC_IPI_SEND) -- confirming AIC_IPI_SEND really is delivered as IRQ, not
    // FIQ, on real hardware is an M1-validation item (docs/windows-native-aic.md OQ-2).
    //
    if (mrs(SYS_IMP_APL_IPI_SR_EL1) & IPI_SR_PENDING) {
#ifdef ENABLE_VGIC_MODULE
        if (!hv_native_aic_windows_active() || hv_native_aic_windows_ready() ||
            ctx->regs[18] != 0) {
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
        } else {
            printf("HV: windows-native-aic: carrier held CPU %d SGI with no KPCR\n",
                   smp_id());
        }
#endif
        if (PERCPU(ipi_queued)) {
            PERCPU(ipi_pending) = true;
            PERCPU(ipi_queued) = false;
        }
        msr(SYS_IMP_APL_IPI_SR_EL1, IPI_SR_PENDING);
        sysop("isb");
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
