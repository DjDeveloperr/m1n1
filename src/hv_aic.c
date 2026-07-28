/* SPDX-License-Identifier: MIT */

#include "adt.h"
#include "aic.h"
#include "aic_regs.h"
#include "hv.h"
#include "hv_vgic.h"
#include "smp.h"
#include "uartproxy.h"
#include "utils.h"

#define IRQTRACE_IRQ BIT(0)

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
#define AIC2_GLOBAL_CONFIG        0x14
#define AIC2_GLOBAL_CONFIG_ENABLE BIT(0)

static bool native_aic_active;
static bool mu_aic_ready;
static bool mu_timer_ready;
static bool windows_aic_phase;
static bool windows_aic_enabled;

bool hv_native_aic_active(void)
{
    return __atomic_load_n(&native_aic_active, __ATOMIC_ACQUIRE);
}

bool hv_native_aic_windows_active(void)
{
    return __atomic_load_n(&windows_aic_phase, __ATOMIC_ACQUIRE);
}

bool hv_native_aic_windows_ready(void)
{
    return __atomic_load_n(&windows_aic_enabled, __ATOMIC_ACQUIRE);
}

bool hv_native_aic_mu_timer_active(void)
{
    return __atomic_load_n(&mu_timer_ready, __ATOMIC_ACQUIRE) &&
           !hv_native_aic_windows_active();
}

void hv_native_aic_enter_cpu(void)
{
    bool startup_carrier = hv_native_aic_windows_active() &&
                           !__atomic_load_n(&windows_aic_enabled, __ATOMIC_ACQUIRE);

    for (u32 lr = 0; lr < hv_vgic3_num_lrs(); ++lr)
        hv_vgic3_write_lr(lr, 0);

    if (startup_carrier) {
        /*
         * Apple implements the EL2 list registers, but carrier LR behavior is
         * not reliable across J414s' two core types. Trap the short startup
         * CPU-interface window, keep IAR/EOIR state in software queues, and let
         * hv_update_fiq drive HCR.VI for deliverable entries. Target priorities
         * come from the guest's GICR state, and
         * hv_exc_exit observes Windows' kernel x18 alias while APs are parked.
         * The first Windows AIC2 CONFIG enable removes ICH,
         * TALL1, IMO, and the carrier VI path permanently.
         */
        msr(ICH_VMCR_EL2, BIT(1));
        msr(ICH_HCR_EL2, BIT(0) | BIT(12));
    } else {
        msr(ICH_HCR_EL2, 0);
    }
    sysop("isb");
    /*
     * Ordinary IRQs are native AIC in both phases.  Apple timer FIQ stays at
     * EL2: Mu receives it through its real-AIC software-IRQ ABI, while Windows
     * receives HCR.VI plus raw AIC EVENT 2/3.  No vGIC interface is involved.
     */
    u64 hcr = (mrs(HCR_EL2) & ~(HCR_IMO | HCR_VI)) | HCR_FMO;
    if (startup_carrier)
        hcr |= HCR_IMO;
    hv_write_hcr(hcr);
}

void hv_native_aic_timer_ready(void)
{
    if (!__atomic_load_n(&mu_aic_ready, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&mu_timer_ready, __ATOMIC_ACQUIRE) ||
        hv_native_aic_windows_active())
        return;

    /*
     * TimerDxe registers its callback before enabling the architectural timer.
     * From this point EL2 may reflect timer FIQ as a real AIC software IRQ; Mu's
     * native AIC handler maps the reserved per-CPU source back to logical 17/18.
     */
    __atomic_store_n(&mu_timer_ready, true, __ATOMIC_RELEASE);
    hv_timer_native_enable();
    hv_native_aic_enter_cpu();
    printf("HV: windows-native-aic: Mu timer handler ready; pure-AIC SW timer reflection active on CPU %d\n",
           smp_id());
}

static bool handle_native_aic_transition(struct exc_info *ctx, u64 addr, u64 *val,
                                         bool write, int width)
{
    bool config_write = write && width == 2 &&
                        addr == aic->base + AIC2_GLOBAL_CONFIG;

    /* The hook replaces the normal identity mapping for this page. */
    if (!hv_pa_rw(ctx, addr, val, write, width))
        return false;

    /*
     * A post-handoff timer/IPI is signaled by a real AIC software-doorbell
     * IRQ. Consume that physical EVENT first, then expose the processor-local
     * source token expected by the Windows AIC HAL extension.
     */
    if (!write && width == 2 && addr == aic->base + aic->regs.event) {
        u64 raw_event = *val;

        if (hv_native_aic_event_read(raw_event, val))
            return true;
    }

    if (config_write && (*val & AIC2_GLOBAL_CONFIG_ENABLE) &&
        !__atomic_load_n(&mu_aic_ready, __ATOMIC_ACQUIRE) &&
        !hv_native_aic_windows_active()) {
        /*
         * AppleAicDxe enables CONFIG before it registers its exception handler.
         * Record controller readiness here, but keep timer FIQ held until the
         * first timer programming write proves TimerDxe registered its callback.
         */
        __atomic_store_n(&mu_aic_ready, true, __ATOMIC_RELEASE);
        printf("HV: windows-native-aic: Mu AIC2 configured; waiting for timer handler on CPU %d\n",
               smp_id());
    } else if (config_write && !(*val & AIC2_GLOBAL_CONFIG_ENABLE) &&
        !hv_native_aic_windows_active()) {
        /*
         * AppleAicDxe masks every source and clears CONFIG in its
         * ExitBootServices callback.  That is the exact firmware/Windows
         * boundary: keep IRQs native, but start trapping the Apple timer FIQ
         * so Windows never receives an architectural FIQ exception.
        */
        __atomic_store_n(&windows_aic_phase, true, __ATOMIC_RELEASE);
        hv_timer_reflect_hold();
        hv_native_aic_enter_cpu();
        printf("HV: windows-native-aic: Mu ExitBootServices observed; startup carrier active on CPU %d\n",
               smp_id());
    } else if (config_write && (*val & AIC2_GLOBAL_CONFIG_ENABLE) &&
               hv_native_aic_windows_active()) {
        hv_carrier_retire_active_sgis();
        __atomic_store_n(&windows_aic_enabled, true, __ATOMIC_RELEASE);
        hv_native_aic_enter_cpu();
        hv_timer_reflect_enable();
        printf("HV: windows-native-aic: Windows enabled AIC2 CONFIG on CPU %d\n",
               smp_id());
    }
    return true;
}

void hv_native_aic_transition_init(void)
{
    __atomic_store_n(&native_aic_active, true, __ATOMIC_RELEASE);
    __atomic_store_n(&mu_aic_ready, false, __ATOMIC_RELEASE);
    __atomic_store_n(&mu_timer_ready, false, __ATOMIC_RELEASE);
    __atomic_store_n(&windows_aic_phase, false, __ATOMIC_RELEASE);
    __atomic_store_n(&windows_aic_enabled, false, __ATOMIC_RELEASE);
    /*
     * Install after the host has completed its broad MMIO mappings (hv_start,
     * not hv_init), otherwise pt_update overwrites these hooks.  CONFIG lives
     * in the first page and EVENT in the split AIC2 event aperture.
     */
    hv_map_hook(aic->base, handle_native_aic_transition, 0x1000);
    hv_map_hook((aic->base + aic->regs.event) & ~0xfffULL,
                handle_native_aic_transition, 0x1000);
    hv_timer_reflect_init();
    hv_native_aic_enter_cpu();
    printf("HV: windows-native-aic: pure AIC active; watching CONFIG 0x%llx and EVENT 0x%llx\n",
           (unsigned long long)(aic->base + AIC2_GLOBAL_CONFIG),
           (unsigned long long)(aic->base + aic->regs.event));
}
#endif

static u32 trace_hw_num[AIC_MAX_DIES][AIC_MAX_HW_NUM / 32];

static void emit_irqtrace(u16 die, u16 type, u16 num)
{
    struct hv_evt_irqtrace evt = {
        .flags = IRQTRACE_IRQ,
        .type = type,
        .num = die * aic->max_irq + num,
    };

    hv_wdt_suspend();
    uartproxy_send_event(EVT_IRQTRACE, &evt, sizeof(evt));
    hv_wdt_resume();
}

static bool trace_aic_event(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    if (!hv_pa_rw(ctx, addr, val, write, width))
        return false;

    if (addr != (aic->base + aic->regs.event) || write || width != 2) {
        return true;
    }

    u16 die = FIELD_GET(AIC_EVENT_DIE, *val);
    u16 type = FIELD_GET(AIC_EVENT_TYPE, *val);
    u16 num = FIELD_GET(AIC_EVENT_NUM, *val);

    if (die > AIC_MAX_DIES)
        return true;

    switch (type) {
        case AIC_EVENT_TYPE_HW:
            if (trace_hw_num[die][num / 32] & BIT(num & 31)) {
                emit_irqtrace(die, type, num);
            }
            break;
        default:
            // ignore
            break;
    }

    return true;
}

bool hv_trace_irq(u32 type, u32 num, u32 count, u32 flags)
{
    dprintf("HV: hv_trace_irq type: %u start: %u num: %u flags: 0x%x\n", type, num, count, flags);
    if (type == AIC_EVENT_TYPE_HW) {
        u32 die = num / aic->max_irq;
        num %= AIC_MAX_HW_NUM;
        if (die >= aic->max_irq || num >= AIC_MAX_HW_NUM || count > AIC_MAX_HW_NUM - num) {
            printf("HV: invalid IRQ range: (%u, %u) for die %u\n", num, num + count, die);
            return false;
        }
        for (u32 n = num; n < num + count; n++) {
            switch (flags) {
                case IRQTRACE_IRQ:
                    trace_hw_num[die][n / 32] |= BIT(n & 31);
                    break;
                default:
                    trace_hw_num[die][n / 32] &= ~(BIT(n & 31));
                    break;
            }
        }
    } else {
        printf("HV: not handling AIC event type: 0x%02x num: %u\n", type, num);
        return false;
    }

    if (!aic) {
        printf("HV: AIC not initialized\n");
        return false;
    }

    static bool hooked = false;

    if (aic && !hooked) {
        hv_map_hook(aic->base, trace_aic_event, aic->regs.reg_size);
        hooked = true;
    }

    return true;
}
