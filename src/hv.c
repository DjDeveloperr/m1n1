/* SPDX-License-Identifier: MIT */

#include "hv.h"
#include "hv_tpm.h"
#include "assert.h"
#include "cpu_regs.h"
#include "display.h"
#include "gxf.h"
#include "memory.h"
#include "mtp_handoff.h"
#include "pcie.h"
#include "platform_identity.h"
#include "smp.h"
#include "string.h"
#include "usb.h"
#include "utils.h"
#include "adt.h"
#include "xnuboot.h"

#define HV_TICK_RATE      5000
#define HV_SLOW_TICK_RATE 1

DECLARE_SPINLOCK(bhl);

void hv_enter_guest(u64 x0, u64 x1, u64 x2, u64 x3, void *entry);
void hv_exit_guest(void) __attribute__((noreturn));

extern char _hv_vectors_start[0];

u64 hv_tick_interval;
u64 hv_secondary_tick_interval;

int hv_pinned_cpu;
int hv_want_cpu;

static bool hv_has_ecv;
static bool hv_should_exit[MAX_CPUS];
bool hv_started_cpus[MAX_CPUS];
u64 hv_cpus_in_guest;
/*
 * Set for the duration of hv_rendezvous().  hv_cpus_in_guest is cleared ONLY by
 * hv_exc_entry(), and hv_exc_sync()'s fast path -- the one that handles an
 * Apple IMPDEF MSR trap and returns straight to the guest -- deliberately skips
 * it.  A CPU taking those traps back to back therefore stays marked "in guest"
 * no matter how many it services, and a rendezvous requested by any other core
 * can never complete.  Measured on the J414s: CPU 0 runs the pure-AIC software
 * timer reflection, its breadcrumbs read `Sa#sSa#s` (two full fast-path turns,
 * no slow-path entry) while every other CPU ends in `F`, and the HV panics with
 * "Failed to rendezvous, missing CPUs: 0x1".  The fast path consults this flag
 * so it can take the slow round trip exactly when one is outstanding.
 */
u64 hv_rendezvous_pending;
u64 hv_saved_sp[MAX_CPUS];

struct hv_secondary_info_t {
    uint64_t hcr;
    uint64_t hacr;
    uint64_t vtcr, vttbr;
    uint64_t mdcr;
    uint64_t mdscr;
    uint64_t amx_ctl;
    uint64_t apvmkeylo, apvmkeyhi, apsts;
    uint64_t actlr_el2;
    uint64_t actlr_el1;
    uint64_t cnthctl;
    uint64_t sprr_config;
    uint64_t gxf_config;
};

static struct hv_secondary_info_t hv_secondary_info;
static u64 hv_secondary_regs[MAX_CPUS][4];

/*
 * Apple WFI mode 0 is the macOS guest policy, but it does not preserve the
 * complete architectural register state expected by Windows. In particular,
 * J414s captures prove Windows' reserved x18/KPCR register is valid immediately
 * before HalProcessorIdle executes WFI and first becomes zero at the following
 * instruction. m1n1's clock-gate-only mode 2 explicitly preserves CPU
 * registers, so retain it for the native-AIC Windows profile on every CPU.
 */
static void hv_configure_guest_wfi_mode(void)
{
    if (!cpu_features->cyc_ovrd)
        return;

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    const u64 mode = 2;
#else
    const u64 mode = 0;
#endif
    reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK, CYC_OVRD_WFI_MODE(mode));
    sysop("isb");

    u64 value = mrs(SYS_IMP_APL_CYC_OVRD);
    printf("HV: guest WFI mode %lu on CPU %d (CYC_OVRD=0x%lx)\n",
           FIELD_GET(CYC_OVRD_WFI_MODE_MASK, value), smp_id(), value);
}

void hv_init(void)
{
    pcie_shutdown();
    // Relinquish every USB controller except the one carrying this proxy.
    // The guest can then reset those DWC3 blocks into host mode and own their
    // DARTs without stale m1n1 device-mode endpoints or DMA mappings.
    usb_iodev_shutdown_except(uartproxy_iodev);
#if defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && defined(ENABLE_J414S_WINDOWS_USB_HOST_HANDOFF)
    /*
     * The internal PHY host signal does not control connector VBUS.  Put each
     * unused J414s CD3217/TPS6598x policy controller into its source-preferred
     * dual-role configuration while its IRQs are still masked.  The exact
     * proxy-selected controller remains untouched.
     */
    if (platform_is_j414s())
        usb_hpm_handoff_host(uartproxy_iodev);
#endif
    // Make sure we wake up DCP if we put it to sleep, just quiesce it to match ADT
    if (display_is_external && display_start_dcp() >= 0)
        display_shutdown(DCP_QUIESCED);
    // reenable hpm interrupts for the guest for unused iodevs
    usb_hpm_restore_irqs(0);
    smp_start_secondaries();
    smp_set_wfe_mode(true);
    hv_wdt_init();

    hv_pt_init();

#if defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && defined(ENABLE_J414S_WINDOWS_MTP_HANDOFF)
    /*
     * J414s-only Windows preboot handoff.  It boots the MTP IOP and prepares
     * DAPF/DART, but leaves DockChannel INIT/ring ownership to AppleMtpHid.
     * The helper has a second runtime chip/board guard; other m1n1 guests are
     * therefore unaffected even when this Windows build option is compiled.
     */
    mtp_handoff_init();
#endif

    // Configure hypervisor defaults

    //
    // UNKNOWN: do we need to bring TGE back? might have misunderstood why it was there at the start.
    // leaving it off for now.
    //
#ifndef ENABLE_VGIC_MODULE
    hv_write_hcr(HCR_API | // Allow PAuth instructions
                 HCR_APK | // Allow PAuth key registers
                 HCR_TEA | // Trap external aborts
                 HCR_RW |  // AArch64 guest
                 HCR_TSC | // Trap SMC exceptions (only writable on Blizzard/Avalanche cores as the previous generations used a chicken bit for this.)
                 HCR_AMO | // Trap SError exceptions
                 HCR_IMO | // Trap IRQ exceptions (for now)
                 HCR_FMO | // Trap FIQ exceptions (effectively required for now)
                 HCR_VM);  // Enable stage 2 translation
#elif defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    //
    // windows-native-aic transform -- primary HCR_EL2 site (secondary cores inherit
    // this exact value verbatim, see hv_secondary_info.hcr below and
    // hv_init_secondary()). See docs/windows-native-aic.md for the full design.
    //
    // Mu and Windows both drive the physical AIC directly, so ordinary IRQs never
    // route through EL2 and no virtual GIC is exposed.  FMO starts set while Mu's
    // VBAR is still zero.  FMO remains set throughout: once TimerDxe is ready, Mu
    // receives timer ticks as native-AIC software IRQs; after ExitBootServices,
    // Windows receives HCR.VI plus AIC EVENT(2/3).  No vGIC state is exposed.
    //
    // HCR_EL2.FMO stays set: the Apple timer is FIQ-only (there is no IRQ-mode timer
    // delivery on this hardware) and physical FIQs must keep trapping to EL2, because
    // Windows bugchecks (0x2B/0x3D) if a raw FIQ is ever delivered to EL1. m1n1
    // intercepts the timer FIQ, masks the physical source, and reflects it to the
    // guest as an ordinary per-CPU AIC software IRQ with an explicit re-arm handshake
    // -- see hv_update_fiq() and hv_timer_reflect_init() in hv_exc.c.
    //
    // HCR_EL2.TID3 is kept: it is unrelated to IRQ/FIQ routing. It still lets m1n1 OR
    // in the "GICv3 CPU interface present" bit on a trapped ID_AA64PFR0_EL1 read
    // (hv_exc.c, SYSREG_ISS(ID_AA64PFR0_EL1) case) for whatever UEFI/HAL GIC probing
    // logic may still run before/alongside the native AIC HAL extension; the vGIC
    // virtual CPU interface itself (ICH_*) is left enabled per-core (see
    // hv_vgicv3_enable_virtual_interrupts() calls below and in hv_init_secondary())
    // but is no longer used for timer delivery, only for the pre-existing, vestigial
    // ICC_SGI1R_EL1 SGI-emulation path -- see docs/windows-native-aic.md.
    //
    hv_write_hcr(HCR_API | // Allow PAuth instructions
                 HCR_APK | // Allow PAuth key registers
                 HCR_TEA | // Trap external aborts
                 HCR_RW |  // AArch64 guest
                 HCR_TSC | // Trap SMC exceptions (only writable on Blizzard/Avalanche cores as the previous generations used a chicken bit for this.)
                 HCR_TID3 | // Trap ID group 3 registers (AA64 PFR, MMFR, ISAR, AFR ID registers) - required to support the vanilla ArmGicDxe UEFI driver.
                 HCR_AMO | // Trap SError exceptions
                 HCR_FMO | // Hold timer FIQ until Mu's AIC and timer handlers are ready.
                 HCR_VM);  // Enable stage 2 translation
#else
    hv_write_hcr(HCR_API | // Allow PAuth instructions
                 HCR_APK | // Allow PAuth key registers
                 HCR_TEA | // Trap external aborts
                 HCR_RW |  // AArch64 guest
                 HCR_TSC | // Trap SMC exceptions (only writable on Blizzard/Avalanche cores as the previous generations used a chicken bit for this.)
                 HCR_TID3 | // Trap ID group 3 registers (AA64 PFR, MMFR, ISAR, AFR ID registers) - required to support the vanilla ArmGicDxe UEFI driver.
                 HCR_AMO | // Trap SError exceptions
                 HCR_IMO | // Trap IRQ exceptions (for now)
                 HCR_FMO | // Trap FIQ exceptions (effectively required for now)
                 HCR_VM);  // Enable stage 2 translation
#endif

    // No guest vectors initially
    msr(VBAR_EL12, 0);

    //set up a HACR bit (56)
    printf("DEBUG: setting up HACR\n");
    uint64_t hacr_val = mrs(HACR_EL2);
    hacr_val |= BIT(56);
    msr(HACR_EL2, hacr_val);

    //
    // m1n1_windows change: initialize PSCI.
    //
    printf("DEBUG: setting up PSCI\n");
    hv_psci_init();
#ifdef ENABLE_VGIC_MODULE
#ifndef ENABLE_NATIVE_AIC_PASSTHROUGH
    //
    // m1n1_windows change: set up the vGIC
    //

    //
    hv_vgicv3_init();
    init_vgic_irq_queues();
#endif
#endif

    // Compute tick interval
    hv_tick_interval = mrs(CNTFRQ_EL0) / HV_TICK_RATE;

    hv_has_ecv = mrs(ID_AA64MMFR0_EL1) & (0xfULL << 60);

    if (hv_has_ecv) {
        printf("HV: ECV enabled\n");
        reg_set(CNTHCTL_EL2,
                CNTHCTL_EL1NVVCT | CNTHCTL_EL1NVPCT | CNTHCTL_EL1TVT | CNTHCTL_EL1PCTEN);
        hv_secondary_tick_interval = mrs(CNTFRQ_EL0) / HV_SLOW_TICK_RATE;
    } else {
        printf("HV: No ECV supported\n");
        // Enable physical timer for EL1
        msr(CNTHCTL_EL2, CNTHCTL_EL1PTEN | CNTHCTL_EL1PCTEN);

        hv_secondary_tick_interval = hv_tick_interval;
    }

    hv_configure_guest_wfi_mode();

    sysop("dsb ishst");
    sysop("tlbi alle1is");
    sysop("dsb ish");
    sysop("isb");
}

static void hv_set_gxf_vbar(void)
{
    msr(SYS_IMP_APL_VBAR_GL1, _hv_vectors_start);
}

void hv_start(void *entry, u64 regs[4])
{
    if (boot_cpu_idx == -1) {
        printf("Boot CPU has not been found, can't start hypervisor\n");
        return;
    }

    memset(hv_should_exit, 0, sizeof(hv_should_exit));
    memset(hv_started_cpus, 0, sizeof(hv_started_cpus));

    hv_started_cpus[boot_cpu_idx] = true;

    msr(VBAR_EL1, _hv_vectors_start);

    if (gxf_enabled())
        gl2_call(hv_set_gxf_vbar, 0, 0, 0, 0);

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    /*
     * Windows needs the GICD/GICR MMIO records advertised by Mu only while it
     * classifies the startup controller.  Initialize those register hooks after
     * the host's broad mappings are complete, but leave HCR.IMO clear and never
     * enable ICH/LRs: this is a topology carrier, not an interrupt-delivery path.
     */
#ifdef ENABLE_VGIC_MODULE
    hv_vgicv3_init();
    init_vgic_irq_queues();
#endif

    /* Host MMIO mappings are complete before hv_start(), so these hooks persist. */
    hv_native_aic_transition_init();
#endif

    //
    // windows-native-aic: this is the "secondary CPU path" half of the HCR_EL2 update
    // in hv_init() above.  APs started by Windows apply the current pure-AIC phase
    // policy in hv_init_secondary() rather than inheriting a stale Mu-era FIQ state.
    //
    hv_secondary_info.hcr = mrs(HCR_EL2);
    hv_secondary_info.hacr = mrs(HACR_EL2);
    hv_secondary_info.vtcr = mrs(VTCR_EL2);
    hv_secondary_info.vttbr = mrs(VTTBR_EL2);
    hv_secondary_info.mdcr = mrs(MDCR_EL2);
    hv_secondary_info.mdscr = mrs(MDSCR_EL1);
    hv_secondary_info.amx_ctl = mrs(SYS_IMP_APL_AMX_CTL_EL2);
    hv_secondary_info.apvmkeylo = mrs(SYS_IMP_APL_APVMKEYLO_EL2);
    hv_secondary_info.apvmkeyhi = mrs(SYS_IMP_APL_APVMKEYHI_EL2);
    hv_secondary_info.apsts = mrs(SYS_IMP_APL_APSTS_EL12);
    hv_secondary_info.actlr_el2 = mrs(ACTLR_EL2);
    if (cpu_features->actlr_el2)
        hv_secondary_info.actlr_el1 = mrs(SYS_ACTLR_EL12);
    else
        hv_secondary_info.actlr_el1 = mrs(SYS_IMP_APL_ACTLR_EL12);
    hv_secondary_info.cnthctl = mrs(CNTHCTL_EL2);
    hv_secondary_info.sprr_config = mrs(SYS_IMP_APL_SPRR_CONFIG_EL1);
    hv_secondary_info.gxf_config = mrs(SYS_IMP_APL_GXF_CONFIG_EL1);

#if defined(ENABLE_VGIC_MODULE) && !defined(ENABLE_NATIVE_AIC_PASSTHROUGH)
    hv_vgicv3_enable_virtual_interrupts();
    hv_vgicv3_init_list_registers();
#endif

    hv_arm_tick(false);
    hv_pinned_cpu = -1;
    hv_want_cpu = -1;
    hv_cpus_in_guest = BIT(smp_id());

    u64 adt_base;
    if(chip_id == T8103 || chip_id == T8112)
        adt_base = ADT_EL2_36_BIT;
    else
        adt_base = ADT_EL2_42_BIT;

    //map the address of the (EL2) ADT to a fixed location so EL1 can patch it
    hv_map_hw(adt_base, (u64)adt, ALIGN_UP(cur_boot_args.devtree_size, SZ_16K));

    /*
     * TEE ACPI Profile 4.6.3 requires a TPM's Error, Cancel and Start bits to
     * be clear when firmware hands control to the OS. No-op unless a CRB was
     * mapped, so this is unconditional rather than gated on a flag nobody
     * would remember to set.
     */
    hv_tpm_prepare_for_guest();

    hv_enter_guest(regs[0], regs[1], regs[2], regs[3], entry);

    __atomic_and_fetch(&hv_cpus_in_guest, ~BIT(smp_id()), __ATOMIC_ACQUIRE);
    spin_lock(&bhl);

    hv_wdt_stop();

    printf("HV: Exiting hypervisor (main CPU)\n");

    spin_unlock(&bhl);
    // Wait a bit for the guest CPUs to exit on their own if they are in the process.
    udelay(200000);
    spin_lock(&bhl);

    hv_started_cpus[boot_cpu_idx] = false;

    for (int i = 0; i < MAX_CPUS; i++) {
        if (i == boot_cpu_idx) {
            continue;
        }
        hv_should_exit[i] = true;
        if (hv_started_cpus[i]) {
            printf("HV: Waiting for CPU %d to exit\n", i);
            spin_unlock(&bhl);
            smp_wait(i);
            spin_lock(&bhl);
            hv_started_cpus[i] = false;
        }
    }

    printf("HV: All CPUs exited\n");
    spin_unlock(&bhl);
}

static void hv_init_secondary(struct hv_secondary_info_t *info)
{
    gxf_init();

    msr(VBAR_EL1, _hv_vectors_start);

    //
    // windows-native-aic: secondary-CPU HCR_EL2 site. info->hcr is the value hv_init()
    // computed on the boot CPU (see the primary HCR_EL2 comment there) captured by
    // hv_start() above; this core gets the identical IMO-clear/FMO-set configuration,
    // not a re-derived one, so there is nothing native-AIC-specific to add here beyond
    // this note.
    //
    msr(HCR_EL2, info->hcr);
    msr(HACR_EL2, info->hacr);
    msr(VTCR_EL2, info->vtcr);
    msr(VTTBR_EL2, info->vttbr);
    msr(MDCR_EL2, info->mdcr);
    msr(MDSCR_EL1, info->mdscr);
    msr(SYS_IMP_APL_AMX_CTL_EL2, info->amx_ctl);
    msr(SYS_IMP_APL_APVMKEYLO_EL2, info->apvmkeylo);
    msr(SYS_IMP_APL_APVMKEYHI_EL2, info->apvmkeyhi);
    msr(SYS_IMP_APL_APSTS_EL12, info->apsts);
    msr(ACTLR_EL2, info->actlr_el2);
    if (cpu_features->actlr_el2)
        msr(SYS_ACTLR_EL12, info->actlr_el1);
    else
        msr(SYS_IMP_APL_ACTLR_EL12, info->actlr_el1);
    msr(CNTHCTL_EL2, info->cnthctl);
    msr(SYS_IMP_APL_SPRR_CONFIG_EL1, info->sprr_config);
    msr(SYS_IMP_APL_GXF_CONFIG_EL1, info->gxf_config);

#ifdef ENABLE_NATIVE_AIC_PASSTHROUGH
    /* APs started by Windows inherit the post-EBS FIQ bridge policy. */
    hv_native_aic_enter_cpu();
#elif defined(ENABLE_VGIC_MODULE)
    hv_vgicv3_enable_virtual_interrupts();
    hv_vgicv3_init_list_registers();
#endif

    hv_configure_guest_wfi_mode();

    if (gxf_enabled())
        gl2_call(hv_set_gxf_vbar, 0, 0, 0, 0);

    hv_arm_tick(true);
}

static void hv_enter_secondary(void *entry, u64 regs[4])
{
    hv_enter_guest(regs[0], regs[1], regs[2], regs[3], entry);

    spin_lock(&bhl);

    printf("HV: Exiting from CPU %d\n", smp_id());

    __atomic_and_fetch(&hv_cpus_in_guest, ~BIT(smp_id()), __ATOMIC_ACQUIRE);

    hv_started_cpus[smp_id()] = false;
    spin_unlock(&bhl);
}

void hv_start_secondary(int cpu, void *entry, u64 regs[4])
{
    printf("HV: Initializing secondary %d\n", cpu);
    iodev_console_flush();

    mmu_init_secondary(cpu);
    iodev_console_flush();
    smp_call4(cpu, hv_init_secondary, (u64)&hv_secondary_info, 0, 0, 0);
    smp_wait(cpu);
    iodev_console_flush();

    printf("HV: Entering guest secondary %d at %p\n", cpu, entry);
    hv_started_cpus[cpu] = true;
    __atomic_or_fetch(&hv_cpus_in_guest, BIT(cpu), __ATOMIC_ACQUIRE);

    /*
     * smp_call4() returns to the caller as soon as the target increments its
     * acknowledgement flag, before the target necessarily dereferences the
     * argument pointer.  PSCI's CPU_ON caller supplies a stack-local regs[];
     * retaining that pointer races the next CPU_ON and can give an AP another
     * processor's context ID.  Keep the guest entry registers in stable
     * per-CPU storage for the lifetime of the asynchronous guest call.
     */
    memcpy(hv_secondary_regs[cpu], regs, sizeof(hv_secondary_regs[cpu]));
    sysop("dmb sy");

    iodev_console_flush();
    smp_call4(cpu, hv_enter_secondary, (u64)entry,
              (u64)hv_secondary_regs[cpu], 0, 0);
}

void hv_exit_cpu(int cpu)
{
    if (cpu == -1)
        cpu = smp_id();

    printf("HV: Requesting exit of CPU#%d from the guest\n", cpu);
    hv_should_exit[cpu] = true;
}

void hv_rendezvous(void)
{
    int timeout = 1000000;

    if (!__atomic_load_n(&hv_cpus_in_guest, __ATOMIC_ACQUIRE))
        return;

    /*
     * Publish BEFORE the IPIs.  A CPU already inside the sync fast path will
     * not take the IPI until it next opens an FIQ window, and if it is
     * servicing a dense stream of IMPDEF MSR traps that window may never come.
     * The flag lets that CPU notice the rendezvous from inside the fast path
     * itself rather than depending on interrupt delivery.
     */
    __atomic_store_n(&hv_rendezvous_pending, 1, __ATOMIC_RELEASE);

    /* IPI all CPUs. This might result in spurious IPIs to the guest... */
    for (int i = 0; i < MAX_CPUS; i++) {
        if (i != smp_id() && hv_started_cpus[i]) {
            smp_send_ipi(i);
        }
    }

    while (timeout--) {
        if (!__atomic_load_n(&hv_cpus_in_guest, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&hv_rendezvous_pending, 0, __ATOMIC_RELEASE);
            return;
        }
    }

    __atomic_store_n(&hv_rendezvous_pending, 0, __ATOMIC_RELEASE);
    hv_panic("HV: Failed to rendezvous, missing CPUs: 0x%lx (current: %d)\n",
             __atomic_load_n(&hv_cpus_in_guest, __ATOMIC_ACQUIRE), smp_id());
}

bool hv_switch_cpu(int cpu)
{
    if (cpu > MAX_CPUS || cpu < 0 || !hv_started_cpus[cpu]) {
        printf("HV: CPU #%d is inactive or invalid\n", cpu);
        return false;
    }
    printf("HV: switching to CPU #%d\n", cpu);
    hv_want_cpu = cpu;
    hv_rendezvous();
    return true;
}

void hv_pin_cpu(int cpu)
{
    hv_pinned_cpu = cpu;
}

void hv_write_hcr(u64 val)
{
    if (gxf_enabled() && !in_gl12())
        gl2_call(hv_write_hcr, val, 0, 0, 0);
    else
        msr(HCR_EL2, val);
}

u64 hv_get_spsr(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_SPSR_GL1);
    else
        return mrs(SPSR_EL2);
}

void hv_set_spsr(u64 val)
{
    if (in_gl12())
        return msr(SYS_IMP_APL_SPSR_GL1, val);
    else
        return msr(SPSR_EL2, val);
}

u64 hv_get_esr(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_ESR_GL1);
    else
        return mrs(ESR_EL2);
}

u64 hv_get_far(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_FAR_GL1);
    else
        return mrs(FAR_EL2);
}

u64 hv_get_afsr1(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_AFSR1_GL1);
    else
        return mrs(AFSR1_EL2);
}

u64 hv_get_elr(void)
{
    if (in_gl12())
        return mrs(SYS_IMP_APL_ELR_GL1);
    else
        return mrs(ELR_EL2);
}

void hv_set_elr(u64 val)
{
    if (in_gl12())
        return msr(SYS_IMP_APL_ELR_GL1, val);
    else
        return msr(ELR_EL2, val);
}

void hv_arm_tick(bool secondary)
{
    if (secondary)
        msr(CNTP_TVAL_EL0, hv_secondary_tick_interval);
    else
        msr(CNTP_TVAL_EL0, hv_tick_interval);
    msr(CNTP_CTL_EL0, CNTx_CTL_ENABLE);
}

void hv_maybe_exit(void)
{
    if (hv_should_exit[smp_id()]) {
        hv_exit_guest();
    }
}

void hv_tick(struct exc_info *ctx)
{
    hv_wdt_pet();
    iodev_handle_events(uartproxy_iodev);
    if (iodev_can_read(uartproxy_iodev)) {
        printf("HV: User interrupt\n");
        iodev_console_flush();
        if (hv_pinned_cpu == -1 || hv_pinned_cpu == smp_id())
            hv_exc_proxy(ctx, START_HV, HV_USER_INTERRUPT, NULL);
    }
    hv_vuart_poll();
}
