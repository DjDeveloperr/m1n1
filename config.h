/* SPDX-License-Identifier: MIT */

#ifndef CONFIG_H
#define CONFIG_H

// Enable framebuffer console
#define USE_FB
// Disable framebuffer console unless verbose boot is enabled
//#define FB_SILENT_MODE
// Initialize USB early and break into proxy if device is opened within this time (sec)
//#define EARLY_PROXY_TIMEOUT 5

// Minimal build for bring-up
//#define BRINGUP
// Disable display configuration / bringup on desktop devices
//#define NO_DISPLAY

// Print RTKit logs to the console
//#define RTKIT_SYSLOG

// Target for device-specific debug builds
//#define TARGET T8103
// Some devices like Apple TV HD use other uarts for debug console
//#define TARGET_BOARD 0x34

// Enable SMMU abstraction layer to expose a fake SMMU to the guest which will redirect writes to the host IOMMU in a compatible manner
// #define ENABLE_SMMU

//
// Enable the vGIC module.
//
 #define ENABLE_VGIC_MODULE

//
// windows-native-aic transform (branch windows-native-aic; see
// docs/windows-native-aic.md for the full design writeup, the timer re-arm handshake,
// and the M1-VALIDATION CHECKLIST). UNTESTED -- there is no aarch64 m1n1 toolchain on
// the machine this was written on and no M1 to boot it on; every register/bit fact
// below is cited from m1n1's own headers, and every genuinely uncertain point is
// called out explicitly rather than guessed.
//
// When this is defined (it requires ENABLE_VGIC_MODULE, enforced below), a guest
// booted under the hypervisor drives the real Apple AIC directly instead of the
// emulated GICv3 distributor/redistributor:
//
//  - hv.c no longer sets HCR_EL2.IMO, so physical (AIC-routed) IRQs are delivered
//    straight to the guest at EL1 with zero EL2 involvement. HCR_EL2.FMO stays set
//    unconditionally, because the Apple timer is FIQ-only and Windows bugchecks
//    (0x2B/0x3D) if a raw FIQ ever reaches EL1.
//  - hv_vgic.c's hv_vgicv3_init() skips installing the GICD/GICR/ITS MMIO hv_map_hook
//    traps -- the guest gets the real AIC MMIO straight through (it already did; no
//    hook ever targeted it) and drives it itself.
//  - hv_exc.c's hv_exc_irq() no longer translates real AIC IRQ events into injected
//    vGIC interrupts; physical IRQ is not expected to trap to EL2 at all anymore
//    (a defensive fail-closed fallback remains in case that assumption is wrong).
//  - The physical timer FIQ is reflected to the guest as an ordinary per-CPU AIC
//    software-generated IRQ (aic_set_sw(), NOT a GICv3 list-register injection --
//    that was the original plan for this file but was superseded by a per-CPU
//    mask/pending/re-arm handshake; see hv_exc.c's hv_update_fiq() and
//    hv_timer_reflect_init(), and docs/windows-native-aic.md "Timer re-arm handshake").
//
#define ENABLE_NATIVE_AIC_PASSTHROUGH

#if defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && !defined(ENABLE_VGIC_MODULE)
#error "ENABLE_NATIVE_AIC_PASSTHROUGH requires ENABLE_VGIC_MODULE -- see config.h comment above"
#endif

//
// Use PSCI to turn on the CPUs in earnest rather than just setting up the spintables that m1n1 uses.
//
// #define PSCI_POWER_ON_CPUS_ENABLE

#ifdef RELEASE
# define FB_SILENT_MODE
# ifdef CHAINLOADING
#  define EARLY_PROXY_TIMEOUT 5
# endif
#endif

#endif
