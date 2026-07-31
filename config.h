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
// Unified J414s Windows profile (branch feature/j414s-windows-unified; see
// docs/windows-native-aic.md for the full design writeup, the timer re-arm handshake,
// and docs/windows-unified.md for the build and ownership contract).
//
// When this is defined (it requires ENABLE_VGIC_MODULE, enforced below), a guest
// booted under the hypervisor starts on the emulated GICv3 carrier, then drives
// the real Apple AIC after the Windows HAL extension enables AIC2 CONFIG:
//
//  - HCR_EL2.IMO and the GICD/GICR hooks remain active during firmware and early
//    Windows startup. This is required by the inbox GIC callbacks that execute
//    before the Apple HAL's deferred carrier replacement.
//  - hv_aic.c forwards the real AIC register page and watches AIC2 CONFIG. On its
//    enable write, each CPU clears its virtual-interface state and HCR_EL2.IMO;
//    physical AIC IRQs then reach the guest at EL1 with zero EL2 involvement.
//  - The physical timer FIQ is reflected to the guest as an ordinary per-CPU AIC
//    software-generated IRQ after handoff; before handoff it uses the original
//    GIC list-register carrier path. HCR_EL2.FMO remains set throughout.
//
#define ENABLE_NATIVE_AIC_PASSTHROUGH

//
// J414s Windows MTP/DockChannel preboot handoff.
//
// The MTP coprocessor must already be running when the Windows AppleMtpHid
// ACPI driver takes ownership of the DockChannel transport.  The handoff
// helper is deliberately compiled only with the native-AIC Windows profile
// and has a second, exact runtime J414s identity check, so normal
// m1n1/macOS/Linux boots
// never touch MTP, its DART, or its DockChannel FIFOs.
//
#define ENABLE_J414S_WINDOWS_MTP_HANDOFF

// Leave every non-proxy J414s Type-C policy controller in a Source/DFP
// configuration before Windows takes ownership of its xHCI controller.  This
// is required in addition to the internal USB2 PHY host-role handoff: the PHY
// signal alone does not make the external Type-C controller source VBUS.
#define ENABLE_J414S_WINDOWS_USB_HOST_HANDOFF

// Opt-in proxy operation that installs the persistent J414s BCM4388 SID-1
// deny-all domain after pcie_init() and before hv_start(). It additionally
// requires an explicit top-of-memory reservation paired with Mu's DRT0
// profile; compiling this capability never reserves or mutates wireless state.
#define ENABLE_J414S_WINDOWS_WIRELESS_HANDOFF

// Opt-in proxy operation for the J414s media profile (MCA/ADMAC audio, AOP PDM
// microphones, ISP camera).  Like the wireless handoff above, compiling this
// capability neither reads nor mutates any media device: media_handoff.c has no
// automatic call site anywhere in m1n1, so a boot that never issues the
// explicit P_MEDIA_HANDOFF_INIT proxy request behaves exactly as it does
// today.  The operation itself defaults to a read-only census; each hardware
// write it can perform needs its own flag bit on top.
#define ENABLE_J414S_WINDOWS_MEDIA_HANDOFF

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
