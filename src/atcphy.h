/* SPDX-License-Identifier: MIT */

#ifndef ATCPHY_H
#define ATCPHY_H

#include "atcphy_core.h"
#include "types.h"

/*
 * m1n1 MMIO glue for the Apple Type-C PHY (ATCPHY) on T6020 (J414s). See
 * src/atcphy_core.h for the register/mode model and docs/j414s-atcphy.md for
 * the full design note, provenance, and safety discussion.
 *
 * This layer resolves the 5 ATCPHY MMIO windows from the live ADT and
 * applies the atcphy_core.h op-lists against real hardware via
 * read32/write32/udelay. It contains NO policy of its own beyond the
 * pipe-mux safety gate documented on atcphy_apply_mode() below.
 */

typedef struct {
    u64 usb2phy;     /* /arm-io/atc-phyN reg[0], already used by usb.c */
    u64 core;        /* /arm-io/atc-phyN reg[3], NEW */
    u64 pipehandler; /* /arm-io/usb-drdN reg[3], already used by usb.c */
    u64 axi2af;      /* /arm-io/atc-phyN reg[24], NEW */
    u64 lpdptx;      /* /arm-io/atc-phyN reg[20], NEW, DP-mode only */
} atcphy_regs_t;

/*
 * Resolves the MMIO windows for ATC port idx (0..2 on J414s; port 3 is the
 * HDMI-only DP path and is out of scope for this file). Returns 0 on
 * success, -1 if any expected ADT node/reg entry is missing.
 */
int atcphy_get_regs(u32 idx, atcphy_regs_t *regs);

/*
 * Safe, orientation-only entry point. Programs the ATCPHY core for
 * (ATCPHY_MODE_USB2, flipped) -- Apple's own "SuperSpeed lanes off, only
 * the crossbar's orientation-dependent protocol byte differs" state
 * (atcphy_modes[APPLE_ATCPHY_MODE_USB2], atc.c:660-679). This function:
 *
 *   - runs the usb2 PHY + ATCPHY core "small"/"big" power-domain wake
 *     sequence (atcphy_power_on, atc.c:1694-1723) that m1n1 has never run
 *     before this change (see docs/j414s-atcphy.md sec 3);
 *   - applies the global (non-lane) tunables read directly from the ADT;
 *   - runs the CFG0/SLEEP_CTRL override dance and CIO3PLL clock enable;
 *   - writes ACIOPHY_CROSSBAR + ACIOPHY_LANE_MODE + ATCPHY_MISC for the
 *     requested orientation;
 *   - releases the USB3 PHY reset line;
 *   - (re)asserts the pipehandler DUMMY (USB2-only) backend -- the ONLY
 *     backend this function ever selects.
 *
 * It NEVER switches the pipehandler PIPE mux away from DUMMY, so it is
 * safe to call at any time, including while a guest OS's xHCI/dwc3 driver
 * is bound to this port: from DWC3's perspective nothing about the
 * SuperSpeed PIPE interface changes, only USB2-irrelevant core-window
 * state that dwc3 does not itself touch.
 *
 * Returns 0 on success, -1 on failure (a diagnostic is printed).
 */
int atcphy_set_orientation(u32 idx, bool flipped);

/*
 * General entry point covering the full mode table (see atcphy_core.h sec
 * 3). USB3/USB3_DP require allow_pipe_switch=true because their pipe_state
 * is ATCPHY_PIPE_STATE_USB3: applying them reprograms the pipehandler PIPE
 * mux DWC3 depends on for SuperSpeed. Calling this with a non-DUMMY mode
 * while a guest's xHCI has already enumerated the port on the dummy
 * backend is UNTESTED and matches exactly the kind of live PIPE-topology
 * change flagged as dangerous in docs/j414s-atcphy.md sec 8/10 (the
 * project brief's own "0x144 BUGCODE_USB3_DRIVER" concern). No caller in
 * this tree passes allow_pipe_switch=true; it exists so the sequence is
 * compiled and host-test-covered without being reachable from any
 * automatic path.
 *
 * If dp_rate_valid, additionally programs the AUSPLL for dp_rate after the
 * lane config (meaningful only for ATCPHY_MODE_DP / ATCPHY_MODE_USB3_DP).
 * This does NOT include the per-lane DP analog calibration
 * (atcphy_dp_configure_lane in atc.c) -- see docs/j414s-atcphy.md sec 9. A
 * DP link programmed by this alone is not expected to train.
 *
 * Returns 0 on success, -1 on failure (a diagnostic is printed).
 */
int atcphy_apply_mode(u32 idx, atcphy_mode_t mode, bool flipped, bool allow_pipe_switch,
                      bool dp_rate_valid, atcphy_dp_rate_t dp_rate);

/*
 * Upstream-faithful full power-down: usb2 PHY off (atcphy_usb2_power_off,
 * atc.c:1616-1635), DP AUX disable (atcphy_disable_dp_aux, atc.c:1267-1281,
 * run unconditionally exactly as atcphy_power_off does at atc.c:1642), then
 * the core "big"/"small" domain sleep (atcphy_power_off, atc.c:1637-1667).
 *
 * This is the recover-to-baseline command for proxy-driven experiments: it
 * returns the PHY to (approximately) the state Linux's probe_finalize resets
 * to (atc.c:2242-2247), EXCEPT that unlike probe_finalize it deliberately
 * does NOT:
 *   - assert the dwc3 reset via pipehandler AON_GEN (a DWC3 touch, banned
 *     under a live guest -- see docs/j414s-atcphy.md sec 6), and
 *   - re-run the lock-less pipehandler->dummy mux write
 *     (atcphy_setup_pipehandler, atc.c:1141-1161): with the core domains
 *     asleep the pipehandler's response is unverified, and m1n1's own boot
 *     path already parks the mux on DUMMY.
 *
 * NOTE: atcphy_apply_mode(idx, ATCPHY_MODE_OFF, ...) is NOT this: it keeps
 * the PHY powered and parks the lanes/crossbar in the OFF mode-table entry
 * (upstream's configure() short-circuits MODE_OFF to atcphy_power_off
 * instead, atc.c:1733-1737). Use this function for upstream-equivalent OFF.
 *
 * Returns 0 on success, -1 on failure (a diagnostic is printed).
 */
int atcphy_power_off(u32 idx);

/*
 * Resolves and returns the base address of one MMIO window (atcphy_block_t)
 * for ATC port idx, or 0 on failure. Exists so the proxyclient can
 * cross-check its own Python-side ADT resolution against the C driver's
 * (this project has been bitten by divergent address decodes before).
 */
u64 atcphy_reg_base(u32 idx, u32 block);

/*
 * Guest-handoff re-apply latch.
 *
 * Booting a guest through the hypervisor (hv_init, hv.c) calls
 * usb_iodev_shutdown_except -> usb_phy_handoff_host, which re-parks the
 * pipehandler PIPE mux on the DUMMY backend as the boot-chain default
 * (usb.c) -- silently undoing any proxy-applied ATCPHY state. Arming a
 * (mode, flipped) pair here makes usb_phy_handoff_host re-apply it via
 * atcphy_apply_mode() immediately after its dummy parking.
 *
 * That call site is the ONE place in this tree that passes
 * allow_pipe_switch=true, and it is safe by this project's own rule
 * precisely because of when it runs: hv_init happens before the guest is
 * entered, so dwc3 is freshly reset and NO guest xHCI driver is bound to
 * the port yet. (docs/j414s-atcphy.md sec 6/12; the hardware session of
 * 2026-07-30 validated the full USB3 apply path on port 2.)
 *
 * Fail-safe: nothing is armed by default, so the boot chain's behaviour
 * is unchanged unless an operator explicitly arms a port over the proxy
 * (P_ATCPHY_ARM_GUEST_MODE). Arming does not touch hardware by itself.
 */
void atcphy_arm_guest_mode(u32 idx, atcphy_mode_t mode, bool flipped, bool armed);

/*
 * Called by usb_phy_handoff_host after it parks the PIPE mux on DUMMY.
 * Re-applies the armed mode for idx, or does nothing if not armed.
 * Returns 0 (including the not-armed case), -1 on a failed re-apply.
 */
int atcphy_reapply_guest_mode(u32 idx);

#endif
