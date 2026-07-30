/* SPDX-License-Identifier: MIT */

#ifndef ATCPHY_CORE_H
#define ATCPHY_CORE_H

/*
 * Apple Type-C PHY (ATCPHY) register/mode/tunable contract for T6020
 * (J414s / MacBook Pro 14" M2 Pro), host-testable pure-logic seam.
 *
 * This file contains NO MMIO. Every sequence is expressed as a declarative
 * op-list (atcphy_seq_op_t) interpreted by atcphy_seq_apply() through
 * caller-supplied read/write/delay callbacks. The real callbacks (src/atcphy.c)
 * use m1n1's read32/write32/mask32/udelay against addresses resolved from the
 * live ADT; host tests use fake callbacks over an in-memory register file.
 *
 * PROVENANCE (facts transcribed with citations; nothing invented):
 *
 *  - m1n1 (MIT), /Users/dj/Developer/m1n1, branch feature/j414s-windows-unified
 *    @ 3994baa8 (this tree) -- src/usb.c, src/kboot_atc.c. This is the exact
 *    deployed preboot binary; its usb2phy/pipehandler register values and the
 *    28-entry tunable vocabulary are ground truth for what m1n1 does TODAY
 *    (which is: usb2phy + pipehandler only -- see docs/j414s-atcphy.md sec 3).
 *    Cited as "usb.c:N" / "kboot_atc.c:N".
 *
 *  - Asahi Linux, /Users/dj/Developer/Asahi/repos/linux-asahi @ 030248d39b40,
 *    branch asahi -- drivers/phy/apple/atc.c (GPL-2.0 OR BSD-2-Clause, used
 *    here under the BSD-2-Clause arm), drivers/soc/apple/tunable.c (GPL-2.0-only
 *    OR MIT, used under MIT), arch/arm64/boot/dts/apple/t602x-dieX.dtsi
 *    (GPL-2.0+ OR MIT). This is the ONLY open-source driver for this PHY and
 *    is independently reverse engineered from XNU debug output (see atc.c's
 *    own file header). Cited as "atc.c:N".
 *
 *  - A live ADT capture of the actual target machine:
 *    ~/Library/Application Support/ntasi/private-hardware-evidence/
 *    20260729-142731-j414s-ans-v5/j414s-adt.bin
 *    sha256 93d96b4a3ea736288278606b723f263361c6ae6c3d5c4f24f08f6f7a73f4b66e.
 *    Parsed with m1n1's own proxyclient/m1n1/adt.py (load_adt + node.get_reg).
 *    This resolves, for real (grade A, not inferred), which raw ADT `reg[]`
 *    index of /arm-io/atc-phy0..3 and /arm-io/usb-drd0..2 corresponds to each
 *    of atc.c's five named MMIO windows (core/lpdptx/axi2af/usb2phy/
 *    pipehandler) AND to each named tunable sub-block in kboot_atc.c's table.
 *    Every one of kboot_atc.c's 28 tunable (adt_name, block_offset) pairs was
 *    independently matched against a distinct reg[] entry at that exact
 *    core-relative offset -- see docs/j414s-atcphy.md sec 2 for the full
 *    reg[] table and the match-up. Cited as "ADT:regN".
 *
 * Label discipline: everything here is static-source-derived and host-tested
 * only. NOTHING in this file is hardware-proven on the J414s -- the machine
 * has a live Windows session on it and this session operated under an
 * absolute hardware-access ban. Register offsets independently confirmed
 * against the live ADT capture are noted as such; bit-field *meanings* remain
 * exactly as reverse engineered by the Asahi atc.c authors from XNU debug
 * strings, i.e. grade B (plausible names, unverified semantics) except where
 * m1n1's own deployed code exercises the same bits (grade A, e.g. usb2phy).
 */

#ifdef ATCPHY_CORE_HOST_TEST
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int32_t s32;
#else
#include "types.h"
#endif

/* ------------------------------------------------------------------ */
/* 1. Topology                                                         */
/* ------------------------------------------------------------------ */

/*
 * T6020 has 4 ATC instances on die 0. Ports 0-2 are USB-C receptacles
 * (paired with hpm0/1/2 on i2c0); port 3 feeds the HDMI connector with no
 * PD controller and is DP-only (dwc3_3 does not exist). This file only
 * models ports 0-2's USB2/USB3/DP behaviour; port 3 is out of scope.
 * (t602x-j414-j416.dtsi, t600x-j314-j316.dtsi -- cited via
 * docs/j414s-atcphy.md, not re-verified against a sparse Linux checkout in
 * this session.)
 */
#define ATCPHY_T6020_USBC_PORT_COUNT 3u

/* ------------------------------------------------------------------ */
/* 2. MMIO windows / blocks                                            */
/* ------------------------------------------------------------------ */

/*
 * Five windows per ATC port (atc.c:2217-2221 reg-names "core"/"lpdptx"/
 * "axi2af"/"usb2phy"/"pipehandler"). On the real ADT (grade A, ADT capture
 * above) these resolve to:
 *   usb2phy      = /arm-io/atc-phyN reg[0]   (already used by m1n1 usb.c)
 *   core         = /arm-io/atc-phyN reg[3]   (NEW -- not used by m1n1 today)
 *   axi2af       = /arm-io/atc-phyN reg[24]  (NEW; 16MB range, real axi2af
 *                                             tunable registers in 1st page)
 *   lpdptx       = /arm-io/atc-phyN reg[20]  (NEW; DP-mode only)
 *   pipehandler  = /arm-io/usb-drdN reg[3]   (already used by m1n1 usb.c,
 *                                             called "drd_regs_unk3" there)
 * All other core-window sub-blocks (AUSPLL, CIO3PLL, per-lane AUSPMA, ATC
 * fabric, DP TX control) live at fixed offsets from the single "core" base
 * and were each independently corroborated by their own reg[] entry --
 * see docs/j414s-atcphy.md sec 2 for the full table.
 */
typedef enum {
    ATCPHY_BLOCK_USB2PHY = 0,
    ATCPHY_BLOCK_CORE,
    ATCPHY_BLOCK_PIPEHANDLER,
    ATCPHY_BLOCK_AXI2AF,
    ATCPHY_BLOCK_LPDPTX,
    ATCPHY_BLOCK_COUNT,
} atcphy_block_t;

/* ---- usb2phy (block USB2PHY); usb.c:59-77, atc.c:464-484 agree ---- */
#define ATCPHY_USB2PHY_USBCTL             0x00u
#define ATCPHY_USB2PHY_USBCTL_RUN         0x2u /* whole-register value, usb.c:60 */
#define ATCPHY_USB2PHY_USBCTL_ISOLATION   0x4u /* whole-register value, usb.c:61 */

#define ATCPHY_USB2PHY_CTL                0x04u
#define ATCPHY_USB2PHY_CTL_RESET          (1u << 0) /* usb.c:64 */
#define ATCPHY_USB2PHY_CTL_PORT_RESET     (1u << 1) /* usb.c:65 */
#define ATCPHY_USB2PHY_CTL_APB_RESET_N    (1u << 2) /* usb.c:66 */
#define ATCPHY_USB2PHY_CTL_SIDDQ          (1u << 3) /* usb.c:67 */

#define ATCPHY_USB2PHY_SIG                          0x08u
#define ATCPHY_USB2PHY_SIG_VBUSDET_FORCE_VAL         (1u << 0)
#define ATCPHY_USB2PHY_SIG_VBUSDET_FORCE_EN          (1u << 1)
#define ATCPHY_USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL      (1u << 2)
#define ATCPHY_USB2PHY_SIG_VBUSVLDEXT_FORCE_EN       (1u << 3)
#define ATCPHY_USB2PHY_SIG_VBUS_MASK                 0xFu /* usb.c:72, all 4 above */
#define ATCPHY_USB2PHY_SIG_HOST                      (7u << 12) /* usb.c:73, atc.c:481 */

#define ATCPHY_USB2PHY_MISCTUNE                0x1Cu
#define ATCPHY_USB2PHY_MISCTUNE_APB_GATE_OFF   (1u << 29) /* usb.c:76 */
#define ATCPHY_USB2PHY_MISCTUNE_REF_GATE_OFF   (1u << 30) /* usb.c:77 */

/* ---- pipehandler (block PIPEHANDLER); atc.c:427-462, usb.c:43-58 agree --- */
#define ATCPHY_PIPEHANDLER_OVERRIDE                0x00u
#define ATCPHY_PIPEHANDLER_OVERRIDE_RXVALID        (1u << 0) /* atc.c:429 */
#define ATCPHY_PIPEHANDLER_OVERRIDE_RXDETECT       (1u << 2) /* atc.c:430 */

#define ATCPHY_PIPEHANDLER_OVERRIDE_VALUES         0x04u
#define ATCPHY_PIPEHANDLER_OVERRIDE_VAL_RXDETECT0  (1u << 1) /* atc.c:433 */
#define ATCPHY_PIPEHANDLER_OVERRIDE_VAL_RXDETECT1  (1u << 2) /* atc.c:434 */
#define ATCPHY_PIPEHANDLER_OVERRIDE_VAL_PHY_STATUS (1u << 4) /* atc.c:435 */

#define ATCPHY_PIPEHANDLER_MUX_CTRL          0x0Cu
#define ATCPHY_PIPEHANDLER_MUX_DATA_SHIFT    0u
#define ATCPHY_PIPEHANDLER_MUX_DATA_MASK     0x7u
#define ATCPHY_PIPEHANDLER_MUX_CLK_SHIFT     3u
#define ATCPHY_PIPEHANDLER_MUX_CLK_MASK      0x38u
#define ATCPHY_PIPEHANDLER_MUX_CLK_OFF       0u /* atc.c:440 */
#define ATCPHY_PIPEHANDLER_MUX_CLK_USB3      1u /* atc.c:441 */
#define ATCPHY_PIPEHANDLER_MUX_CLK_USB4      2u /* atc.c:442 */
#define ATCPHY_PIPEHANDLER_MUX_CLK_DUMMY     4u /* atc.c:443 */
#define ATCPHY_PIPEHANDLER_MUX_DATA_USB3     0u /* atc.c:445 */
#define ATCPHY_PIPEHANDLER_MUX_DATA_USB4     1u /* atc.c:446 */
#define ATCPHY_PIPEHANDLER_MUX_DATA_DUMMY    2u /* atc.c:447 */

/*
 * m1n1's whole-register bring-up writes (usb.c:42-44) decode exactly onto
 * the CLK/DATA fields above: 0x08 = CLK_USB3|DATA_USB3, 0x11 =
 * CLK_USB4|DATA_USB4, 0x22 = CLK_DUMMY|DATA_DUMMY. m1n1 always writes the
 * whole register in one shot; atc.c always sequences CLK then DATA then CLK
 * again with 10us delays (see atcphy_seq_pipehandler_dummy /
 * _usb3_host_bist below). Both are faithfully modeled.
 */
#define ATCPHY_PIPEHANDLER_MUX_VALUE_USB3         0x08u
#define ATCPHY_PIPEHANDLER_MUX_VALUE_USB4_TUNNEL  0x11u
#define ATCPHY_PIPEHANDLER_MUX_VALUE_DUMMY        0x22u

#define ATCPHY_PIPEHANDLER_LOCK_REQ  0x10u /* atc.c:449 */
#define ATCPHY_PIPEHANDLER_LOCK_ACK  0x14u /* atc.c:450 */
#define ATCPHY_PIPEHANDLER_LOCK_EN   (1u << 0)
#define ATCPHY_PIPEHANDLER_LOCK_ACK_TIMEOUT_US 1000u /* atc.c:462 */

#define ATCPHY_PIPEHANDLER_AON_GEN                       0x1Cu
#define ATCPHY_PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN   (1u << 4) /* atc.c:454 */
#define ATCPHY_PIPEHANDLER_AON_GEN_DWC3_RESET_N          (1u << 0) /* atc.c:455 */

#define ATCPHY_PIPEHANDLER_NONSELECTED_OVERRIDE   0x20u
#define ATCPHY_PIPEHANDLER_NATIVE_RESET           (1u << 12) /* atc.c:458 */
#define ATCPHY_PIPEHANDLER_DUMMY_PHY_EN           (1u << 15) /* atc.c:459 */
#define ATCPHY_PIPEHANDLER_NATIVE_POWER_DOWN_SHIFT 0u
#define ATCPHY_PIPEHANDLER_NATIVE_POWER_DOWN_MASK  0xFu /* atc.c:460 */

/* ---- core (block CORE); all offsets absolute from the core base ---- */

/* ADT:reg3 -- CFG0/CROSSBAR/LANE_MODE/BIST/SLEEP_CTRL/PLL_FSM/PLL_COMMON  */
#define ATCPHY_CORE_ACIOPHY_CFG0                  0x08u /* atc.c:126 */
#define ATCPHY_CORE_ACIOPHY_CFG0_COMMON_BIG_OV     (1u << 1)  /* atc.c:127 */
#define ATCPHY_CORE_ACIOPHY_CFG0_COMMON_SMALL_OV   (1u << 3)  /* atc.c:128 */
#define ATCPHY_CORE_ACIOPHY_CFG0_COMMON_CLAMP_OV   (1u << 5)  /* atc.c:129 */
#define ATCPHY_CORE_ACIOPHY_CFG0_RX_SMALL_OV_SHIFT 8u
#define ATCPHY_CORE_ACIOPHY_CFG0_RX_SMALL_OV_MASK  0x300u /* GENMASK(9,8), atc.c:130 */
#define ATCPHY_CORE_ACIOPHY_CFG0_RX_BIG_OV_SHIFT   12u
#define ATCPHY_CORE_ACIOPHY_CFG0_RX_BIG_OV_MASK    0x3000u /* GENMASK(13,12), atc.c:131 */
#define ATCPHY_CORE_ACIOPHY_CFG0_RX_CLAMP_OV_SHIFT 16u
#define ATCPHY_CORE_ACIOPHY_CFG0_RX_CLAMP_OV_MASK  0x30000u /* GENMASK(17,16), atc.c:132 */

#define ATCPHY_CORE_ACIOPHY_CROSSBAR                  0x4Cu /* atc.c:134, ADT:reg3 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_MASK    0x1Fu /* GENMASK(4,0), atc.c:135 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB4          0x00u /* atc.c:136 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED  0x01u /* atc.c:137 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3          0x0Au /* atc.c:138 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED  0x0Bu /* atc.c:139 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP         0x10u /* atc.c:140 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED 0x11u /* atc.c:141 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_DP              0x14u /* atc.c:142 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_SHIFT 5u
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_MASK  0x1FFE0u /* GENMASK(16,5), atc.c:143 */
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE   0x000u
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100 0x100u
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008 0x008u
#define ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_BOTH_PMA          (1u << 17) /* atc.c:147 */

#define ATCPHY_CORE_ACIOPHY_LANE_MODE          0x48u /* atc.c:149, ADT:reg3 */
#define ATCPHY_CORE_LANE_MODE_RX0_SHIFT 0u
#define ATCPHY_CORE_LANE_MODE_RX0_MASK  0x7u  /* GENMASK(2,0), atc.c:150 */
#define ATCPHY_CORE_LANE_MODE_TX0_SHIFT 3u
#define ATCPHY_CORE_LANE_MODE_TX0_MASK  0x38u /* GENMASK(5,3), atc.c:151 */
#define ATCPHY_CORE_LANE_MODE_RX1_SHIFT 6u
#define ATCPHY_CORE_LANE_MODE_RX1_MASK  0x1C0u /* GENMASK(8,6), atc.c:152 */
#define ATCPHY_CORE_LANE_MODE_TX1_SHIFT 9u
#define ATCPHY_CORE_LANE_MODE_TX1_MASK  0xE00u /* GENMASK(11,9), atc.c:153 */

typedef enum {
    ATCPHY_LANE_MODE_USB4 = 0, /* atc.c:156 */
    ATCPHY_LANE_MODE_USB3 = 1, /* atc.c:157 */
    ATCPHY_LANE_MODE_DP = 2,   /* atc.c:158 */
    ATCPHY_LANE_MODE_OFF = 3,  /* atc.c:159 */
} atcphy_lane_mode_t;

#define ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1              0x84u /* atc.c:162 */
#define ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1_CLK_EN       (1u << 27) /* atc.c:163 */
#define ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1_BIST_EN      (1u << 28) /* atc.c:164 */

#define ATCPHY_CORE_TOP_BIST_OV_CFG                   0x8Cu /* atc.c:166 */
#define ATCPHY_CORE_TOP_BIST_OV_CFG_LN0_RESET_N_OV    (1u << 13) /* atc.c:167 */
#define ATCPHY_CORE_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV   (1u << 25) /* atc.c:168 */

#define ATCPHY_CORE_TOP_BIST_READ_CTRL                       0x90u /* atc.c:170 */
#define ATCPHY_CORE_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE     (1u << 2) /* atc.c:171 */

#define ATCPHY_CORE_TOP_PHY_STAT             0x9Cu /* atc.c:173 */
#define ATCPHY_CORE_TOP_PHY_STAT_LN0_UNK0    (1u << 0)  /* atc.c:174 */
#define ATCPHY_CORE_TOP_PHY_STAT_LN0_UNK23   (1u << 23) /* atc.c:175 */

#define ATCPHY_CORE_TOP_BIST_PHY_CFG0             0xA8u /* atc.c:177 */
#define ATCPHY_CORE_TOP_BIST_PHY_CFG0_LN0_RESET_N (1u << 0) /* atc.c:178 */

#define ATCPHY_CORE_TOP_BIST_PHY_CFG1                    0xACu /* atc.c:180 */
#define ATCPHY_CORE_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_SHIFT 10u
#define ATCPHY_CORE_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_MASK  0x3C00u /* GENMASK(13,10), atc.c:181 */

#define ATCPHY_CORE_ACIOPHY_SLEEP_CTRL                0x1B0u /* atc.c:183 */
#define ATCPHY_CORE_SLEEP_CTRL_TX_BIG_OV_SHIFT   2u
#define ATCPHY_CORE_SLEEP_CTRL_TX_BIG_OV_MASK    0xCu /* GENMASK(3,2), atc.c:184 */
#define ATCPHY_CORE_SLEEP_CTRL_TX_SMALL_OV_SHIFT 6u
#define ATCPHY_CORE_SLEEP_CTRL_TX_SMALL_OV_MASK  0xC0u /* GENMASK(7,6), atc.c:185 */
#define ATCPHY_CORE_SLEEP_CTRL_TX_CLAMP_OV_SHIFT 10u
#define ATCPHY_CORE_SLEEP_CTRL_TX_CLAMP_OV_MASK  0xC00u /* GENMASK(11,10), atc.c:186 */

#define ATCPHY_CORE_PLL_PCTL_FSM_CTRL1     0x1014u /* atc.c:47,188 (dup macro name in Linux) */
#define ATCPHY_CORE_PLL_COMMON_CTRL        0x1028u /* atc.c:190 */
#define ATCPHY_CORE_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT (1u << 24) /* atc.c:191 */

/* ADT:reg2 (also core+0x20000, same silicon, two ADT apertures) */
#define ATCPHY_CORE_ATCPHY_POWER_CTRL        0x20000u /* atc.c:193 */
#define ATCPHY_CORE_ATCPHY_POWER_STAT        0x20004u /* atc.c:194 */
#define ATCPHY_CORE_POWER_SLEEP_SMALL   (1u << 0) /* atc.c:195 */
#define ATCPHY_CORE_POWER_SLEEP_BIG     (1u << 1) /* atc.c:196 */
#define ATCPHY_CORE_POWER_CLAMP_EN      (1u << 2) /* atc.c:197 */
#define ATCPHY_CORE_POWER_APB_RESET_N   (1u << 3) /* atc.c:198 */
#define ATCPHY_CORE_POWER_PHY_RESET_N   (1u << 4) /* atc.c:199 */

#define ATCPHY_CORE_ATCPHY_MISC              0x20008u /* atc.c:200 */
#define ATCPHY_CORE_MISC_RESET_N        (1u << 0) /* atc.c:201 */
#define ATCPHY_CORE_MISC_LANE_SWAP      (1u << 2) /* atc.c:202 */

/* ADT:reg4 (AUSPLL "top") -- APB command handshake + freq descriptors */
#define ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE          0x2000u /* atc.c:49 */
#define ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_REQ      (1u << 0)  /* atc.c:50 */
#define ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_ACK      (1u << 1)  /* atc.c:51 */
#define ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_UNK28    (1u << 28) /* atc.c:52 */
#define ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_CMD_SHIFT 3u
#define ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_CMD_MASK  0xFFFFF8u /* GENMASK(27,3), atc.c:53 */

#define ATCPHY_CORE_AUSPLL_FREQ_DESC_A                 0x2080u /* atc.c:55 */
#define ATCPHY_CORE_AUSPLL_FD_FREQ_COUNT_TARGET_SHIFT  0u
#define ATCPHY_CORE_AUSPLL_FD_FREQ_COUNT_TARGET_MASK   0x3FFu /* GENMASK(9,0), atc.c:56 */
#define ATCPHY_CORE_AUSPLL_FD_FBDIVN_HALF              (1u << 10) /* atc.c:57 */
#define ATCPHY_CORE_AUSPLL_FD_REV_DIVN_MASK            0x3800u /* GENMASK(13,11), atc.c:58 */
#define ATCPHY_CORE_AUSPLL_FD_KI_MAN_SHIFT 14u
#define ATCPHY_CORE_AUSPLL_FD_KI_MAN_MASK  0x3C000u /* GENMASK(17,14), atc.c:59 */
#define ATCPHY_CORE_AUSPLL_FD_KI_EXP_SHIFT 18u
#define ATCPHY_CORE_AUSPLL_FD_KI_EXP_MASK  0x3C0000u /* GENMASK(21,18), atc.c:60 */
#define ATCPHY_CORE_AUSPLL_FD_KP_MAN_SHIFT 22u
#define ATCPHY_CORE_AUSPLL_FD_KP_MAN_MASK  0x3C00000u /* GENMASK(25,22), atc.c:61 */
#define ATCPHY_CORE_AUSPLL_FD_KP_EXP_SHIFT 26u
#define ATCPHY_CORE_AUSPLL_FD_KP_EXP_MASK  0x3C000000u /* GENMASK(29,26), atc.c:62 */
#define ATCPHY_CORE_AUSPLL_FD_KPKI_SCALE_HBW_MASK 0xC0000000u /* GENMASK(31,30), atc.c:63 */

#define ATCPHY_CORE_AUSPLL_FREQ_DESC_B              0x2084u /* atc.c:65 */
#define ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_DEN_SHIFT 0u
#define ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_DEN_MASK  0x3FFFu /* GENMASK(13,0), atc.c:66 */
#define ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_NUM_SHIFT 14u
#define ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_NUM_MASK  0xFFFC000u /* GENMASK(27,14), atc.c:67 */

#define ATCPHY_CORE_AUSPLL_FREQ_DESC_C            0x2088u /* atc.c:69 */
#define ATCPHY_CORE_AUSPLL_FD_SDM_SSC_STEP_MASK   0xFFu /* GENMASK(7,0), atc.c:70 */
#define ATCPHY_CORE_AUSPLL_FD_SDM_SSC_EN          (1u << 8) /* atc.c:71 */
#define ATCPHY_CORE_AUSPLL_FD_PCLK_DIV_SEL_SHIFT  9u
#define ATCPHY_CORE_AUSPLL_FD_PCLK_DIV_SEL_MASK   0x3E00u /* GENMASK(13,9), atc.c:72 */
#define ATCPHY_CORE_AUSPLL_FD_LFSDM_DIV_SHIFT     14u
#define ATCPHY_CORE_AUSPLL_FD_LFSDM_DIV_MASK      0xC000u /* GENMASK(15,14), atc.c:73 */
#define ATCPHY_CORE_AUSPLL_FD_LFCLK_CTRL_SHIFT    16u
#define ATCPHY_CORE_AUSPLL_FD_LFCLK_CTRL_MASK     0xF0000u /* GENMASK(19,16), atc.c:74 */
#define ATCPHY_CORE_AUSPLL_FD_VCLK_OP_DIVN_SHIFT  20u
#define ATCPHY_CORE_AUSPLL_FD_VCLK_OP_DIVN_MASK   0x300000u /* GENMASK(21,20), atc.c:75 */
#define ATCPHY_CORE_AUSPLL_FD_VCLK_PRE_DIVN       (1u << 22) /* atc.c:76 */

#define ATCPHY_CORE_AUSPLL_CLKOUT_DIV                     0x2208u /* atc.c:90 */
#define ATCPHY_CORE_AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI_SHIFT 16u
#define ATCPHY_CORE_AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI_MASK  0x1F0000u /* GENMASK(20,16), atc.c:91 */

/* ADT:reg5 (AUSPLL "core") */
#define ATCPHY_CORE_AUSPLL_CLKOUT_MASTER                       0x2200u /* atc.c:85 */
#define ATCPHY_CORE_AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN          (1u << 2) /* atc.c:86 */
#define ATCPHY_CORE_AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN         (1u << 4) /* atc.c:87 */
#define ATCPHY_CORE_AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN     (1u << 6) /* atc.c:88 */
#define ATCPHY_CORE_AUSPLL_BGR                    0x2214u /* atc.c:93 */
#define ATCPHY_CORE_AUSPLL_BGR_CTRL_AVAIL         (1u << 0) /* atc.c:94 */
#define ATCPHY_CORE_AUSPLL_CLKOUT_DTC_VREG        0x2220u /* atc.c:96 */
#define ATCPHY_CORE_AUSPLL_DTC_VREG_BYPASS        (1u << 7) /* atc.c:98 */
#define ATCPHY_CORE_AUSPLL_FREQ_CFG               0x2224u /* atc.c:100 */
#define ATCPHY_CORE_AUSPLL_FREQ_REFCLK_MASK       0x3u /* GENMASK(1,0), atc.c:101 */

/* ADT:reg6 (CIO3PLL "top"), ADT:reg7 (CIO3PLL "core") */
#define ATCPHY_CORE_CIO3PLL_CLK_CTRL          0x2A00u /* atc.c:112, ADT:reg7 */
#define ATCPHY_CORE_CIO3PLL_CLK_PCLK_EN       (1u << 1) /* atc.c:113 */
#define ATCPHY_CORE_CIO3PLL_CLK_REFCLK_EN     (1u << 5) /* atc.c:114 */

/* ADT:reg9 (AUS_CMN_SHM), ADT:reg10 (AUS_CMN_TOP) */
#define ATCPHY_CORE_AUS_COMMON_SHIM_BLK_VREG  0x0A04u /* atc.c:103 */
#define ATCPHY_CORE_AUS_UNK_A20                0x0A20u /* atc.c:106 */
#define ATCPHY_CORE_AUS_UNK_A20_TX_CAL_CODE_SHIFT 20u
#define ATCPHY_CORE_AUS_UNK_A20_TX_CAL_CODE_MASK  0xF00000u /* GENMASK(23,20), atc.c:107 */
#define ATCPHY_CORE_ACIOPHY_CMN_SHM_STS_REG0             0x0A74u /* atc.c:109 */
#define ATCPHY_CORE_ACIOPHY_CMN_SHM_STS_REG0_CMD_READY   (1u << 0) /* atc.c:110 */

/* ADT:reg8 -- DP TX control block (shared, not per-lane) */
#define ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0    0x7000u /* atc.c:205 */
#define ATCPHY_CORE_DP_PMA_BYTECLK_RESET       (1u << 0) /* atc.c:206 */
#define ATCPHY_CORE_DP_MAC_DIV20_CLK_SEL       (1u << 1) /* atc.c:207 */
#define ATCPHY_CORE_DPTXPHY_PMA_LANE_RESET_N   (1u << 2) /* atc.c:208 */
#define ATCPHY_CORE_DPTXPHY_PMA_LANE_RESET_N_OV (1u << 3) /* atc.c:209 */
#define ATCPHY_CORE_DPTX_PCLK1_SELECT_SHIFT 4u
#define ATCPHY_CORE_DPTX_PCLK1_SELECT_MASK  0x70u /* GENMASK(6,4), atc.c:210 */
#define ATCPHY_CORE_DPTX_PCLK2_SELECT_SHIFT 7u
#define ATCPHY_CORE_DPTX_PCLK2_SELECT_MASK  0x380u /* GENMASK(9,7), atc.c:211 */
#define ATCPHY_CORE_DPRX_PCLK_SELECT_SHIFT  10u
#define ATCPHY_CORE_DPRX_PCLK_SELECT_MASK   0x1C00u /* GENMASK(12,10), atc.c:212 */
#define ATCPHY_CORE_DPTX_PCLK1_ENABLE  (1u << 13) /* atc.c:213 */
#define ATCPHY_CORE_DPTX_PCLK2_ENABLE  (1u << 14) /* atc.c:214 */
#define ATCPHY_CORE_DPRX_PCLK_ENABLE   (1u << 15) /* atc.c:215 */
#define ATCPHY_CORE_ACIOPHY_DP_PCLK_STAT    0x7044u /* atc.c:217 */
#define ATCPHY_CORE_ACIOPHY_AUSPLL_LOCK     (1u << 3) /* atc.c:218 */

/* ADT:reg25 -- ATC fabric (tunable target only, no named bits used here) */
#define ATCPHY_CORE_ATC_FABRIC_OFFSET 0x45000u /* kboot_atc.c:72, ADT:reg25 */

/* Per-lane AUSPMA sub-blocks (ADT:reg17/26/15/13/11 = lane0,
 * ADT:reg18/27/16/14/12 = lane1). Only the RX_TOP PMAFSM sub-register is
 * used outside the tunable blobs and the (unported, see sec 9 of the design
 * doc) per-lane DP analog calibration function. */
#define ATCPHY_CORE_LN0_AUSPMA_RX_TOP 0x9000u  /* atc.c:220, ADT:reg17 */
#define ATCPHY_CORE_LN0_AUSPMA_RX_EQ  0xA000u  /* atc.c:221, ADT:reg26 */
#define ATCPHY_CORE_LN0_AUSPMA_RX_SHM 0xB000u  /* atc.c:222, ADT:reg15 */
#define ATCPHY_CORE_LN0_AUSPMA_TX_TOP 0xC000u  /* atc.c:223, ADT:reg13 */
#define ATCPHY_CORE_LN0_AUSPMA_TX_SHM 0xD000u  /* atc.c:224, ADT:reg11 */
#define ATCPHY_CORE_LN1_AUSPMA_RX_TOP 0x10000u /* atc.c:226, ADT:reg18 */
#define ATCPHY_CORE_LN1_AUSPMA_RX_EQ  0x11000u /* atc.c:227, ADT:reg27 */
#define ATCPHY_CORE_LN1_AUSPMA_RX_SHM 0x12000u /* atc.c:228, ADT:reg16 */
#define ATCPHY_CORE_LN1_AUSPMA_TX_TOP 0x13000u /* atc.c:229, ADT:reg14 */
#define ATCPHY_CORE_LN1_AUSPMA_TX_SHM 0x14000u /* atc.c:230, ADT:reg12 */

#define ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM         0x0010u /* atc.c:232, relative */
#define ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV  (1u << 0) /* atc.c:233 */
#define ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ (1u << 9) /* atc.c:234 */

/* ---- lpdptx (block LPDPTX); atc.c:402-425. DP-mode-only window. ---- */
#define ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_CTRL          0x0000u
#define ATCPHY_LPDPTX_BLK_AUX_CTRL_PWRDN            (1u << 4)
#define ATCPHY_LPDPTX_BLK_AUX_RXOFFSET_SHIFT        22u
#define ATCPHY_LPDPTX_BLK_AUX_RXOFFSET_MASK         0x3C00000u /* GENMASK(25,22) */
#define ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_LDO_CTRL       0x0008u
#define ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_MARGIN          0x000Cu
#define ATCPHY_LPDPTX_MARGIN_RCAL_RXOFFSET_EN       (1u << 5)
#define ATCPHY_LPDPTX_AUX_MARGIN_RCAL_TXSWING_SHIFT 6u
#define ATCPHY_LPDPTX_AUX_MARGIN_RCAL_TXSWING_MASK  0x7C0u /* GENMASK(10,6) */
#define ATCPHY_LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG0 0x0204u /* ADT:reg21 */
#define ATCPHY_LPDPTX_CFG_PMA_AUX_SEL_LF_DATA       (1u << 15)
#define ATCPHY_LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1 0x0208u
#define ATCPHY_LPDPTX_CFG_PMA_PHYS_ADJ_SHIFT        20u
#define ATCPHY_LPDPTX_CFG_PMA_PHYS_ADJ_MASK         0x700000u /* GENMASK(22,20) */
#define ATCPHY_LPDPTX_CFG_PMA_PHYS_ADJ_OV           (1u << 19)
#define ATCPHY_LPDPTX_AUX_CONTROL          0x4000u /* ADT:reg22 */
#define ATCPHY_LPDPTX_AUX_PWN_DOWN         0x10u
#define ATCPHY_LPDPTX_AUX_CLAMP_EN         0x04u
#define ATCPHY_LPDPTX_SLEEP_B_BIG_IN       0x02u
#define ATCPHY_LPDPTX_SLEEP_B_SML_IN       0x01u
#define ATCPHY_LPDPTX_TXTERM_CODEMSB       0x400u
#define ATCPHY_LPDPTX_TXTERM_CODE_SHIFT    5u
#define ATCPHY_LPDPTX_TXTERM_CODE_MASK     0x3E0u /* GENMASK(9,5) */

/* ------------------------------------------------------------------ */
/* 3. PHY operating modes and the mode x orientation table             */
/* ------------------------------------------------------------------ */

/*
 * atc.c:512-528 (enum atcphy_mode). USB3-only intentionally programs the
 * "other" lane as DP (atc.c comment at 682-684: USB3/USB3 does not work
 * and 20Gbps is unsupported); the only USB3 vs USB3_DP difference is DP-AUX
 * enablement.
 */
typedef enum {
    ATCPHY_MODE_OFF = 0,
    ATCPHY_MODE_USB2,
    ATCPHY_MODE_USB3,
    ATCPHY_MODE_USB3_DP,
    ATCPHY_MODE_TBT,
    ATCPHY_MODE_USB4,
    ATCPHY_MODE_DP,
    ATCPHY_MODE_COUNT,
} atcphy_mode_t;

typedef enum {
    ATCPHY_PIPE_STATE_DUMMY = 0,
    ATCPHY_PIPE_STATE_USB3,
    ATCPHY_PIPE_STATE_USB4, /* atc.c:1133-1136: unimplemented upstream too;
                              * this file falls back to DUMMY for it, same
                              * as Linux does. */
} atcphy_pipe_state_t;

typedef struct {
    u8 crossbar_protocol;
    u16 crossbar_dp_single_pma;
    bool crossbar_dp_both_pma;
    u8 lane_mode[2]; /* atcphy_lane_mode_t */
    bool dp_lane[2];
    bool set_swap; /* ATCPHY_MISC_LANE_SWAP */
    bool enable_dp_aux;
    atcphy_pipe_state_t pipe_state;
} atcphy_mode_config_t;

/*
 * Mode x orientation configuration, transcribed field-for-field from
 * atcphy_modes[] (atc.c:634-786, i.e. lines 634+6=640 through 785 in the
 * pinned checkout -- see docs/j414s-atcphy.md sec 4 for the full transcript
 * table). Returns NULL for an invalid mode.
 */
const atcphy_mode_config_t *atcphy_mode_config(atcphy_mode_t mode, bool swapped);

/* Which PIPE backend a MUX_CTRL value selects (reverse of the CLK/DATA
 * field encode); used for diagnostics / classifying captured state. */
typedef enum {
    ATCPHY_PIPE_BACKEND_USB3 = 0,
    ATCPHY_PIPE_BACKEND_USB4,
    ATCPHY_PIPE_BACKEND_DUMMY,
    ATCPHY_PIPE_BACKEND_CLK_OFF,
    ATCPHY_PIPE_BACKEND_INCONSISTENT,
} atcphy_pipe_backend_t;

atcphy_pipe_backend_t atcphy_pipe_mux_decode(u32 mux_ctrl);

/* ------------------------------------------------------------------ */
/* 4. DisplayPort link rates                                           */
/* ------------------------------------------------------------------ */

typedef enum {
    ATCPHY_DP_RATE_RBR = 0,
    ATCPHY_DP_RATE_HBR,
    ATCPHY_DP_RATE_HBR2,
    ATCPHY_DP_RATE_HBR3,
    ATCPHY_DP_RATE_COUNT,
} atcphy_dp_rate_t;

/* DPCD-style link-bw wire codes and kHz, atcphy_dpphy_configure atc.c
 * 1908-1921 (RBR/HBR/HBR2/HBR3 = 1620/2700/5400/8100 kHz), and the standard
 * DP wire codes 0x06/0x0a/0x14/0x1e which are also fixed by the DisplayPort
 * spec itself (uncopyrightable facts). This PHY has no UHBR; HBR3 is the
 * ceiling (atc.c has no rate above HBR3 anywhere in dp_lr_config[]).
 */
#define ATCPHY_DP_WIRE_RBR  0x06u
#define ATCPHY_DP_WIRE_HBR  0x0Au
#define ATCPHY_DP_WIRE_HBR2 0x14u
#define ATCPHY_DP_WIRE_HBR3 0x1Eu

int atcphy_dp_rate_from_wire(u32 wire_code); /* -1 if unknown */
u32 atcphy_dp_rate_to_wire(atcphy_dp_rate_t rate);
u32 atcphy_dp_rate_khz(atcphy_dp_rate_t rate);

/* AUSPLL frequency-descriptor configuration per link rate, dp_lr_config[]
 * (atc.c:787-831). Field names are from XNU debug output per atc.c's own
 * file comment -- meanings beyond "PLL tuning knob" are NOT independently
 * understood by this file's author either. */
typedef struct {
    u16 freqinit_count_target;
    u16 fbdivn_frac_den;
    u16 fbdivn_frac_num;
    u8 pclk_div_sel;
    u8 lfclk_ctrl;
    u8 vclk_op_divn;
    bool plla_clkout_vreg_bypass;
    bool txa_ldoclk_bypass;
    bool txa_div2_en;
} atcphy_dp_lr_config_t;

const atcphy_dp_lr_config_t *atcphy_dp_lr_config(atcphy_dp_rate_t rate);

/* ------------------------------------------------------------------ */
/* 5. m1n1 tunable vocabulary (atc-phy,t6020) -- now ADT-address-resolved */
/* ------------------------------------------------------------------ */

typedef enum {
    ATCPHY_TUNABLE_TARGET_AXI2AF = 0,
    ATCPHY_TUNABLE_TARGET_CORE,
} atcphy_tunable_target_t;

typedef enum {
    ATCPHY_TUNABLE_SCOPE_GLOBAL = 0, /* applied after power on/reset */
    ATCPHY_TUNABLE_SCOPE_LANE_USB3,
    ATCPHY_TUNABLE_SCOPE_LANE_DP,
    ATCPHY_TUNABLE_SCOPE_LANE_CIO, /* USB4/Thunderbolt -- unused by this file */
} atcphy_tunable_scope_t;

typedef struct {
    const char *adt_name;
    atcphy_tunable_target_t target;
    u32 block_offset; /* added to each record's 24-bit ADT offset; core- or
                        * axi2af-relative absolute offset, ADT-confirmed --
                        * see docs/j414s-atcphy.md sec 2 */
    u32 block_size;   /* record offset+4 must stay inside this (m1n1's own
                        * validation bound, kboot_atc.c:222-244; NOT
                        * necessarily the true hardware window size) */
    bool required;
    atcphy_tunable_scope_t scope;
    u8 lane; /* 0/1 for lane-scoped entries, 0 for global */
} atcphy_tunable_vocab_t;

/* 10 global + 2 DP-lane + 8 USB3-lane + 8 CIO-lane = 28, kboot_atc.c:69-100 */
#define ATCPHY_T6020_TUNABLE_COUNT 28u

const atcphy_tunable_vocab_t *atcphy_t6020_tunable_vocab(size_t *count);
const atcphy_tunable_vocab_t *atcphy_t6020_tunable_lookup(const char *adt_name);

/* One 12-byte ADT tunable wire record: {offset:24, size:8, mask:32, value:32}
 * (kboot_atc.c:38-44 struct atc_tunable). */
typedef struct {
    u32 offset; /* 24 bits used */
    u8 size;    /* must be 32 (bits) */
    u32 mask;
    u32 value;
} atcphy_tunable_record_t;

typedef enum {
    ATCPHY_TUNABLE_BLOB_OK = 0,
    ATCPHY_TUNABLE_BLOB_EMPTY,      /* length 0: valid, no-op (t6020 fuses) */
    ATCPHY_TUNABLE_BLOB_BAD_LENGTH, /* not a multiple of 12 bytes */
    ATCPHY_TUNABLE_BLOB_BAD_RECORD, /* size/alignment/range violation */
} atcphy_tunable_blob_verdict_t;

/*
 * Validates a raw ADT tunable property blob exactly as m1n1's
 * dt_append_atc_tunable does (kboot_atc.c:206-256): length % 12 == 0,
 * every record size == 32, offset % 4 == 0, offset+4 <= vocab->block_size.
 * Decodes accepted records into out[] (caller-sized, returns count via
 * *out_count) for the glue layer to apply via read-modify-write.
 */
atcphy_tunable_blob_verdict_t atcphy_tunable_validate(const atcphy_tunable_vocab_t *vocab,
                                                       const u8 *blob, size_t blob_len,
                                                       atcphy_tunable_record_t *out,
                                                       size_t out_max, size_t *out_count,
                                                       size_t *bad_record_index);

/* Pure RMW helper mirroring apple_tunable_apply (tunable.c:62-75):
 * new = (old & ~mask) | value. */
u32 atcphy_tunable_apply_one(u32 old_value, u32 mask, u32 value);

/*
 * t6020 fuse policy (kboot_atc.c:130-134,186-193): unlike t8103, t6020 needs
 * no eFuse-derived tuning at all; m1n1 emits an intentionally-empty
 * "apple,tunable-fuses" FDT property for Linux. There is nothing for this
 * file's direct-apply path to do for fuses on t6020 -- recorded here so a
 * future porter doesn't go looking for a fuse table that doesn't exist.
 */
typedef enum {
    ATCPHY_FUSES_REQUIRED = 0,          /* t8103-class: table-driven, NOT modeled here */
    ATCPHY_FUSES_EMPTY_INTENTIONAL = 1, /* t6020-class: this machine */
    ATCPHY_FUSES_UNKNOWN = 2,
} atcphy_fuse_policy_t;

atcphy_fuse_policy_t atcphy_fuse_policy(const char *adt_compatible);

/* ------------------------------------------------------------------ */
/* 6. Declarative op-list sequences + interpreter                      */
/* ------------------------------------------------------------------ */

typedef enum {
    ATCPHY_OP_WRITE = 0,  /* reg = arg1 */
    ATCPHY_OP_SET,        /* reg |= arg1 */
    ATCPHY_OP_CLEAR,      /* reg &= ~arg1 */
    ATCPHY_OP_MASK,       /* reg = (reg & ~arg1) | arg2   (arg1=clear-mask) */
    ATCPHY_OP_DELAY_US,   /* delay arg1 microseconds; block/offset unused */
    ATCPHY_OP_POLL_SET,   /* wait until (reg & arg1) == arg1, timeout arg2 us */
    ATCPHY_OP_POLL_CLEAR, /* wait until (reg & arg1) == 0,    timeout arg2 us */
} atcphy_op_kind_t;

typedef struct {
    atcphy_block_t block;
    atcphy_op_kind_t kind;
    u32 offset;
    u32 arg1;
    u32 arg2;
    const char *cite; /* atc.c source line, for auditability */
} atcphy_seq_op_t;

typedef u32 (*atcphy_seq_read_fn)(void *ctx, atcphy_block_t block, u32 offset);
typedef void (*atcphy_seq_write_fn)(void *ctx, atcphy_block_t block, u32 offset, u32 value);
typedef void (*atcphy_seq_delay_fn)(void *ctx, u32 us);

/*
 * Applies an op list against a caller-provided register file via callbacks.
 * Returns 0 if every op completed (including all polls succeeding before
 * their timeout), or -1 if a POLL op never saw its condition; *fail_index
 * (if non-NULL) is set to the failing op's index. delay_fn may be NULL, in
 * which case DELAY_US is a no-op and polls just call read_fn arg2 times
 * (host tests use this to avoid a real 10000-iteration spin).
 */
int atcphy_seq_apply(const atcphy_seq_op_t *ops, size_t count, atcphy_seq_read_fn read_fn,
                     atcphy_seq_write_fn write_fn, atcphy_seq_delay_fn delay_fn, void *ctx,
                     size_t *fail_index);

/* ---- Fixed (parameter-free) sequences, transcribed from atc.c ---- */

/* usb2 PHY power on/off. atcphy_usb2_power_on atc.c:1669-1692,
 * atcphy_usb2_power_off atc.c:1616-1635. Functionally the union of what
 * m1n1's usb_phy_bringup/usb_phy_handoff_host already do today (usb.c),
 * transcribed here from the upstream Linux driver's own sequencing so the
 * mode-config apply path (sec 7 below) can be self-contained. */
const atcphy_seq_op_t *atcphy_seq_usb2_power_on(size_t *count);
const atcphy_seq_op_t *atcphy_seq_usb2_power_off(size_t *count);

/* Core "small"/"big" power domain wake + APB reset release / power off.
 * atcphy_power_on atc.c:1694-1723, atcphy_power_off atc.c:1637-1667. NEW:
 * m1n1 never runs this sequence today (see docs/j414s-atcphy.md sec 3). */
const atcphy_seq_op_t *atcphy_seq_core_power_on(size_t *count);
const atcphy_seq_op_t *atcphy_seq_core_power_off(size_t *count);

/* ACIOPHY_CFG0 / ACIOPHY_SLEEP_CTRL "override" dance that atcphy_configure
 * runs on every mode change after tunables are applied. atc.c:1746-1771. */
const atcphy_seq_op_t *atcphy_seq_cfg0_sleep_override(size_t *count);

/* CIO3PLL clock enable. atc.c:1778-1779. */
const atcphy_seq_op_t *atcphy_seq_cio3pll_enable(size_t *count);

/*
 * Two unconditional AUSPLL override writes atcphy_configure runs right
 * after atcphy_apply_tunables and before the CFG0/SLEEP_CTRL dance
 * (atc.c:1743-1744). 0x1fe000 is an opaque magic constant in atc.c itself
 * (no named bitfield) -- kept opaque here too.
 */
const atcphy_seq_op_t *atcphy_seq_auspll_fsm_override(size_t *count);

/* Final step of atcphy_configure: release the USB3 PHY reset line.
 * atc.c:1783. Must run AFTER lane config, per atc.c's ordering. */
const atcphy_seq_op_t *atcphy_seq_release_phy_reset(size_t *count);

/* Pipehandler -> dummy (USB2-only) backend. atcphy_configure_pipehandler_dummy
 * atc.c:1081-1120. This is the ONLY pipehandler state m1n1 ever uses today. */
const atcphy_seq_op_t *atcphy_seq_pipehandler_dummy(size_t *count);

/*
 * Pipehandler -> USB3 backend, HOST-mode variant only (device mode is not
 * modeled: m1n1 only ever configures these ports as USB host, usb.c:209).
 * atcphy_configure_pipehandler_usb3(atcphy, host=true), atc.c:975-1079.
 *
 * *** SAFETY: this sequence reprograms the PIPE mux DWC3 depends on for its
 * SuperSpeed data path. It must NEVER be invoked while a guest OS already
 * has an active xHCI/dwc3 driver bound to this port (see
 * docs/j414s-atcphy.md sec 8 and sec 10 "L2"). This file only provides the
 * sequence, tested for content-fidelity against atc.c; src/atcphy.c's glue
 * does NOT call it from any automatic/boot-time path. ***
 */
const atcphy_seq_op_t *atcphy_seq_pipehandler_usb3_host_bist(size_t *count);

/* lpdptx AUX channel enable/disable. atcphy_enable_dp_aux atc.c:1215-1265,
 * atcphy_disable_dp_aux atc.c:1267-1281. DP-mode only. */
const atcphy_seq_op_t *atcphy_seq_dp_aux_enable(size_t *count);
const atcphy_seq_op_t *atcphy_seq_dp_aux_disable(size_t *count);

/* ---- Parameterized sequences: built into a caller-supplied buffer ---- */

#define ATCPHY_LANE_CONFIG_MAX_OPS 16u
#define ATCPHY_DP_RATE_MAX_OPS     40u

/*
 * LANE_MODE (RX0/TX0/RX1/TX1 fields) + CROSSBAR (protocol, DP single/both
 * PMA) + MISC.LANE_SWAP + per-lane RX PMAFSM, for one (mode, orientation)
 * pair. atcphy_configure_lanes, atc.c:1163-1213. Returns the op count
 * (<= ATCPHY_LANE_CONFIG_MAX_OPS), or 0 for an invalid mode.
 */
size_t atcphy_build_lane_config_ops(atcphy_mode_t mode, bool swapped, atcphy_seq_op_t *out,
                                    size_t out_max);

/*
 * AUSPLL bring-up + lock for one DP link rate: FREQ_DESC A/B/C from the
 * rate table, CLKOUT_DIV/DTC_VREG/BGR/CLKOUT_MASTER, the APB command
 * handshake (twice: command 0 then 0x2800), and the AUSPLL_LOCK poll.
 * atcphy_dp_configure, atc.c:1518-1614, MINUS the per-lane analog
 * calibration call (atcphy_dp_configure_lane, atc.c:1283-1493) which this
 * file does NOT model -- see docs/j414s-atcphy.md sec 9 for why. Returns
 * the op count (<= ATCPHY_DP_RATE_MAX_OPS), or 0 for an invalid rate.
 */
size_t atcphy_build_dp_rate_ops(atcphy_dp_rate_t rate, atcphy_seq_op_t *out, size_t out_max);

#endif /* ATCPHY_CORE_H */
