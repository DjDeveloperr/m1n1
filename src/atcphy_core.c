/* SPDX-License-Identifier: MIT */

#ifdef ATCPHY_CORE_HOST_TEST
#include <string.h>
#else
#include "string.h"
#endif

#include "atcphy_core.h"

#ifndef ARRAY_SIZE_LOCAL
#define ARRAY_SIZE_LOCAL(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

/* ------------------------------------------------------------------ */
/* Mode x orientation table (atc.c:634-786)                            */
/* ------------------------------------------------------------------ */

typedef struct {
    atcphy_mode_config_t normal;
    atcphy_mode_config_t swapped;
} atcphy_mode_pair_t;

static const atcphy_mode_pair_t atcphy_modes[ATCPHY_MODE_COUNT] = {
    [ATCPHY_MODE_OFF] =
        {
            .normal =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_OFF, ATCPHY_LANE_MODE_OFF},
                    .dp_lane = {false, false},
                    .set_swap = false,
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
            .swapped =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_OFF, ATCPHY_LANE_MODE_OFF},
                    .dp_lane = {false, false},
                    .set_swap = false, /* doesn't matter, SS lanes are off */
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
        },
    [ATCPHY_MODE_USB2] =
        {
            .normal =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_OFF, ATCPHY_LANE_MODE_OFF},
                    .dp_lane = {false, false},
                    .set_swap = false,
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
            .swapped =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_OFF, ATCPHY_LANE_MODE_OFF},
                    .dp_lane = {false, false},
                    .set_swap = false,
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
        },
    [ATCPHY_MODE_USB3] =
        {
            /* USB3/USB3 does not work (20Gbps unsupported); the "other" lane
             * is programmed as DP. atc.c:681-684 comment. */
            .normal =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_USB3, ATCPHY_LANE_MODE_DP},
                    .dp_lane = {false, true},
                    .set_swap = false,
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_USB3,
                },
            .swapped =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_DP, ATCPHY_LANE_MODE_USB3},
                    .dp_lane = {true, false},
                    .set_swap = true,
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_USB3,
                },
        },
    [ATCPHY_MODE_USB3_DP] =
        {
            .normal =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_USB3, ATCPHY_LANE_MODE_DP},
                    .dp_lane = {false, true},
                    .set_swap = false,
                    .enable_dp_aux = true,
                    .pipe_state = ATCPHY_PIPE_STATE_USB3,
                },
            .swapped =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_DP, ATCPHY_LANE_MODE_USB3},
                    .dp_lane = {true, false},
                    .set_swap = true,
                    .enable_dp_aux = true,
                    .pipe_state = ATCPHY_PIPE_STATE_USB3,
                },
        },
    [ATCPHY_MODE_TBT] =
        {
            .normal =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB4,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_USB4, ATCPHY_LANE_MODE_USB4},
                    .dp_lane = {false, false},
                    .set_swap = false,
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
            .swapped =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_USB4, ATCPHY_LANE_MODE_USB4},
                    .dp_lane = {false, false},
                    .set_swap = false, /* intentionally false, atc.c:740 */
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
        },
    [ATCPHY_MODE_USB4] =
        {
            .normal =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB4,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_USB4, ATCPHY_LANE_MODE_USB4},
                    .dp_lane = {false, false},
                    .set_swap = false,
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_USB4,
                },
            .swapped =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
                    .crossbar_dp_both_pma = false,
                    .lane_mode = {ATCPHY_LANE_MODE_USB4, ATCPHY_LANE_MODE_USB4},
                    .dp_lane = {false, false},
                    .set_swap = false, /* intentionally false, atc.c:760 */
                    .enable_dp_aux = false,
                    .pipe_state = ATCPHY_PIPE_STATE_USB4,
                },
        },
    [ATCPHY_MODE_DP] =
        {
            .normal =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_DP,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100,
                    .crossbar_dp_both_pma = true,
                    .lane_mode = {ATCPHY_LANE_MODE_DP, ATCPHY_LANE_MODE_DP},
                    .dp_lane = {true, true},
                    .set_swap = false,
                    .enable_dp_aux = true,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
            .swapped =
                {
                    .crossbar_protocol = ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_DP,
                    .crossbar_dp_single_pma = ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    .crossbar_dp_both_pma = false, /* intentionally false, atc.c:777 */
                    .lane_mode = {ATCPHY_LANE_MODE_DP, ATCPHY_LANE_MODE_DP},
                    .dp_lane = {true, true},
                    .set_swap = false, /* intentionally false, atc.c:780 */
                    .enable_dp_aux = true,
                    .pipe_state = ATCPHY_PIPE_STATE_DUMMY,
                },
        },
};

const atcphy_mode_config_t *atcphy_mode_config(atcphy_mode_t mode, bool swapped)
{
    if ((u32)mode >= ATCPHY_MODE_COUNT)
        return NULL;
    return swapped ? &atcphy_modes[mode].swapped : &atcphy_modes[mode].normal;
}

atcphy_pipe_backend_t atcphy_pipe_mux_decode(u32 mux_ctrl)
{
    u32 clk = (mux_ctrl & ATCPHY_PIPEHANDLER_MUX_CLK_MASK) >> ATCPHY_PIPEHANDLER_MUX_CLK_SHIFT;
    u32 data = (mux_ctrl & ATCPHY_PIPEHANDLER_MUX_DATA_MASK) >> ATCPHY_PIPEHANDLER_MUX_DATA_SHIFT;

    if (clk == ATCPHY_PIPEHANDLER_MUX_CLK_OFF)
        return ATCPHY_PIPE_BACKEND_CLK_OFF;

    if (clk == ATCPHY_PIPEHANDLER_MUX_CLK_USB3 && data == ATCPHY_PIPEHANDLER_MUX_DATA_USB3)
        return ATCPHY_PIPE_BACKEND_USB3;
    if (clk == ATCPHY_PIPEHANDLER_MUX_CLK_USB4 && data == ATCPHY_PIPEHANDLER_MUX_DATA_USB4)
        return ATCPHY_PIPE_BACKEND_USB4;
    if (clk == ATCPHY_PIPEHANDLER_MUX_CLK_DUMMY && data == ATCPHY_PIPEHANDLER_MUX_DATA_DUMMY)
        return ATCPHY_PIPE_BACKEND_DUMMY;

    return ATCPHY_PIPE_BACKEND_INCONSISTENT;
}

/* ------------------------------------------------------------------ */
/* DisplayPort link rates (atc.c:787-831, atc.c:1908-1921)             */
/* ------------------------------------------------------------------ */

static const atcphy_dp_lr_config_t atcphy_dp_lr_configs[ATCPHY_DP_RATE_COUNT] = {
    [ATCPHY_DP_RATE_RBR] =
        {
            .freqinit_count_target = 0x21c,
            .fbdivn_frac_den = 0x0,
            .fbdivn_frac_num = 0x0,
            .pclk_div_sel = 0x13,
            .lfclk_ctrl = 0x5,
            .vclk_op_divn = 0x2,
            .plla_clkout_vreg_bypass = true,
            .txa_ldoclk_bypass = true,
            .txa_div2_en = true,
        },
    [ATCPHY_DP_RATE_HBR] =
        {
            .freqinit_count_target = 0x1c2,
            .fbdivn_frac_den = 0x3ffe,
            .fbdivn_frac_num = 0x1fff,
            .pclk_div_sel = 0x9,
            .lfclk_ctrl = 0x5,
            .vclk_op_divn = 0x2,
            .plla_clkout_vreg_bypass = true,
            .txa_ldoclk_bypass = true,
            .txa_div2_en = false,
        },
    [ATCPHY_DP_RATE_HBR2] =
        {
            .freqinit_count_target = 0x1c2,
            .fbdivn_frac_den = 0x3ffe,
            .fbdivn_frac_num = 0x1fff,
            .pclk_div_sel = 0x4,
            .lfclk_ctrl = 0x5,
            .vclk_op_divn = 0x0,
            .plla_clkout_vreg_bypass = true,
            .txa_ldoclk_bypass = true,
            .txa_div2_en = false,
        },
    [ATCPHY_DP_RATE_HBR3] =
        {
            .freqinit_count_target = 0x2a3,
            .fbdivn_frac_den = 0x3ffc,
            .fbdivn_frac_num = 0x2ffd,
            .pclk_div_sel = 0x4,
            .lfclk_ctrl = 0x6,
            .vclk_op_divn = 0x0,
            .plla_clkout_vreg_bypass = false,
            .txa_ldoclk_bypass = false,
            .txa_div2_en = false,
        },
};

int atcphy_dp_rate_from_wire(u32 wire_code)
{
    switch (wire_code) {
        case ATCPHY_DP_WIRE_RBR:
            return ATCPHY_DP_RATE_RBR;
        case ATCPHY_DP_WIRE_HBR:
            return ATCPHY_DP_RATE_HBR;
        case ATCPHY_DP_WIRE_HBR2:
            return ATCPHY_DP_RATE_HBR2;
        case ATCPHY_DP_WIRE_HBR3:
            return ATCPHY_DP_RATE_HBR3;
        default:
            return -1;
    }
}

u32 atcphy_dp_rate_to_wire(atcphy_dp_rate_t rate)
{
    switch (rate) {
        case ATCPHY_DP_RATE_RBR:
            return ATCPHY_DP_WIRE_RBR;
        case ATCPHY_DP_RATE_HBR:
            return ATCPHY_DP_WIRE_HBR;
        case ATCPHY_DP_RATE_HBR2:
            return ATCPHY_DP_WIRE_HBR2;
        case ATCPHY_DP_RATE_HBR3:
            return ATCPHY_DP_WIRE_HBR3;
        default:
            return 0;
    }
}

u32 atcphy_dp_rate_khz(atcphy_dp_rate_t rate)
{
    switch (rate) {
        case ATCPHY_DP_RATE_RBR:
            return 1620;
        case ATCPHY_DP_RATE_HBR:
            return 2700;
        case ATCPHY_DP_RATE_HBR2:
            return 5400;
        case ATCPHY_DP_RATE_HBR3:
            return 8100;
        default:
            return 0;
    }
}

const atcphy_dp_lr_config_t *atcphy_dp_lr_config(atcphy_dp_rate_t rate)
{
    if ((u32)rate >= ATCPHY_DP_RATE_COUNT)
        return NULL;
    return &atcphy_dp_lr_configs[rate];
}

/* ------------------------------------------------------------------ */
/* Tunable vocabulary (kboot_atc.c:69-100), ADT-address-confirmed      */
/* ------------------------------------------------------------------ */

static const atcphy_tunable_vocab_t atcphy_tunable_table[ATCPHY_T6020_TUNABLE_COUNT] = {
    /* global, applied after power on/reset. block_offset/size match
     * kboot_atc.c:71-80 exactly; each was independently matched to a
     * distinct reg[] entry in the live ADT capture -- ADT:regN noted. */
    {"tunable_ATC0AXI2AF", ATCPHY_TUNABLE_TARGET_AXI2AF, 0x0, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg24 */
    {"tunable_ATC_FABRIC", ATCPHY_TUNABLE_TARGET_CORE, 0x45000, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg25 */
    {"tunable_USB_ACIOPHY_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x0, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg3 */
    {"tunable_AUS_CMN_SHM", ATCPHY_TUNABLE_TARGET_CORE, 0xa00, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg9 */
    {"tunable_AUS_CMN_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x800, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg10 */
    {"tunable_AUSPLL_CORE", ATCPHY_TUNABLE_TARGET_CORE, 0x2200, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg5 */
    {"tunable_AUSPLL_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x2000, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg4 */
    {"tunable_CIO3PLL_CORE", ATCPHY_TUNABLE_TARGET_CORE, 0x2a00, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg7 */
    {"tunable_CIO3PLL_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x2800, 0x4000, true,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg6 */
    {"tunable_CIO_CIO3PLL_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x2800, 0x4000, false,
     ATCPHY_TUNABLE_SCOPE_GLOBAL, 0}, /* ADT:reg6, optional dup */

    /* lane-specific, applied after a cable is connected */
    {"tunable_DP_LN0_AUSPMA_TX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0xc000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_DP, 0}, /* ADT:reg13 */
    {"tunable_DP_LN1_AUSPMA_TX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x13000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_DP, 1}, /* ADT:reg14 */
    {"tunable_USB_LN0_AUSPMA_TX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0xc000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 0}, /* ADT:reg13 */
    {"tunable_USB_LN0_AUSPMA_RX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x9000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 0}, /* ADT:reg17 */
    {"tunable_USB_LN0_AUSPMA_RX_SHM", ATCPHY_TUNABLE_TARGET_CORE, 0xb000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 0}, /* ADT:reg15 */
    {"tunable_USB_LN0_AUSPMA_RX_EQ", ATCPHY_TUNABLE_TARGET_CORE, 0xa000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 0}, /* ADT:reg26 */
    {"tunable_USB_LN1_AUSPMA_TX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x13000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 1}, /* ADT:reg14 */
    {"tunable_USB_LN1_AUSPMA_RX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x10000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 1}, /* ADT:reg18 */
    {"tunable_USB_LN1_AUSPMA_RX_SHM", ATCPHY_TUNABLE_TARGET_CORE, 0x12000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 1}, /* ADT:reg16 */
    {"tunable_USB_LN1_AUSPMA_RX_EQ", ATCPHY_TUNABLE_TARGET_CORE, 0x11000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_USB3, 1}, /* ADT:reg27 */
    {"tunable_CIO_LN0_AUSPMA_TX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0xc000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 0}, /* ADT:reg13 */
    {"tunable_CIO_LN0_AUSPMA_RX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x9000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 0}, /* ADT:reg17 */
    {"tunable_CIO_LN0_AUSPMA_RX_SHM", ATCPHY_TUNABLE_TARGET_CORE, 0xb000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 0}, /* ADT:reg15 */
    {"tunable_CIO_LN0_AUSPMA_RX_EQ", ATCPHY_TUNABLE_TARGET_CORE, 0xa000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 0}, /* ADT:reg26 */
    {"tunable_CIO_LN1_AUSPMA_TX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x13000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 1}, /* ADT:reg14 */
    {"tunable_CIO_LN1_AUSPMA_RX_TOP", ATCPHY_TUNABLE_TARGET_CORE, 0x10000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 1}, /* ADT:reg18 */
    {"tunable_CIO_LN1_AUSPMA_RX_SHM", ATCPHY_TUNABLE_TARGET_CORE, 0x12000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 1}, /* ADT:reg16 */
    {"tunable_CIO_LN1_AUSPMA_RX_EQ", ATCPHY_TUNABLE_TARGET_CORE, 0x11000, 0x1000, true,
     ATCPHY_TUNABLE_SCOPE_LANE_CIO, 1}, /* ADT:reg27 */
};

const atcphy_tunable_vocab_t *atcphy_t6020_tunable_vocab(size_t *count)
{
    if (count)
        *count = ATCPHY_T6020_TUNABLE_COUNT;
    return atcphy_tunable_table;
}

static bool str_eq(const char *a, const char *b)
{
    if (!a || !b)
        return false;
    while (*a && *b) {
        if (*a != *b)
            return false;
        a++;
        b++;
    }
    return *a == *b;
}

const atcphy_tunable_vocab_t *atcphy_t6020_tunable_lookup(const char *adt_name)
{
    for (size_t i = 0; i < ATCPHY_T6020_TUNABLE_COUNT; i++) {
        if (str_eq(atcphy_tunable_table[i].adt_name, adt_name))
            return &atcphy_tunable_table[i];
    }
    return NULL;
}

atcphy_tunable_blob_verdict_t atcphy_tunable_validate(const atcphy_tunable_vocab_t *vocab,
                                                       const u8 *blob, size_t blob_len,
                                                       atcphy_tunable_record_t *out,
                                                       size_t out_max, size_t *out_count,
                                                       size_t *bad_record_index)
{
    if (out_count)
        *out_count = 0;

    if (!vocab || (!blob && blob_len))
        return ATCPHY_TUNABLE_BLOB_BAD_RECORD;

    if (blob_len == 0)
        return ATCPHY_TUNABLE_BLOB_EMPTY;

    if (blob_len % 12 != 0)
        return ATCPHY_TUNABLE_BLOB_BAD_LENGTH;

    size_t n = blob_len / 12;
    size_t emitted = 0;

    for (size_t i = 0; i < n; i++) {
        const u8 *rec = blob + i * 12;
        /* wire layout: u32 offset:24 | size:8 (LE), u32 mask (LE), u32 value (LE) */
        u32 word0 = (u32)rec[0] | ((u32)rec[1] << 8) | ((u32)rec[2] << 16) | ((u32)rec[3] << 24);
        u32 offset = word0 & 0xFFFFFFu;
        u32 size = (word0 >> 24) & 0xFFu;
        u32 mask = (u32)rec[4] | ((u32)rec[5] << 8) | ((u32)rec[6] << 16) | ((u32)rec[7] << 24);
        u32 value = (u32)rec[8] | ((u32)rec[9] << 8) | ((u32)rec[10] << 16) | ((u32)rec[11] << 24);

        if (size != 32) {
            if (bad_record_index)
                *bad_record_index = i;
            return ATCPHY_TUNABLE_BLOB_BAD_RECORD;
        }
        if (offset % 4 != 0) {
            if (bad_record_index)
                *bad_record_index = i;
            return ATCPHY_TUNABLE_BLOB_BAD_RECORD;
        }
        if (offset + 4 > vocab->block_size) {
            if (bad_record_index)
                *bad_record_index = i;
            return ATCPHY_TUNABLE_BLOB_BAD_RECORD;
        }

        if (out && emitted < out_max) {
            out[emitted].offset = vocab->block_offset + offset;
            out[emitted].size = 32;
            out[emitted].mask = mask;
            out[emitted].value = value;
            emitted++;
        }
    }

    if (out_count)
        *out_count = emitted;
    return ATCPHY_TUNABLE_BLOB_OK;
}

u32 atcphy_tunable_apply_one(u32 old_value, u32 mask, u32 value)
{
    return (old_value & ~mask) | value;
}

atcphy_fuse_policy_t atcphy_fuse_policy(const char *adt_compatible)
{
    if (!adt_compatible)
        return ATCPHY_FUSES_UNKNOWN;
    if (str_eq(adt_compatible, "atc-phy,t8103"))
        return ATCPHY_FUSES_REQUIRED;
    if (str_eq(adt_compatible, "atc-phy,t6020"))
        return ATCPHY_FUSES_EMPTY_INTENTIONAL;
    return ATCPHY_FUSES_UNKNOWN;
}

/* ------------------------------------------------------------------ */
/* Op-list interpreter                                                 */
/* ------------------------------------------------------------------ */

int atcphy_seq_apply(const atcphy_seq_op_t *ops, size_t count, atcphy_seq_read_fn read_fn,
                     atcphy_seq_write_fn write_fn, atcphy_seq_delay_fn delay_fn, void *ctx,
                     size_t *fail_index)
{
    if (!ops || !read_fn || !write_fn)
        return -1;

    for (size_t i = 0; i < count; i++) {
        const atcphy_seq_op_t *op = &ops[i];
        u32 reg;

        switch (op->kind) {
            case ATCPHY_OP_WRITE:
                write_fn(ctx, op->block, op->offset, op->arg1);
                break;
            case ATCPHY_OP_SET:
                reg = read_fn(ctx, op->block, op->offset);
                write_fn(ctx, op->block, op->offset, reg | op->arg1);
                break;
            case ATCPHY_OP_CLEAR:
                reg = read_fn(ctx, op->block, op->offset);
                write_fn(ctx, op->block, op->offset, reg & ~op->arg1);
                break;
            case ATCPHY_OP_MASK:
                reg = read_fn(ctx, op->block, op->offset);
                write_fn(ctx, op->block, op->offset, (reg & ~op->arg1) | op->arg2);
                break;
            case ATCPHY_OP_DELAY_US:
                if (delay_fn)
                    delay_fn(ctx, op->arg1);
                break;
            case ATCPHY_OP_POLL_SET: {
                bool ok = false;
                for (u32 iter = 0; iter < op->arg2; iter++) {
                    if ((read_fn(ctx, op->block, op->offset) & op->arg1) == op->arg1) {
                        ok = true;
                        break;
                    }
                    if (delay_fn)
                        delay_fn(ctx, 1);
                }
                if (!ok) {
                    if (fail_index)
                        *fail_index = i;
                    return -1;
                }
                break;
            }
            case ATCPHY_OP_POLL_CLEAR: {
                bool ok = false;
                for (u32 iter = 0; iter < op->arg2; iter++) {
                    if ((read_fn(ctx, op->block, op->offset) & op->arg1) == 0) {
                        ok = true;
                        break;
                    }
                    if (delay_fn)
                        delay_fn(ctx, 1);
                }
                if (!ok) {
                    if (fail_index)
                        *fail_index = i;
                    return -1;
                }
                break;
            }
            default:
                if (fail_index)
                    *fail_index = i;
                return -1;
        }
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Fixed sequences                                                      */
/* ------------------------------------------------------------------ */

/* atcphy_usb2_power_on, atc.c:1669-1692 */
static const atcphy_seq_op_t atcphy_seq_usb2_power_on_ops[] = {
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_SET, ATCPHY_USB2PHY_SIG, ATCPHY_USB2PHY_SIG_VBUS_MASK, 0,
     "atc.c:1671"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1674"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_CLEAR, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_SIDDQ, 0,
     "atc.c:1677"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1678"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_CLEAR, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_RESET, 0,
     "atc.c:1681"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1682"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_CLEAR, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_PORT_RESET, 0,
     "atc.c:1683"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1684"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_SET, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_APB_RESET_N, 0,
     "atc.c:1685"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1686"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_CLEAR, ATCPHY_USB2PHY_MISCTUNE,
     ATCPHY_USB2PHY_MISCTUNE_APB_GATE_OFF, 0, "atc.c:1687"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_CLEAR, ATCPHY_USB2PHY_MISCTUNE,
     ATCPHY_USB2PHY_MISCTUNE_REF_GATE_OFF, 0, "atc.c:1688"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_WRITE, ATCPHY_USB2PHY_USBCTL, ATCPHY_USB2PHY_USBCTL_RUN, 0,
     "atc.c:1691"},
};

const atcphy_seq_op_t *atcphy_seq_usb2_power_on(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_usb2_power_on_ops);
    return atcphy_seq_usb2_power_on_ops;
}

/* atcphy_usb2_power_off, atc.c:1616-1635 */
static const atcphy_seq_op_t atcphy_seq_usb2_power_off_ops[] = {
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_WRITE, ATCPHY_USB2PHY_USBCTL,
     ATCPHY_USB2PHY_USBCTL_ISOLATION, 0, "atc.c:1619"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1620"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_SET, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_SIDDQ, 0,
     "atc.c:1623"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1624"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_SET, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_PORT_RESET, 0,
     "atc.c:1627"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1628"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_SET, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_RESET, 0,
     "atc.c:1629"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1630"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_CLEAR, ATCPHY_USB2PHY_CTL, ATCPHY_USB2PHY_CTL_APB_RESET_N, 0,
     "atc.c:1631"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1632"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_SET, ATCPHY_USB2PHY_MISCTUNE,
     ATCPHY_USB2PHY_MISCTUNE_APB_GATE_OFF, 0, "atc.c:1633"},
    {ATCPHY_BLOCK_USB2PHY, ATCPHY_OP_SET, ATCPHY_USB2PHY_MISCTUNE,
     ATCPHY_USB2PHY_MISCTUNE_REF_GATE_OFF, 0, "atc.c:1634"},
};

const atcphy_seq_op_t *atcphy_seq_usb2_power_off(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_usb2_power_off_ops);
    return atcphy_seq_usb2_power_off_ops;
}

/* atcphy_power_on, atc.c:1694-1723 (usb2_power_on inlined above by the
 * caller in src/atcphy.c; this table covers the core-domain part only). */
static const atcphy_seq_op_t atcphy_seq_core_power_on_ops[] = {
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ATCPHY_MISC, ATCPHY_CORE_MISC_RESET_N, 0,
     "atc.c:1701"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ATCPHY_POWER_CTRL, ATCPHY_CORE_POWER_SLEEP_SMALL,
     0, "atc.c:1703"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_ATCPHY_POWER_STAT,
     ATCPHY_CORE_POWER_SLEEP_SMALL, 100000, "atc.c:1704-1705"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ATCPHY_POWER_CTRL, ATCPHY_CORE_POWER_SLEEP_BIG,
     0, "atc.c:1711"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_ATCPHY_POWER_STAT,
     ATCPHY_CORE_POWER_SLEEP_BIG, 100000, "atc.c:1712-1713"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ATCPHY_POWER_CTRL, ATCPHY_CORE_POWER_CLAMP_EN,
     0, "atc.c:1719"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ATCPHY_POWER_CTRL, ATCPHY_CORE_POWER_APB_RESET_N,
     0, "atc.c:1720"},
};

const atcphy_seq_op_t *atcphy_seq_core_power_on(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_core_power_on_ops);
    return atcphy_seq_core_power_on_ops;
}

/* atcphy_power_off, atc.c:1637-1667 (dp_aux disable is a separate,
 * conditional prefix -- see atcphy_seq_dp_aux_disable) */
static const atcphy_seq_op_t atcphy_seq_core_power_off_ops[] = {
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ATCPHY_POWER_CTRL,
     ATCPHY_CORE_POWER_PHY_RESET_N, 0, "atc.c:1645"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ATCPHY_POWER_CTRL, ATCPHY_CORE_POWER_CLAMP_EN, 0,
     "atc.c:1646"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ATCPHY_MISC,
     ATCPHY_CORE_MISC_RESET_N | ATCPHY_CORE_MISC_LANE_SWAP, 0, "atc.c:1647"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ATCPHY_POWER_CTRL,
     ATCPHY_CORE_POWER_APB_RESET_N, 0, "atc.c:1648"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ATCPHY_POWER_CTRL, ATCPHY_CORE_POWER_SLEEP_BIG,
     0, "atc.c:1650"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_CLEAR, ATCPHY_CORE_ATCPHY_POWER_STAT,
     ATCPHY_CORE_POWER_SLEEP_BIG, 1000, "atc.c:1651-1652"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ATCPHY_POWER_CTRL,
     ATCPHY_CORE_POWER_SLEEP_SMALL, 0, "atc.c:1658"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_CLEAR, ATCPHY_CORE_ATCPHY_POWER_STAT,
     ATCPHY_CORE_POWER_SLEEP_SMALL, 1000, "atc.c:1659-1660"},
};

const atcphy_seq_op_t *atcphy_seq_core_power_off(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_core_power_off_ops);
    return atcphy_seq_core_power_off_ops;
}

/* atcphy_configure's CFG0/SLEEP_CTRL override dance, atc.c:1746-1771 */
static const atcphy_seq_op_t atcphy_seq_cfg0_sleep_override_ops[] = {
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_CFG0,
     ATCPHY_CORE_ACIOPHY_CFG0_COMMON_SMALL_OV, 0, "atc.c:1746"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1747"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_CFG0,
     ATCPHY_CORE_ACIOPHY_CFG0_COMMON_BIG_OV, 0, "atc.c:1748"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1749"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_CFG0,
     ATCPHY_CORE_ACIOPHY_CFG0_COMMON_CLAMP_OV, 0, "atc.c:1750"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1751"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_SLEEP_CTRL,
     ATCPHY_CORE_SLEEP_CTRL_TX_SMALL_OV_MASK, 3u << ATCPHY_CORE_SLEEP_CTRL_TX_SMALL_OV_SHIFT,
     "atc.c:1753-1754"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1755"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_SLEEP_CTRL,
     ATCPHY_CORE_SLEEP_CTRL_TX_BIG_OV_MASK, 3u << ATCPHY_CORE_SLEEP_CTRL_TX_BIG_OV_SHIFT,
     "atc.c:1756-1757"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1758"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_SLEEP_CTRL,
     ATCPHY_CORE_SLEEP_CTRL_TX_CLAMP_OV_MASK, 3u << ATCPHY_CORE_SLEEP_CTRL_TX_CLAMP_OV_SHIFT,
     "atc.c:1759-1760"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1761"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_CFG0,
     ATCPHY_CORE_ACIOPHY_CFG0_RX_BIG_OV_MASK, 3u << ATCPHY_CORE_ACIOPHY_CFG0_RX_BIG_OV_SHIFT,
     "atc.c:1763-1764"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1765"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_CFG0,
     ATCPHY_CORE_ACIOPHY_CFG0_RX_SMALL_OV_MASK, 3u << ATCPHY_CORE_ACIOPHY_CFG0_RX_SMALL_OV_SHIFT,
     "atc.c:1766-1767"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1768"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_CFG0,
     ATCPHY_CORE_ACIOPHY_CFG0_RX_CLAMP_OV_MASK, 3u << ATCPHY_CORE_ACIOPHY_CFG0_RX_CLAMP_OV_SHIFT,
     "atc.c:1769-1770"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1771"},
};

const atcphy_seq_op_t *atcphy_seq_cfg0_sleep_override(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_cfg0_sleep_override_ops);
    return atcphy_seq_cfg0_sleep_override_ops;
}

/* atc.c:1778-1779 */
static const atcphy_seq_op_t atcphy_seq_cio3pll_enable_ops[] = {
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_CIO3PLL_CLK_CTRL,
     ATCPHY_CORE_CIO3PLL_CLK_PCLK_EN, 0, "atc.c:1778"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_CIO3PLL_CLK_CTRL,
     ATCPHY_CORE_CIO3PLL_CLK_REFCLK_EN, 0, "atc.c:1779"},
};

const atcphy_seq_op_t *atcphy_seq_cio3pll_enable(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_cio3pll_enable_ops);
    return atcphy_seq_cio3pll_enable_ops;
}

/* atc.c:1743-1744 */
static const atcphy_seq_op_t atcphy_seq_auspll_fsm_override_ops[] = {
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_PLL_PCTL_FSM_CTRL1, 0x1fe000u, 0, "atc.c:1743"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
     ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_UNK28, 0, "atc.c:1744"},
};

const atcphy_seq_op_t *atcphy_seq_auspll_fsm_override(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_auspll_fsm_override_ops);
    return atcphy_seq_auspll_fsm_override_ops;
}

/* atc.c:1783 */
static const atcphy_seq_op_t atcphy_seq_release_phy_reset_ops[] = {
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ATCPHY_POWER_CTRL,
     ATCPHY_CORE_POWER_PHY_RESET_N, 0, "atc.c:1783"},
};

const atcphy_seq_op_t *atcphy_seq_release_phy_reset(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_release_phy_reset_ops);
    return atcphy_seq_release_phy_reset_ops;
}

/* atcphy_configure_pipehandler_dummy, atc.c:1081-1120 */
static const atcphy_seq_op_t atcphy_seq_pipehandler_dummy_ops[] = {
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_CLEAR, ATCPHY_PIPEHANDLER_OVERRIDE_VALUES,
     ATCPHY_PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 | ATCPHY_PIPEHANDLER_OVERRIDE_VAL_RXDETECT1, 0,
     "atc.c:1090-1091"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_SET, ATCPHY_PIPEHANDLER_OVERRIDE,
     ATCPHY_PIPEHANDLER_OVERRIDE_RXVALID, 0, "atc.c:1092"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_SET, ATCPHY_PIPEHANDLER_OVERRIDE,
     ATCPHY_PIPEHANDLER_OVERRIDE_RXDETECT, 0, "atc.c:1093"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_SET, ATCPHY_PIPEHANDLER_LOCK_REQ,
     ATCPHY_PIPEHANDLER_LOCK_EN, 0, "atc.c:1095 (via atcphy_pipehandler_lock)"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_POLL_SET, ATCPHY_PIPEHANDLER_LOCK_ACK,
     ATCPHY_PIPEHANDLER_LOCK_EN, ATCPHY_PIPEHANDLER_LOCK_ACK_TIMEOUT_US,
     "atc.c:932-933 (via atcphy_pipehandler_lock)"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_MUX_CTRL,
     ATCPHY_PIPEHANDLER_MUX_CLK_MASK, ATCPHY_PIPEHANDLER_MUX_CLK_OFF
                                          << ATCPHY_PIPEHANDLER_MUX_CLK_SHIFT,
     "atc.c:1100-1101"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1102"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_MUX_CTRL,
     ATCPHY_PIPEHANDLER_MUX_DATA_MASK, ATCPHY_PIPEHANDLER_MUX_DATA_DUMMY
                                           << ATCPHY_PIPEHANDLER_MUX_DATA_SHIFT,
     "atc.c:1103-1104"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1105"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_MUX_CTRL,
     ATCPHY_PIPEHANDLER_MUX_CLK_MASK, ATCPHY_PIPEHANDLER_MUX_CLK_DUMMY
                                          << ATCPHY_PIPEHANDLER_MUX_CLK_SHIFT,
     "atc.c:1106-1107"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1108"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_CLEAR, ATCPHY_PIPEHANDLER_LOCK_REQ,
     ATCPHY_PIPEHANDLER_LOCK_EN, 0, "atc.c:1110 (via atcphy_pipehandler_unlock)"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_POLL_CLEAR, ATCPHY_PIPEHANDLER_LOCK_ACK,
     ATCPHY_PIPEHANDLER_LOCK_EN, ATCPHY_PIPEHANDLER_LOCK_ACK_TIMEOUT_US,
     "atc.c:948-949 (via atcphy_pipehandler_unlock)"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_NONSELECTED_OVERRIDE,
     ATCPHY_PIPEHANDLER_NATIVE_POWER_DOWN_MASK, 2u << ATCPHY_PIPEHANDLER_NATIVE_POWER_DOWN_SHIFT,
     "atc.c:1114-1115"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_SET, ATCPHY_PIPEHANDLER_NONSELECTED_OVERRIDE,
     ATCPHY_PIPEHANDLER_NATIVE_RESET, 0, "atc.c:1116-1117"},
};

const atcphy_seq_op_t *atcphy_seq_pipehandler_dummy(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_pipehandler_dummy_ops);
    return atcphy_seq_pipehandler_dummy_ops;
}

/*
 * atcphy_configure_pipehandler_usb3(atcphy, host=true), atc.c:975-1079.
 * SAFETY: see the header comment on atcphy_seq_pipehandler_usb3_host_bist.
 */
static const atcphy_seq_op_t atcphy_seq_pipehandler_usb3_host_bist_ops[] = {
    /* atcphy_pipehandler_check: if already locked, unlock first. Not
     * representable as an unconditional op list entry; the glue layer
     * performs this check before applying this sequence (see atcphy.c). */

    /* Force disable link detection, atc.c:989-995 */
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_CLEAR, ATCPHY_PIPEHANDLER_OVERRIDE_VALUES,
     ATCPHY_PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 | ATCPHY_PIPEHANDLER_OVERRIDE_VAL_RXDETECT1, 0,
     "atc.c:990-991"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_SET, ATCPHY_PIPEHANDLER_OVERRIDE,
     ATCPHY_PIPEHANDLER_OVERRIDE_RXVALID, 0, "atc.c:992-993"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_SET, ATCPHY_PIPEHANDLER_OVERRIDE,
     ATCPHY_PIPEHANDLER_OVERRIDE_RXDETECT, 0, "atc.c:994-995"},

    /* atcphy_pipehandler_lock, atc.c:997 */
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_SET, ATCPHY_PIPEHANDLER_LOCK_REQ,
     ATCPHY_PIPEHANDLER_LOCK_EN, 0, "atc.c:930"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_POLL_SET, ATCPHY_PIPEHANDLER_LOCK_ACK,
     ATCPHY_PIPEHANDLER_LOCK_EN, ATCPHY_PIPEHANDLER_LOCK_ACK_TIMEOUT_US, "atc.c:932-933"},

    /* BIST dance, atc.c:1004-1027 */
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_PHY_CFG0,
     ATCPHY_CORE_TOP_BIST_PHY_CFG0_LN0_RESET_N, 0, "atc.c:1004-1005"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_OV_CFG,
     ATCPHY_CORE_TOP_BIST_OV_CFG_LN0_RESET_N_OV, 0, "atc.c:1006"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_CLEAR, ATCPHY_CORE_TOP_PHY_STAT,
     ATCPHY_CORE_TOP_PHY_STAT_LN0_UNK23, 10000, "atc.c:1007-1008"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_READ_CTRL,
     ATCPHY_CORE_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE, 0, "atc.c:1013-1014"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_TOP_BIST_READ_CTRL,
     ATCPHY_CORE_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE, 0, "atc.c:1015-1016"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_TOP_BIST_PHY_CFG1,
     ATCPHY_CORE_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_MASK,
     3u << ATCPHY_CORE_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_SHIFT, "atc.c:1018-1020"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_OV_CFG,
     ATCPHY_CORE_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV, 0, "atc.c:1022-1023"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1,
     ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1_CLK_EN, 0, "atc.c:1024-1025"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1,
     ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1_BIST_EN, 0, "atc.c:1026-1027"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_WRITE, ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1, 0, 0, "atc.c:1028"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_TOP_PHY_STAT,
     ATCPHY_CORE_TOP_PHY_STAT_LN0_UNK0, 10000, "atc.c:1030-1031"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_CLEAR, ATCPHY_CORE_TOP_PHY_STAT,
     ATCPHY_CORE_TOP_PHY_STAT_LN0_UNK23, 10000, "atc.c:1036-1037"},

    /* Clear reset for non-selected USB3 PHY, atc.c:1043-1046 */
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_NONSELECTED_OVERRIDE,
     ATCPHY_PIPEHANDLER_NATIVE_POWER_DOWN_MASK, 3u << ATCPHY_PIPEHANDLER_NATIVE_POWER_DOWN_SHIFT,
     "atc.c:1043-1044"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_CLEAR, ATCPHY_PIPEHANDLER_NONSELECTED_OVERRIDE,
     ATCPHY_PIPEHANDLER_NATIVE_RESET, 0, "atc.c:1045-1046"},

    /* More BIST stuff, atc.c:1049-1053 */
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_WRITE, ATCPHY_CORE_TOP_BIST_OV_CFG, 0, 0, "atc.c:1049"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1,
     ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1_CLK_EN, 0, "atc.c:1050-1051"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1,
     ATCPHY_CORE_TOP_BIST_CIOPHY_CFG1_BIST_EN, 0, "atc.c:1052-1053"},

    /* Configure PIPE mux to USB3 PHY, atc.c:1056-1065 */
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_MUX_CTRL,
     ATCPHY_PIPEHANDLER_MUX_CLK_MASK, ATCPHY_PIPEHANDLER_MUX_CLK_OFF
                                          << ATCPHY_PIPEHANDLER_MUX_CLK_SHIFT,
     "atc.c:1057-1058"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1059"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_MUX_CTRL,
     ATCPHY_PIPEHANDLER_MUX_DATA_MASK, ATCPHY_PIPEHANDLER_MUX_DATA_USB3
                                           << ATCPHY_PIPEHANDLER_MUX_DATA_SHIFT,
     "atc.c:1060-1061"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1062"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_MASK, ATCPHY_PIPEHANDLER_MUX_CTRL,
     ATCPHY_PIPEHANDLER_MUX_CLK_MASK, ATCPHY_PIPEHANDLER_MUX_CLK_USB3
                                          << ATCPHY_PIPEHANDLER_MUX_CLK_SHIFT,
     "atc.c:1063-1064"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1065"},

    /* Remove link detection override, atc.c:1068-1069 */
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_CLEAR, ATCPHY_PIPEHANDLER_OVERRIDE,
     ATCPHY_PIPEHANDLER_OVERRIDE_RXVALID, 0, "atc.c:1068"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_CLEAR, ATCPHY_PIPEHANDLER_OVERRIDE,
     ATCPHY_PIPEHANDLER_OVERRIDE_RXDETECT, 0, "atc.c:1069"},

    /* atcphy_pipehandler_unlock (host mode only), atc.c:1073 */
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_CLEAR, ATCPHY_PIPEHANDLER_LOCK_REQ,
     ATCPHY_PIPEHANDLER_LOCK_EN, 0, "atc.c:947"},
    {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_POLL_CLEAR, ATCPHY_PIPEHANDLER_LOCK_ACK,
     ATCPHY_PIPEHANDLER_LOCK_EN, ATCPHY_PIPEHANDLER_LOCK_ACK_TIMEOUT_US, "atc.c:948-949"},
};

const atcphy_seq_op_t *atcphy_seq_pipehandler_usb3_host_bist(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_pipehandler_usb3_host_bist_ops);
    return atcphy_seq_pipehandler_usb3_host_bist_ops;
}

/* atcphy_enable_dp_aux, atc.c:1215-1265 (minus atcphy->dp_link_rate = -1,
 * which is state tracking outside this file's scope) */
static const atcphy_seq_op_t atcphy_seq_dp_aux_enable_ops[] = {
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTXPHY_PMA_LANE_RESET_N, 0, "atc.c:1217"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTXPHY_PMA_LANE_RESET_N_OV, 0, "atc.c:1218"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPRX_PCLK_SELECT_MASK, 1u << ATCPHY_CORE_DPRX_PCLK_SELECT_SHIFT,
     "atc.c:1220-1221"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPRX_PCLK_ENABLE, 0, "atc.c:1222"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTX_PCLK1_SELECT_MASK, 1u << ATCPHY_CORE_DPTX_PCLK1_SELECT_SHIFT,
     "atc.c:1224-1225"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTX_PCLK1_ENABLE, 0, "atc.c:1226"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTX_PCLK2_SELECT_MASK, 1u << ATCPHY_CORE_DPTX_PCLK2_SELECT_SHIFT,
     "atc.c:1228-1229"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTX_PCLK2_ENABLE, 0, "atc.c:1230"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_PLL_COMMON_CTRL,
     ATCPHY_CORE_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT, 0, "atc.c:1232-1233"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_AUX_CLAMP_EN, 0,
     "atc.c:1235"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_SLEEP_B_SML_IN, 0,
     "atc.c:1236"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1237"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_SLEEP_B_BIG_IN, 0,
     "atc.c:1238"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1239"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_CLEAR, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_AUX_CLAMP_EN, 0,
     "atc.c:1240"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_CLEAR, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_AUX_PWN_DOWN, 0,
     "atc.c:1241"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_CLEAR, ATCPHY_LPDPTX_AUX_CONTROL,
     ATCPHY_LPDPTX_TXTERM_CODEMSB, 0, "atc.c:1242"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_MASK, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_TXTERM_CODE_MASK,
     0x16u << ATCPHY_LPDPTX_TXTERM_CODE_SHIFT, "atc.c:1243-1244"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_LDO_CTRL, 0x1c00u, 0,
     "atc.c:1246"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_MASK, ATCPHY_LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1,
     ATCPHY_LPDPTX_CFG_PMA_PHYS_ADJ_MASK, 5u << ATCPHY_LPDPTX_CFG_PMA_PHYS_ADJ_SHIFT,
     "atc.c:1247-1248"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1,
     ATCPHY_LPDPTX_CFG_PMA_PHYS_ADJ_OV, 0, "atc.c:1249-1250"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_CLEAR, ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_MARGIN,
     ATCPHY_LPDPTX_MARGIN_RCAL_RXOFFSET_EN, 0, "atc.c:1252-1253"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_CLEAR, ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_CTRL,
     ATCPHY_LPDPTX_BLK_AUX_CTRL_PWRDN, 0, "atc.c:1255"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG0,
     ATCPHY_LPDPTX_CFG_PMA_AUX_SEL_LF_DATA, 0, "atc.c:1256-1257"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_MASK, ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_CTRL,
     ATCPHY_LPDPTX_BLK_AUX_RXOFFSET_MASK, 3u << ATCPHY_LPDPTX_BLK_AUX_RXOFFSET_SHIFT,
     "atc.c:1258-1259"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_MASK, ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_MARGIN,
     ATCPHY_LPDPTX_AUX_MARGIN_RCAL_TXSWING_MASK, 12u << ATCPHY_LPDPTX_AUX_MARGIN_RCAL_TXSWING_SHIFT,
     "atc.c:1261-1262"},
};

const atcphy_seq_op_t *atcphy_seq_dp_aux_enable(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_dp_aux_enable_ops);
    return atcphy_seq_dp_aux_enable_ops;
}

/* atcphy_disable_dp_aux, atc.c:1267-1281 */
static const atcphy_seq_op_t atcphy_seq_dp_aux_disable_ops[] = {
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_AUX_PWN_DOWN, 0,
     "atc.c:1269"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_CFG_BLK_AUX_CTRL,
     ATCPHY_LPDPTX_BLK_AUX_CTRL_PWRDN, 0, "atc.c:1270"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_SET, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_AUX_CLAMP_EN, 0,
     "atc.c:1271"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_CLEAR, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_SLEEP_B_SML_IN,
     0, "atc.c:1272"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1273"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_CLEAR, ATCPHY_LPDPTX_AUX_CONTROL, ATCPHY_LPDPTX_SLEEP_B_BIG_IN,
     0, "atc.c:1274"},
    {ATCPHY_BLOCK_LPDPTX, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1275"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTXPHY_PMA_LANE_RESET_N, 0, "atc.c:1277"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPRX_PCLK_ENABLE, 0, "atc.c:1278"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTX_PCLK1_ENABLE, 0, "atc.c:1279"},
    {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
     ATCPHY_CORE_DPTX_PCLK2_ENABLE, 0, "atc.c:1280"},
};

const atcphy_seq_op_t *atcphy_seq_dp_aux_disable(size_t *count)
{
    if (count)
        *count = ARRAY_SIZE_LOCAL(atcphy_seq_dp_aux_disable_ops);
    return atcphy_seq_dp_aux_disable_ops;
}

/* ------------------------------------------------------------------ */
/* Parameterized builders                                              */
/* ------------------------------------------------------------------ */

/* atcphy_configure_lanes, atc.c:1163-1213 */
size_t atcphy_build_lane_config_ops(atcphy_mode_t mode, bool swapped, atcphy_seq_op_t *out,
                                    size_t out_max)
{
    const atcphy_mode_config_t *cfg = atcphy_mode_config(mode, swapped);
    size_t n = 0;

    if (!cfg || !out)
        return 0;

#define EMIT(b, k, o, a1, a2, c)                                                                   \
    do {                                                                                           \
        if (n < out_max) {                                                                         \
            out[n].block = (b);                                                                    \
            out[n].kind = (k);                                                                     \
            out[n].offset = (o);                                                                   \
            out[n].arg1 = (u32)(a1);                                                               \
            out[n].arg2 = (u32)(a2);                                                               \
            out[n].cite = (c);                                                                     \
            n++;                                                                                   \
        }                                                                                          \
    } while (0)

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_LANE_MODE,
         ATCPHY_CORE_LANE_MODE_RX0_MASK, cfg->lane_mode[0] << ATCPHY_CORE_LANE_MODE_RX0_SHIFT,
         "atc.c:1167-1168");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_LANE_MODE,
         ATCPHY_CORE_LANE_MODE_TX0_MASK, cfg->lane_mode[0] << ATCPHY_CORE_LANE_MODE_TX0_SHIFT,
         "atc.c:1169-1170");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_LANE_MODE,
         ATCPHY_CORE_LANE_MODE_RX1_MASK, cfg->lane_mode[1] << ATCPHY_CORE_LANE_MODE_RX1_SHIFT,
         "atc.c:1171-1172");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_LANE_MODE,
         ATCPHY_CORE_LANE_MODE_TX1_MASK, cfg->lane_mode[1] << ATCPHY_CORE_LANE_MODE_TX1_SHIFT,
         "atc.c:1173-1174");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_CROSSBAR,
         ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_MASK, cfg->crossbar_protocol, "atc.c:1175-1176");

    if (cfg->set_swap)
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ATCPHY_MISC,
             ATCPHY_CORE_MISC_LANE_SWAP, 0, "atc.c:1179");
    else
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ATCPHY_MISC,
             ATCPHY_CORE_MISC_LANE_SWAP, 0, "atc.c:1181");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_ACIOPHY_CROSSBAR,
         ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_MASK,
         (u32)cfg->crossbar_dp_single_pma << ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_SHIFT,
         "atc.c:1183-1184");
    if (cfg->crossbar_dp_both_pma)
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_ACIOPHY_CROSSBAR,
             ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_BOTH_PMA, 0, "atc.c:1186");
    else
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ACIOPHY_CROSSBAR,
             ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_BOTH_PMA, 0, "atc.c:1188");

    if (cfg->dp_lane[0]) {
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET,
             ATCPHY_CORE_LN0_AUSPMA_RX_TOP + ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM,
             ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV, 0, "atc.c:1191-1192");
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1193");
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR,
             ATCPHY_CORE_LN0_AUSPMA_RX_TOP + ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM,
             ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ, 0, "atc.c:1194-1195");
    } else {
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR,
             ATCPHY_CORE_LN0_AUSPMA_RX_TOP + ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM,
             ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV, 0, "atc.c:1197-1198");
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1199");
    }

    if (cfg->dp_lane[1]) {
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET,
             ATCPHY_CORE_LN1_AUSPMA_RX_TOP + ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM,
             ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV, 0, "atc.c:1203-1204");
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1205");
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR,
             ATCPHY_CORE_LN1_AUSPMA_RX_TOP + ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM,
             ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ, 0, "atc.c:1206-1207");
    } else {
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR,
             ATCPHY_CORE_LN1_AUSPMA_RX_TOP + ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM,
             ATCPHY_CORE_LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV, 0, "atc.c:1209-1210");
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "atc.c:1211");
    }

#undef EMIT
    return n;
}

/*
 * atcphy_dp_configure, atc.c:1518-1614, MINUS the per-lane analog
 * calibration (atcphy_dp_configure_lane) -- see design doc sec 9.
 */
size_t atcphy_build_dp_rate_ops(atcphy_dp_rate_t rate, atcphy_seq_op_t *out, size_t out_max)
{
    const atcphy_dp_lr_config_t *cfg = atcphy_dp_lr_config(rate);
    size_t n = 0;

    if (!cfg || !out)
        return 0;

#define EMIT(b, k, o, a1, a2, c)                                                                   \
    do {                                                                                           \
        if (n < out_max) {                                                                         \
            out[n].block = (b);                                                                    \
            out[n].kind = (k);                                                                     \
            out[n].offset = (o);                                                                   \
            out[n].arg1 = (u32)(a1);                                                               \
            out[n].arg2 = (u32)(a2);                                                               \
            out[n].cite = (c);                                                                     \
            n++;                                                                                   \
        }                                                                                          \
    } while (0)

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_ACIOPHY_CMN_SHM_STS_REG0,
         ATCPHY_CORE_ACIOPHY_CMN_SHM_STS_REG0_CMD_READY, 10000, "atc.c:1532-1533");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_FREQ_CFG,
         ATCPHY_CORE_AUSPLL_FREQ_REFCLK_MASK, 0, "atc.c:1539");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_FREQ_COUNT_TARGET_MASK,
         (u32)cfg->freqinit_count_target << ATCPHY_CORE_AUSPLL_FD_FREQ_COUNT_TARGET_SHIFT,
         "atc.c:1541-1542");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_FBDIVN_HALF, 0, "atc.c:1543");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_REV_DIVN_MASK, 0, "atc.c:1544");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_KI_MAN_MASK, 8u << ATCPHY_CORE_AUSPLL_FD_KI_MAN_SHIFT,
         "atc.c:1545");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_KI_EXP_MASK, 3u << ATCPHY_CORE_AUSPLL_FD_KI_EXP_SHIFT,
         "atc.c:1546");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_KP_MAN_MASK, 8u << ATCPHY_CORE_AUSPLL_FD_KP_MAN_SHIFT,
         "atc.c:1547");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_KP_EXP_MASK, 7u << ATCPHY_CORE_AUSPLL_FD_KP_EXP_SHIFT,
         "atc.c:1548");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_FREQ_DESC_A,
         ATCPHY_CORE_AUSPLL_FD_KPKI_SCALE_HBW_MASK, 0, "atc.c:1549");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_B,
         ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_DEN_MASK,
         (u32)cfg->fbdivn_frac_den << ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_DEN_SHIFT,
         "atc.c:1551-1552");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_B,
         ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_NUM_MASK,
         (u32)cfg->fbdivn_frac_num << ATCPHY_CORE_AUSPLL_FD_FBDIVN_FRAC_NUM_SHIFT,
         "atc.c:1553-1554");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_FREQ_DESC_C,
         ATCPHY_CORE_AUSPLL_FD_SDM_SSC_STEP_MASK, 0, "atc.c:1556");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_FREQ_DESC_C,
         ATCPHY_CORE_AUSPLL_FD_SDM_SSC_EN, 0, "atc.c:1557");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_C,
         ATCPHY_CORE_AUSPLL_FD_PCLK_DIV_SEL_MASK,
         (u32)cfg->pclk_div_sel << ATCPHY_CORE_AUSPLL_FD_PCLK_DIV_SEL_SHIFT, "atc.c:1558-1559");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_C,
         ATCPHY_CORE_AUSPLL_FD_LFSDM_DIV_MASK, 1u << ATCPHY_CORE_AUSPLL_FD_LFSDM_DIV_SHIFT,
         "atc.c:1560-1561");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_C,
         ATCPHY_CORE_AUSPLL_FD_LFCLK_CTRL_MASK,
         (u32)cfg->lfclk_ctrl << ATCPHY_CORE_AUSPLL_FD_LFCLK_CTRL_SHIFT, "atc.c:1562-1563");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_FREQ_DESC_C,
         ATCPHY_CORE_AUSPLL_FD_VCLK_OP_DIVN_MASK,
         (u32)cfg->vclk_op_divn << ATCPHY_CORE_AUSPLL_FD_VCLK_OP_DIVN_SHIFT, "atc.c:1564-1565");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_AUSPLL_FREQ_DESC_C,
         ATCPHY_CORE_AUSPLL_FD_VCLK_PRE_DIVN, 0, "atc.c:1566");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_CLKOUT_DIV,
         ATCPHY_CORE_AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI_MASK,
         7u << ATCPHY_CORE_AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI_SHIFT, "atc.c:1568-1569");

    if (cfg->plla_clkout_vreg_bypass)
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_AUSPLL_CLKOUT_DTC_VREG,
             ATCPHY_CORE_AUSPLL_DTC_VREG_BYPASS, 0, "atc.c:1572");
    else
        EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_CLKOUT_DTC_VREG,
             ATCPHY_CORE_AUSPLL_DTC_VREG_BYPASS, 0, "atc.c:1574");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_AUSPLL_BGR, ATCPHY_CORE_AUSPLL_BGR_CTRL_AVAIL,
         0, "atc.c:1576");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_AUSPLL_CLKOUT_MASTER,
         ATCPHY_CORE_AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN, 0, "atc.c:1578");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_AUSPLL_CLKOUT_MASTER,
         ATCPHY_CORE_AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN, 0, "atc.c:1579");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, ATCPHY_CORE_AUSPLL_CLKOUT_MASTER,
         ATCPHY_CORE_AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN, 0, "atc.c:1580");

    /* atcphy_auspll_apb_command(atcphy, 0), atc.c:1496-1516,1582 */
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
         ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_CMD_MASK,
         (0u << ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_CMD_SHIFT) |
             ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_REQ | ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_UNK28,
         "atc.c:1501-1506 cmd=0");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
         ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_ACK, 10000, "atc.c:1508-1509");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
         ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_REQ, 0, "atc.c:1513");

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_ACIOPHY_DP_PCLK_STAT,
         ATCPHY_CORE_ACIOPHY_AUSPLL_LOCK, 10000, "atc.c:1586-1587");

    /* atcphy_auspll_apb_command(atcphy, 0x2800), atc.c:1593 */
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
         ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_CMD_MASK,
         (0x2800u << ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_CMD_SHIFT) |
             ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_REQ | ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_UNK28,
         "atc.c:1501-1506 cmd=0x2800");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
         ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_ACK, 10000, "atc.c:1508-1509");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
         ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_REQ, 0, "atc.c:1513");

    /*
     * NOT MODELED: atcphy_dp_configure_lane() per DP lane (atc.c:1283-1493),
     * a ~300-line, ~100-register-field blind analog TX/RX calibration
     * sequence with no independently-understood semantics beyond XNU debug
     * names. See docs/j414s-atcphy.md sec 9 for why this is scoped out.
     */

    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
         ATCPHY_CORE_DP_PMA_BYTECLK_RESET, 0, "atc.c:1609");
    EMIT(ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, ATCPHY_CORE_ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
         ATCPHY_CORE_DP_MAC_DIV20_CLK_SEL, 0, "atc.c:1610");

#undef EMIT
    return n;
}
