/* SPDX-License-Identifier: MIT */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "atcphy_core.h"

/* ------------------------------------------------------------------ */
/* Fake register file for the op-list interpreter                      */
/* ------------------------------------------------------------------ */

#define FAKE_REG_MAX 512

typedef struct {
    atcphy_block_t block;
    u32 offset;
    u32 value;
    bool used;
} fake_reg_t;

typedef struct {
    fake_reg_t regs[FAKE_REG_MAX];
    size_t count;
    u32 poll_success_after; /* POLL ops succeed once this many reads of the
                              * polled register have happened */
    u32 poll_reads;
    u32 total_delay_us;
} fake_ctx_t;

static fake_reg_t *fake_find(fake_ctx_t *ctx, atcphy_block_t block, u32 offset, bool create)
{
    for (size_t i = 0; i < ctx->count; i++) {
        if (ctx->regs[i].block == block && ctx->regs[i].offset == offset)
            return &ctx->regs[i];
    }
    if (!create)
        return NULL;
    assert(ctx->count < FAKE_REG_MAX);
    fake_reg_t *r = &ctx->regs[ctx->count++];
    r->block = block;
    r->offset = offset;
    r->value = 0;
    r->used = true;
    return r;
}

static u32 fake_read(void *ctx_, atcphy_block_t block, u32 offset)
{
    fake_ctx_t *ctx = ctx_;
    fake_reg_t *r = fake_find(ctx, block, offset, false);
    return r ? r->value : 0;
}

static void fake_write(void *ctx_, atcphy_block_t block, u32 offset, u32 value)
{
    fake_ctx_t *ctx = ctx_;
    fake_reg_t *r = fake_find(ctx, block, offset, true);
    r->value = value;
}

static void fake_delay(void *ctx_, u32 us)
{
    fake_ctx_t *ctx = ctx_;
    ctx->total_delay_us += us;
}

static void fake_reset(fake_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

/* ------------------------------------------------------------------ */
/* Mode table                                                          */
/* ------------------------------------------------------------------ */

static void test_mode_config_off_and_usb2_are_orientation_swap_only(void)
{
    const atcphy_mode_config_t *off_n = atcphy_mode_config(ATCPHY_MODE_OFF, false);
    const atcphy_mode_config_t *off_s = atcphy_mode_config(ATCPHY_MODE_OFF, true);
    const atcphy_mode_config_t *usb2_n = atcphy_mode_config(ATCPHY_MODE_USB2, false);
    const atcphy_mode_config_t *usb2_s = atcphy_mode_config(ATCPHY_MODE_USB2, true);

    assert(off_n && off_s && usb2_n && usb2_s);

    assert(off_n->crossbar_protocol == ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3);
    assert(off_s->crossbar_protocol == ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED);
    assert(usb2_n->crossbar_protocol == ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3);
    assert(usb2_s->crossbar_protocol == ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED);

    /* Both OFF and USB2 always keep both SuperSpeed lanes off and the
     * pipehandler on the safe dummy backend -- this is the structural
     * invariant the "safe, auto-wired" path in src/atcphy.c depends on. */
    const atcphy_mode_config_t *cfgs[] = {off_n, off_s, usb2_n, usb2_s};
    for (size_t i = 0; i < 4; i++) {
        assert(cfgs[i]->lane_mode[0] == ATCPHY_LANE_MODE_OFF);
        assert(cfgs[i]->lane_mode[1] == ATCPHY_LANE_MODE_OFF);
        assert(cfgs[i]->dp_lane[0] == false && cfgs[i]->dp_lane[1] == false);
        assert(cfgs[i]->enable_dp_aux == false);
        assert(cfgs[i]->pipe_state == ATCPHY_PIPE_STATE_DUMMY);
    }
}

static void test_mode_config_usb3_swap_mirrors_lanes(void)
{
    const atcphy_mode_config_t *n = atcphy_mode_config(ATCPHY_MODE_USB3, false);
    const atcphy_mode_config_t *s = atcphy_mode_config(ATCPHY_MODE_USB3, true);

    assert(n->crossbar_protocol == ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP);
    assert(s->crossbar_protocol == ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED);
    assert(n->lane_mode[0] == ATCPHY_LANE_MODE_USB3 && n->lane_mode[1] == ATCPHY_LANE_MODE_DP);
    assert(s->lane_mode[0] == ATCPHY_LANE_MODE_DP && s->lane_mode[1] == ATCPHY_LANE_MODE_USB3);
    assert(n->set_swap == false && s->set_swap == true);
    assert(n->pipe_state == ATCPHY_PIPE_STATE_USB3 && s->pipe_state == ATCPHY_PIPE_STATE_USB3);
    /* USB3-only never enables DP AUX; USB3_DP does. */
    assert(n->enable_dp_aux == false);
    assert(atcphy_mode_config(ATCPHY_MODE_USB3_DP, false)->enable_dp_aux == true);
}

static void test_mode_config_dp_asymmetric_swap(void)
{
    const atcphy_mode_config_t *n = atcphy_mode_config(ATCPHY_MODE_DP, false);
    const atcphy_mode_config_t *s = atcphy_mode_config(ATCPHY_MODE_DP, true);

    assert(n->crossbar_dp_single_pma == ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100);
    assert(n->crossbar_dp_both_pma == true);
    /* atc.c:774-781: the swapped DP entry is intentionally NOT a mirror --
     * both_pma and set_swap are forced false. Preserve that asymmetry. */
    assert(s->crossbar_dp_single_pma == ATCPHY_CORE_ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008);
    assert(s->crossbar_dp_both_pma == false);
    assert(s->set_swap == false);
    assert(n->crossbar_protocol == s->crossbar_protocol); /* both DP, atc.c:767,775 */
}

static void test_mode_config_invalid(void)
{
    assert(atcphy_mode_config((atcphy_mode_t)999, false) == NULL);
}

/* ------------------------------------------------------------------ */
/* Pipehandler mux decode                                              */
/* ------------------------------------------------------------------ */

static void test_pipe_mux_decode_matches_m1n1_whole_register_values(void)
{
    /* usb.c:42-44 whole-register values must decode onto the field split. */
    assert(atcphy_pipe_mux_decode(0x08) == ATCPHY_PIPE_BACKEND_USB3);
    assert(atcphy_pipe_mux_decode(0x11) == ATCPHY_PIPE_BACKEND_USB4);
    assert(atcphy_pipe_mux_decode(0x22) == ATCPHY_PIPE_BACKEND_DUMMY);
    assert(atcphy_pipe_mux_decode(0x00) == ATCPHY_PIPE_BACKEND_CLK_OFF);
    /* CLK=USB3 but DATA=DUMMY: mid-transition / foreign state. */
    assert(atcphy_pipe_mux_decode(0x0A) == ATCPHY_PIPE_BACKEND_INCONSISTENT);
}

/* ------------------------------------------------------------------ */
/* DP link rates                                                       */
/* ------------------------------------------------------------------ */

static void test_dp_rate_wire_and_khz_round_trip(void)
{
    struct {
        atcphy_dp_rate_t rate;
        u32 wire;
        u32 khz;
    } cases[] = {
        {ATCPHY_DP_RATE_RBR, 0x06, 1620},
        {ATCPHY_DP_RATE_HBR, 0x0A, 2700},
        {ATCPHY_DP_RATE_HBR2, 0x14, 5400},
        {ATCPHY_DP_RATE_HBR3, 0x1E, 8100},
    };
    for (size_t i = 0; i < 4; i++) {
        assert(atcphy_dp_rate_to_wire(cases[i].rate) == cases[i].wire);
        assert(atcphy_dp_rate_khz(cases[i].rate) == cases[i].khz);
        assert(atcphy_dp_rate_from_wire(cases[i].wire) == (int)cases[i].rate);
    }
    assert(atcphy_dp_rate_from_wire(0xFF) == -1);
    assert(atcphy_dp_lr_config((atcphy_dp_rate_t)99) == NULL);
    /* HBR3 is the ceiling -- no rate above it exists. */
    assert(ATCPHY_DP_RATE_COUNT == 4);
}

/* ------------------------------------------------------------------ */
/* Tunable vocabulary + validation                                     */
/* ------------------------------------------------------------------ */

static void test_tunable_vocab_count_and_lookup(void)
{
    size_t count;
    const atcphy_tunable_vocab_t *vocab = atcphy_t6020_tunable_vocab(&count);
    assert(vocab != NULL);
    assert(count == ATCPHY_T6020_TUNABLE_COUNT);
    assert(count == 28);

    const atcphy_tunable_vocab_t *fabric = atcphy_t6020_tunable_lookup("tunable_ATC_FABRIC");
    assert(fabric != NULL);
    assert(fabric->target == ATCPHY_TUNABLE_TARGET_CORE);
    assert(fabric->block_offset == 0x45000); /* ADT:reg25 */

    const atcphy_tunable_vocab_t *axi2af = atcphy_t6020_tunable_lookup("tunable_ATC0AXI2AF");
    assert(axi2af != NULL);
    assert(axi2af->target == ATCPHY_TUNABLE_TARGET_AXI2AF);
    assert(axi2af->block_offset == 0x0);

    const atcphy_tunable_vocab_t *rxeq0 = atcphy_t6020_tunable_lookup("tunable_USB_LN0_AUSPMA_RX_EQ");
    assert(rxeq0 != NULL);
    assert(rxeq0->block_offset == 0xa000); /* ADT:reg26, == LN0_AUSPMA_RX_EQ */
    assert(rxeq0->lane == 0);

    const atcphy_tunable_vocab_t *rxeq1 = atcphy_t6020_tunable_lookup("tunable_USB_LN1_AUSPMA_RX_EQ");
    assert(rxeq1 != NULL);
    assert(rxeq1->block_offset == 0x11000); /* ADT:reg27, == LN1_AUSPMA_RX_EQ */
    assert(rxeq1->lane == 1);

    assert(atcphy_t6020_tunable_lookup("tunable_DOES_NOT_EXIST") == NULL);
    assert(atcphy_t6020_tunable_lookup(NULL) == NULL);

    /* Every global (non-lane-scoped) required entry must target a distinct
     * block_offset EXCEPT the intentional duplicate (CIO_CIO3PLL_TOP mirrors
     * CIO3PLL_TOP, both 0x2800 -- see kboot_atc.c:80 comment). Spot check a
     * handful for uniqueness to catch transcription typos. */
    assert(atcphy_t6020_tunable_lookup("tunable_AUSPLL_TOP")->block_offset == 0x2000);
    assert(atcphy_t6020_tunable_lookup("tunable_AUSPLL_CORE")->block_offset == 0x2200);
    assert(atcphy_t6020_tunable_lookup("tunable_CIO3PLL_TOP")->block_offset == 0x2800);
    assert(atcphy_t6020_tunable_lookup("tunable_CIO3PLL_CORE")->block_offset == 0x2a00);
    assert(atcphy_t6020_tunable_lookup("tunable_AUS_CMN_TOP")->block_offset == 0x800);
    assert(atcphy_t6020_tunable_lookup("tunable_AUS_CMN_SHM")->block_offset == 0xa00);
}

static void test_tunable_validate_empty_and_length(void)
{
    const atcphy_tunable_vocab_t *vocab = atcphy_t6020_tunable_lookup("tunable_ATC_FABRIC");
    atcphy_tunable_record_t out[8];
    size_t out_count, bad;

    assert(atcphy_tunable_validate(vocab, NULL, 0, out, 8, &out_count, &bad) ==
           ATCPHY_TUNABLE_BLOB_EMPTY);
    assert(out_count == 0);

    u8 bad_len_blob[10] = {0};
    assert(atcphy_tunable_validate(vocab, bad_len_blob, sizeof(bad_len_blob), out, 8, &out_count,
                                   &bad) == ATCPHY_TUNABLE_BLOB_BAD_LENGTH);
}

static void put_le32(u8 *p, u32 v)
{
    p[0] = (u8)(v & 0xff);
    p[1] = (u8)((v >> 8) & 0xff);
    p[2] = (u8)((v >> 16) & 0xff);
    p[3] = (u8)((v >> 24) & 0xff);
}

/* Builds one 12-byte {offset:24,size:8,mask:32,value:32} record. */
static void put_record(u8 *rec, u32 offset, u8 size, u32 mask, u32 value)
{
    u32 word0 = (offset & 0xFFFFFFu) | ((u32)size << 24);
    put_le32(&rec[0], word0);
    put_le32(&rec[4], mask);
    put_le32(&rec[8], value);
}

static void test_tunable_validate_accepts_and_resolves_offset(void)
{
    const atcphy_tunable_vocab_t *vocab = atcphy_t6020_tunable_lookup("tunable_AUSPLL_TOP");
    assert(vocab->block_offset == 0x2000);

    u8 blob[24]; /* two records */
    put_record(&blob[0], 0x80, 32, 0xFFFFFFFFu, 0x12345678u); /* AUSPLL_FREQ_DESC_A-ish */
    put_record(&blob[12], 0x84, 32, 0x000000FFu, 0x000000AAu);

    atcphy_tunable_record_t out[4];
    size_t out_count = 0, bad = 999;
    assert(atcphy_tunable_validate(vocab, blob, sizeof(blob), out, 4, &out_count, &bad) ==
           ATCPHY_TUNABLE_BLOB_OK);
    assert(out_count == 2);
    assert(out[0].offset == 0x2000 + 0x80);
    assert(out[0].mask == 0xFFFFFFFFu);
    assert(out[0].value == 0x12345678u);
    assert(out[1].offset == 0x2000 + 0x84);
    assert(bad == 999); /* untouched on success */
}

static void test_tunable_validate_rejects_bad_records(void)
{
    const atcphy_tunable_vocab_t *vocab = atcphy_t6020_tunable_lookup("tunable_AUSPLL_TOP");
    atcphy_tunable_record_t out[4];
    size_t out_count, bad;

    /* size != 32 */
    u8 bad_size[12];
    put_record(bad_size, 0x0, 16, 0, 0);
    assert(atcphy_tunable_validate(vocab, bad_size, sizeof(bad_size), out, 4, &out_count, &bad) ==
           ATCPHY_TUNABLE_BLOB_BAD_RECORD);
    assert(bad == 0);

    /* unaligned offset */
    u8 bad_align[12];
    put_record(bad_align, 0x1, 32, 0, 0);
    assert(atcphy_tunable_validate(vocab, bad_align, sizeof(bad_align), out, 4, &out_count,
                                   &bad) == ATCPHY_TUNABLE_BLOB_BAD_RECORD);

    /* out of range: block_size is 0x4000, offset 0x4000 means offset+4 > 0x4000 */
    u8 bad_range[12];
    put_record(bad_range, 0x4000, 32, 0, 0);
    assert(atcphy_tunable_validate(vocab, bad_range, sizeof(bad_range), out, 4, &out_count,
                                   &bad) == ATCPHY_TUNABLE_BLOB_BAD_RECORD);

    /* second record bad -> bad_record_index == 1 */
    u8 two[24];
    put_record(&two[0], 0x0, 32, 0, 0);
    put_record(&two[12], 0x1, 32, 0, 0);
    assert(atcphy_tunable_validate(vocab, two, sizeof(two), out, 4, &out_count, &bad) ==
           ATCPHY_TUNABLE_BLOB_BAD_RECORD);
    assert(bad == 1);
}

static void test_tunable_apply_one_rmw_semantics(void)
{
    /* Mirrors apple_tunable_apply (tunable.c:62-75): new = (old & ~mask) | value. */
    assert(atcphy_tunable_apply_one(0xFFFFFFFFu, 0x0000FFFFu, 0x00001234u) == 0xFFFF1234u);
    assert(atcphy_tunable_apply_one(0x00000000u, 0xFFFFFFFFu, 0xDEADBEEFu) == 0xDEADBEEFu);
    /* mask=0, value=0 is a true no-op: nothing is cleared, nothing is set. */
    assert(atcphy_tunable_apply_one(0xAAAAAAAAu, 0x00000000u, 0x00000000u) == 0xAAAAAAAAu);
    /* Literal Linux semantics (tunable.c:70-71): val = (old & ~mask) | value.
     * If mask doesn't cover a bit that value sets, that bit still gets
     * forced on -- well-formed records never do this, but the math is not
     * required to protect against it. */
    assert(atcphy_tunable_apply_one(0xAAAAAAAAu, 0x00000000u, 0xFFFFFFFFu) == 0xFFFFFFFFu);
}

static void test_fuse_policy(void)
{
    assert(atcphy_fuse_policy("atc-phy,t6020") == ATCPHY_FUSES_EMPTY_INTENTIONAL);
    assert(atcphy_fuse_policy("atc-phy,t8103") == ATCPHY_FUSES_REQUIRED);
    assert(atcphy_fuse_policy("atc-phy,t8112") == ATCPHY_FUSES_UNKNOWN);
    assert(atcphy_fuse_policy(NULL) == ATCPHY_FUSES_UNKNOWN);
}

/* ------------------------------------------------------------------ */
/* Op-list interpreter                                                 */
/* ------------------------------------------------------------------ */

static void test_seq_apply_write_set_clear_mask(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx);

    atcphy_seq_op_t ops[] = {
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_WRITE, 0x10, 0xAA, 0, "t"},
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_SET, 0x10, 0x01, 0, "t"},
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_CLEAR, 0x10, 0x80, 0, "t"},
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_MASK, 0x10, 0xFF, 0x55, "t"},
    };
    size_t fail = 999;
    assert(atcphy_seq_apply(ops, 4, fake_read, fake_write, fake_delay, &ctx, &fail) == 0);
    assert(fail == 999);
    assert(fake_read(&ctx, ATCPHY_BLOCK_CORE, 0x10) == 0x55);

    /* Different blocks at the same offset must not alias. */
    fake_write(&ctx, ATCPHY_BLOCK_USB2PHY, 0x10, 0x77);
    assert(fake_read(&ctx, ATCPHY_BLOCK_CORE, 0x10) == 0x55);
    assert(fake_read(&ctx, ATCPHY_BLOCK_USB2PHY, 0x10) == 0x77);
}

static void test_seq_apply_delay_accumulates(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx);
    atcphy_seq_op_t ops[] = {
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 10, 0, "t"},
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_DELAY_US, 0, 15, 0, "t"},
    };
    assert(atcphy_seq_apply(ops, 2, fake_read, fake_write, fake_delay, &ctx, NULL) == 0);
    assert(ctx.total_delay_us == 25);
}

static void test_seq_apply_poll_set_succeeds_when_bit_appears(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx);
    /* Pre-seed the target with the bit already set: POLL_SET must succeed
     * immediately without requiring a prior write op. */
    fake_write(&ctx, ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_PIPEHANDLER_LOCK_ACK,
               ATCPHY_PIPEHANDLER_LOCK_EN);

    atcphy_seq_op_t ops[] = {
        {ATCPHY_BLOCK_PIPEHANDLER, ATCPHY_OP_POLL_SET, ATCPHY_PIPEHANDLER_LOCK_ACK,
         ATCPHY_PIPEHANDLER_LOCK_EN, 1000, "t"},
    };
    size_t fail = 999;
    assert(atcphy_seq_apply(ops, 1, fake_read, fake_write, fake_delay, &ctx, &fail) == 0);
    assert(fail == 999);
}

static void test_seq_apply_poll_set_times_out_and_reports_index(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx);

    atcphy_seq_op_t ops[] = {
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_WRITE, 0x0, 0, 0, "t"}, /* index 0, always fine */
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_SET, ATCPHY_CORE_ATCPHY_POWER_STAT,
         ATCPHY_CORE_POWER_SLEEP_BIG, 50, "t"}, /* index 1: bit never appears */
    };
    size_t fail = 999;
    assert(atcphy_seq_apply(ops, 2, fake_read, fake_write, fake_delay, &ctx, &fail) == -1);
    assert(fail == 1);
}

static void test_seq_apply_poll_clear_succeeds_and_times_out(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx);
    fake_write(&ctx, ATCPHY_BLOCK_CORE, 0x4, 0);
    atcphy_seq_op_t ok[] = {
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_CLEAR, 0x4, 0x1, 100, "t"},
    };
    assert(atcphy_seq_apply(ok, 1, fake_read, fake_write, fake_delay, &ctx, NULL) == 0);

    fake_write(&ctx, ATCPHY_BLOCK_CORE, 0x8, 0x1);
    atcphy_seq_op_t timeout[] = {
        {ATCPHY_BLOCK_CORE, ATCPHY_OP_POLL_CLEAR, 0x8, 0x1, 50, "t"},
    };
    size_t fail = 999;
    assert(atcphy_seq_apply(timeout, 1, fake_read, fake_write, fake_delay, &ctx, &fail) == -1);
    assert(fail == 0);
}

static void test_seq_apply_null_args_rejected(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx);
    atcphy_seq_op_t op = {ATCPHY_BLOCK_CORE, ATCPHY_OP_WRITE, 0, 0, 0, "t"};
    assert(atcphy_seq_apply(NULL, 1, fake_read, fake_write, fake_delay, &ctx, NULL) == -1);
    assert(atcphy_seq_apply(&op, 1, NULL, fake_write, fake_delay, &ctx, NULL) == -1);
    assert(atcphy_seq_apply(&op, 1, fake_read, NULL, fake_delay, &ctx, NULL) == -1);
}

/* ------------------------------------------------------------------ */
/* Fixed sequences: content-fidelity spot checks                       */
/* ------------------------------------------------------------------ */

static void test_fixed_sequences_are_nonempty_and_scoped_to_expected_block(void)
{
    size_t n;
    const atcphy_seq_op_t *ops;

    ops = atcphy_seq_usb2_power_on(&n);
    assert(ops && n > 0);
    for (size_t i = 0; i < n; i++)
        assert(ops[i].block == ATCPHY_BLOCK_USB2PHY || ops[i].kind == ATCPHY_OP_DELAY_US);
    /* Last op is the well-known m1n1 bring-up write, usb.c:167 / atc.c:1691. */
    assert(ops[n - 1].kind == ATCPHY_OP_WRITE);
    assert(ops[n - 1].offset == ATCPHY_USB2PHY_USBCTL);
    assert(ops[n - 1].arg1 == ATCPHY_USB2PHY_USBCTL_RUN);

    ops = atcphy_seq_usb2_power_off(&n);
    assert(ops && n > 0);
    assert(ops[0].kind == ATCPHY_OP_WRITE);
    assert(ops[0].arg1 == ATCPHY_USB2PHY_USBCTL_ISOLATION);

    ops = atcphy_seq_pipehandler_dummy(&n);
    assert(ops && n > 0);
    for (size_t i = 0; i < n; i++)
        assert(ops[i].block == ATCPHY_BLOCK_PIPEHANDLER);

    ops = atcphy_seq_pipehandler_usb3_host_bist(&n);
    assert(ops && n > 0);
    /* This sequence must ONLY touch PIPEHANDLER and CORE -- never USB2PHY,
     * AXI2AF or LPDPTX (it's a PIPE-mux/BIST sequence, not a full reconfig). */
    for (size_t i = 0; i < n; i++)
        assert(ops[i].block == ATCPHY_BLOCK_PIPEHANDLER || ops[i].block == ATCPHY_BLOCK_CORE);

    ops = atcphy_seq_core_power_on(&n);
    assert(ops && n > 0);
    for (size_t i = 0; i < n; i++)
        assert(ops[i].block == ATCPHY_BLOCK_CORE);

    ops = atcphy_seq_core_power_off(&n);
    assert(ops && n > 0);

    ops = atcphy_seq_cfg0_sleep_override(&n);
    assert(ops && n == 18); /* 9 register ops + 9 delays, atc.c:1746-1771 */

    ops = atcphy_seq_cio3pll_enable(&n);
    assert(ops && n == 2);

    ops = atcphy_seq_auspll_fsm_override(&n);
    assert(ops && n == 2);

    ops = atcphy_seq_release_phy_reset(&n);
    assert(ops && n == 1);
    assert(ops[0].arg1 == ATCPHY_CORE_POWER_PHY_RESET_N);

    ops = atcphy_seq_dp_aux_enable(&n);
    assert(ops && n > 0);
    ops = atcphy_seq_dp_aux_disable(&n);
    assert(ops && n > 0);
}

/* ------------------------------------------------------------------ */
/* Parameterized builders, replayed through the interpreter            */
/* ------------------------------------------------------------------ */

static void test_lane_config_off_orientation_produces_expected_crossbar_and_lanemode(void)
{
    atcphy_seq_op_t ops[ATCPHY_LANE_CONFIG_MAX_OPS];

    for (int swap = 0; swap < 2; swap++) {
        fake_ctx_t ctx;
        fake_reset(&ctx);

        size_t n = atcphy_build_lane_config_ops(ATCPHY_MODE_OFF, swap != 0, ops,
                                                ATCPHY_LANE_CONFIG_MAX_OPS);
        assert(n > 0 && n <= ATCPHY_LANE_CONFIG_MAX_OPS);

        size_t fail = 999;
        assert(atcphy_seq_apply(ops, n, fake_read, fake_write, NULL, &ctx, &fail) == 0);
        assert(fail == 999);

        u32 crossbar = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ACIOPHY_CROSSBAR);
        u32 lane_mode = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ACIOPHY_LANE_MODE);
        u32 misc = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ATCPHY_MISC);

        u32 expect_crossbar = swap ? ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED
                                    : ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3;
        assert((crossbar & ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_MASK) == expect_crossbar);
        /* All 4 LANE_MODE fields must read OFF(3): 3 | 3<<3 | 3<<6 | 3<<9 */
        assert(lane_mode == (3u | (3u << 3) | (3u << 6) | (3u << 9)));
        assert((misc & ATCPHY_CORE_MISC_LANE_SWAP) == 0); /* OFF never sets swap */
    }
}

static void test_lane_config_usb3_swapped_sets_lane_swap_and_mirrors_lanes(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx);
    atcphy_seq_op_t ops[ATCPHY_LANE_CONFIG_MAX_OPS];

    size_t n =
        atcphy_build_lane_config_ops(ATCPHY_MODE_USB3, true, ops, ATCPHY_LANE_CONFIG_MAX_OPS);
    assert(n > 0);
    assert(atcphy_seq_apply(ops, n, fake_read, fake_write, NULL, &ctx, NULL) == 0);

    u32 crossbar = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ACIOPHY_CROSSBAR);
    u32 misc = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ATCPHY_MISC);
    assert((crossbar & ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_MASK) ==
           ATCPHY_CORE_ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED);
    assert(misc & ATCPHY_CORE_MISC_LANE_SWAP);

    u32 lane_mode = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ACIOPHY_LANE_MODE);
    u32 rx0 = (lane_mode & ATCPHY_CORE_LANE_MODE_RX0_MASK) >> ATCPHY_CORE_LANE_MODE_RX0_SHIFT;
    u32 rx1 = (lane_mode & ATCPHY_CORE_LANE_MODE_RX1_MASK) >> ATCPHY_CORE_LANE_MODE_RX1_SHIFT;
    assert(rx0 == ATCPHY_LANE_MODE_DP);   /* swapped: lane0 becomes DP */
    assert(rx1 == ATCPHY_LANE_MODE_USB3); /* swapped: lane1 becomes USB3 */
}

static void test_lane_config_invalid_mode_returns_zero(void)
{
    atcphy_seq_op_t ops[ATCPHY_LANE_CONFIG_MAX_OPS];
    assert(atcphy_build_lane_config_ops((atcphy_mode_t)999, false, ops,
                                        ATCPHY_LANE_CONFIG_MAX_OPS) == 0);
    assert(atcphy_build_lane_config_ops(ATCPHY_MODE_OFF, false, NULL, 4) == 0);
}

static void test_dp_rate_ops_encode_freq_desc_a_target(void)
{
    static const atcphy_dp_rate_t rates[] = {ATCPHY_DP_RATE_RBR, ATCPHY_DP_RATE_HBR,
                                             ATCPHY_DP_RATE_HBR2, ATCPHY_DP_RATE_HBR3};

    for (size_t i = 0; i < 4; i++) {
        fake_ctx_t ctx;
        fake_reset(&ctx);
        /* Pre-seed every polled register so POLL_SET ops succeed instantly;
         * this test only cares about the computed register CONTENT, not the
         * live hardware handshake (that's atc.c's job, not this file's). */
        fake_write(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ACIOPHY_CMN_SHM_STS_REG0,
                   ATCPHY_CORE_ACIOPHY_CMN_SHM_STS_REG0_CMD_READY);
        fake_write(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_ACIOPHY_DP_PCLK_STAT,
                   ATCPHY_CORE_ACIOPHY_AUSPLL_LOCK);
        fake_write(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE,
                   ATCPHY_CORE_AUSPLL_APB_CMD_OVERRIDE_ACK);

        atcphy_seq_op_t ops[ATCPHY_DP_RATE_MAX_OPS];
        size_t n = atcphy_build_dp_rate_ops(rates[i], ops, ATCPHY_DP_RATE_MAX_OPS);
        assert(n > 0 && n <= ATCPHY_DP_RATE_MAX_OPS);

        size_t fail = 999;
        assert(atcphy_seq_apply(ops, n, fake_read, fake_write, NULL, &ctx, &fail) == 0);
        assert(fail == 999);

        const atcphy_dp_lr_config_t *cfg = atcphy_dp_lr_config(rates[i]);
        u32 desc_a = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_AUSPLL_FREQ_DESC_A);
        u32 target = (desc_a & ATCPHY_CORE_AUSPLL_FD_FREQ_COUNT_TARGET_MASK) >>
                     ATCPHY_CORE_AUSPLL_FD_FREQ_COUNT_TARGET_SHIFT;
        assert(target == cfg->freqinit_count_target);

        u32 dtc_vreg = fake_read(&ctx, ATCPHY_BLOCK_CORE, ATCPHY_CORE_AUSPLL_CLKOUT_DTC_VREG);
        bool bypass_set = (dtc_vreg & ATCPHY_CORE_AUSPLL_DTC_VREG_BYPASS) != 0;
        assert(bypass_set == cfg->plla_clkout_vreg_bypass);
    }
}

static void test_dp_rate_ops_timeout_propagates(void)
{
    fake_ctx_t ctx;
    fake_reset(&ctx); /* CMD_READY never set -> first poll must time out */
    atcphy_seq_op_t ops[ATCPHY_DP_RATE_MAX_OPS];
    size_t n = atcphy_build_dp_rate_ops(ATCPHY_DP_RATE_HBR, ops, ATCPHY_DP_RATE_MAX_OPS);
    assert(n > 0);
    size_t fail = 999;
    assert(atcphy_seq_apply(ops, n, fake_read, fake_write, NULL, &ctx, &fail) == -1);
    assert(fail == 0); /* the CMD_READY poll is op index 0 */
}

int main(void)
{
    test_mode_config_off_and_usb2_are_orientation_swap_only();
    test_mode_config_usb3_swap_mirrors_lanes();
    test_mode_config_dp_asymmetric_swap();
    test_mode_config_invalid();

    test_pipe_mux_decode_matches_m1n1_whole_register_values();

    test_dp_rate_wire_and_khz_round_trip();

    test_tunable_vocab_count_and_lookup();
    test_tunable_validate_empty_and_length();
    test_tunable_validate_accepts_and_resolves_offset();
    test_tunable_validate_rejects_bad_records();
    test_tunable_apply_one_rmw_semantics();
    test_fuse_policy();

    test_seq_apply_write_set_clear_mask();
    test_seq_apply_delay_accumulates();
    test_seq_apply_poll_set_succeeds_when_bit_appears();
    test_seq_apply_poll_set_times_out_and_reports_index();
    test_seq_apply_poll_clear_succeeds_and_times_out();
    test_seq_apply_null_args_rejected();

    test_fixed_sequences_are_nonempty_and_scoped_to_expected_block();

    test_lane_config_off_orientation_produces_expected_crossbar_and_lanemode();
    test_lane_config_usb3_swapped_sets_lane_swap_and_mirrors_lanes();
    test_lane_config_invalid_mode_returns_zero();

    test_dp_rate_ops_encode_freq_desc_a_target();
    test_dp_rate_ops_timeout_propagates();

    puts("atcphy_core tests: PASS");
    return 0;
}
