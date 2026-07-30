/* SPDX-License-Identifier: MIT */

/*
 * Host tests for the Apple/Asahi PCIe port bring-up sequence.
 *
 * The reference ordering is Linux drivers/pci/controller/pcie-apple.c
 * (AsahiLinux/linux, asahi branch): apple_pcie_setup_link() enables APPCLK,
 * asserts PERST#, runs apple_pcie_setup_refclk(), waits Tperst-clk, then
 * releases PERST#; the tail of apple_pcie_setup_port() enables refclk clock
 * gating, starts LTSSM and waits for the link.
 *
 * These tests record every register access, PERST# transition and delay in
 * order and assert the whole trace, so an ordering regression fails here rather
 * than on a hardware boot.
 */

#include "pcie.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

static void check(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "pcie-bringup: FAIL: %s\n", message);
        failures++;
    }
}

static void check_u32(u32 got, u32 want, const char *message)
{
    if (got != want) {
        fprintf(stderr, "pcie-bringup: FAIL: %s (got %#x, want %#x)\n", message, got, want);
        failures++;
    }
}

/* J414s port 0, as measured on hardware. */
#define PORT0_BASE     UINT64_C(0x594008000)
#define PORT0_PHY_BASE UINT64_C(0x594084000)
#define PORT1_BASE     UINT64_C(0x595008000)

#define TRACE_MAX 64

enum event_kind {
    EV_READ,
    EV_WRITE,
    EV_SET,
    EV_CLEAR,
    EV_POLL,
    EV_DELAY,
    EV_PERST,
};

struct event {
    enum event_kind kind;
    u64 address;
    u32 a;
    u32 b;
    u32 timeout_us;
    bool asserted;
};

struct fake {
    struct event trace[TRACE_MAX];
    unsigned count;
    bool overflow;

    /* Register file: only the handful of offsets the sequence touches. */
    u32 lane_cfg;
    u32 linksts;
    u32 ltssmctl;
    u32 appclk;
    u32 perst;

    /* Fault injection. */
    bool refclk0_ack_stuck;
    bool refclk1_ack_stuck;
    bool link_never_up;
    bool link_up_after_ltssm;
    bool perst_assert_fails;
    bool perst_release_fails;

    /* Observed PERST# state; starts asserted, as J414s does out of iBoot. */
    bool perst_asserted;
    bool perst_ever_released;
    bool refclk_enabled_while_perst_asserted;
};

static struct fake fake;

static void record(struct event ev)
{
    if (fake.count >= TRACE_MAX) {
        fake.overflow = true;
        return;
    }
    fake.trace[fake.count++] = ev;
}

static u32 *reg_of(u64 address, const struct pcie_port_bringup *cfg)
{
    if (address == cfg->port_phy_base + PCIE_PHY_LANE_CFG && cfg->port_phy_base)
        return &fake.lane_cfg;
    if (address == cfg->port_base + PCIE_PORT_LINKSTS)
        return &fake.linksts;
    if (address == cfg->port_base + PCIE_PORT_LTSSMCTL)
        return &fake.ltssmctl;
    if (address == cfg->port_base + PCIE_PORT_APPCLK)
        return &fake.appclk;
    if (address == cfg->port_base + cfg->perst_reg)
        return &fake.perst;
    return NULL;
}

/* The configuration under test, so reg_of() can decode addresses. */
static const struct pcie_port_bringup *active_cfg;

static int fake_read32(void *context, u64 address, u32 *value)
{
    u32 *reg = reg_of(address, active_cfg);

    (void)context;
    record((struct event){.kind = EV_READ, .address = address});
    *value = reg ? *reg : 0;
    return 0;
}

static int fake_write32(void *context, u64 address, u32 value)
{
    u32 *reg = reg_of(address, active_cfg);

    (void)context;
    record((struct event){.kind = EV_WRITE, .address = address, .a = value});
    if (reg)
        *reg = value;
    if (address == active_cfg->port_base + PCIE_PORT_LTSSMCTL &&
        (value & PCIE_PORT_LTSSMCTL_START) && fake.link_up_after_ltssm)
        fake.linksts |= PCIE_PORT_LINKSTS_UP;
    return 0;
}

static int fake_set32(void *context, u64 address, u32 set)
{
    u32 *reg = reg_of(address, active_cfg);

    (void)context;
    record((struct event){.kind = EV_SET, .address = address, .a = set});
    if (reg)
        *reg |= set;
    if (reg == &fake.lane_cfg &&
        (set & (PCIE_PHY_LANE_CFG_REFCLKEN0 | PCIE_PHY_LANE_CFG_REFCLKEN1)) &&
        fake.perst_asserted)
        fake.refclk_enabled_while_perst_asserted = true;
    return 0;
}

static int fake_clear32(void *context, u64 address, u32 clear)
{
    u32 *reg = reg_of(address, active_cfg);

    (void)context;
    record((struct event){.kind = EV_CLEAR, .address = address, .a = clear});
    if (reg)
        *reg &= ~clear;
    return 0;
}

static int fake_poll32(void *context, u64 address, u32 mask, u32 target, u32 timeout_us)
{
    u32 *reg = reg_of(address, active_cfg);

    (void)context;
    record((struct event){
        .kind = EV_POLL, .address = address, .a = mask, .b = target, .timeout_us = timeout_us});

    if (reg == &fake.lane_cfg) {
        if (mask == PCIE_PHY_LANE_CFG_REFCLK0ACK) {
            if (fake.refclk0_ack_stuck)
                return -1;
            fake.lane_cfg |= PCIE_PHY_LANE_CFG_REFCLK0ACK;
            return 0;
        }
        if (mask == PCIE_PHY_LANE_CFG_REFCLK1ACK) {
            if (fake.refclk1_ack_stuck)
                return -1;
            fake.lane_cfg |= PCIE_PHY_LANE_CFG_REFCLK1ACK;
            return 0;
        }
    }

    if (reg == &fake.linksts && mask == PCIE_PORT_LINKSTS_UP) {
        if (fake.link_never_up)
            return -1;
        fake.linksts |= PCIE_PORT_LINKSTS_UP;
        return 0;
    }

    return (reg && (*reg & mask) == target) ? 0 : -1;
}

static void fake_delay_us(void *context, u32 us)
{
    (void)context;
    record((struct event){.kind = EV_DELAY, .a = us});
}

static int fake_perst_set(void *context, bool asserted)
{
    (void)context;
    record((struct event){.kind = EV_PERST, .asserted = asserted});
    if (asserted && fake.perst_assert_fails)
        return -1;
    if (!asserted && fake.perst_release_fails)
        return -1;
    fake.perst_asserted = asserted;
    if (!asserted)
        fake.perst_ever_released = true;
    return 0;
}

static const struct pcie_port_bringup_ops ops = {
    .read32 = fake_read32,
    .write32 = fake_write32,
    .set32 = fake_set32,
    .clear32 = fake_clear32,
    .poll32 = fake_poll32,
    .delay_us = fake_delay_us,
    .perst_set = fake_perst_set,
};

static void reset_fake(void)
{
    memset(&fake, 0, sizeof(fake));
    /* J414s reports 0xab000208 with UP and BUSY both clear. */
    fake.linksts = UINT32_C(0xab000208);
    fake.perst_asserted = true;
}

static const char *kind_name(enum event_kind kind)
{
    switch (kind) {
        case EV_READ:
            return "read";
        case EV_WRITE:
            return "write";
        case EV_SET:
            return "set";
        case EV_CLEAR:
            return "clear";
        case EV_POLL:
            return "poll";
        case EV_DELAY:
            return "delay";
        case EV_PERST:
            return "perst";
    }
    return "?";
}

static void dump_trace(void)
{
    for (unsigned i = 0; i < fake.count; i++) {
        const struct event *ev = &fake.trace[i];
        fprintf(stderr, "  [%2u] %-6s addr=%#llx a=%#x b=%#x timeout=%u asserted=%d\n", i,
                kind_name(ev->kind), (unsigned long long)ev->address, ev->a, ev->b, ev->timeout_us,
                ev->asserted);
    }
}

static void expect_event(unsigned index, struct event want, const char *message)
{
    const struct event *got;

    if (index >= fake.count) {
        fprintf(stderr, "pcie-bringup: FAIL: %s (trace has only %u events)\n", message,
                fake.count);
        failures++;
        return;
    }

    got = &fake.trace[index];
    if (got->kind != want.kind || got->address != want.address || got->a != want.a ||
        got->b != want.b || got->asserted != want.asserted) {
        fprintf(stderr, "pcie-bringup: FAIL: %s at index %u\n", message, index);
        fprintf(stderr, "  want %-6s addr=%#llx a=%#x b=%#x asserted=%d\n", kind_name(want.kind),
                (unsigned long long)want.address, want.a, want.b, want.asserted);
        fprintf(stderr, "  got  %-6s addr=%#llx a=%#x b=%#x asserted=%d\n", kind_name(got->kind),
                (unsigned long long)got->address, got->a, got->b, got->asserted);
        failures++;
    }
}

static struct pcie_port_bringup t602x_port0(bool have_gpio)
{
    struct pcie_port_bringup cfg = {
        .port_base = PORT0_BASE,
        .port_phy_base = PORT0_PHY_BASE,
        .perst_reg = UINT64_C(0x82c),
        .phy_ack_timeout_us = 50000,
        .link_up_timeout_us = PCIE_PERST_LINK_UP_TIMEOUT_US,
        .have_perst_gpio = have_gpio,
    };

    /* J414s publishes t-refclk-to-perst = 100 and perst-to-config = 100. */
    pcie_perst_delays_from_adt(&cfg.delays, true, 100, true, 100);
    return cfg;
}

/*
 * The full J414s port-0 sequence.  This is the test that would have caught the
 * bug: without the two EV_PERST events the endpoint stays in reset forever.
 */
static void test_t602x_full_sequence(void)
{
    struct pcie_port_bringup cfg = t602x_port0(true);
    u64 lane_cfg = PORT0_PHY_BASE + PCIE_PHY_LANE_CFG;
    unsigned i = 0;
    int ret;

    reset_fake();
    active_cfg = &cfg;
    fake.link_up_after_ltssm = true;

    ret = pcie_port_release_perst(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "release_perst succeeds on J414s port 0");

    expect_event(i++,
                 (struct event){
                     .kind = EV_SET, .address = PORT0_BASE + PCIE_PORT_APPCLK, .a = PCIE_PORT_APPCLK_EN},
                 "APPCLK is enabled first");
    expect_event(i++, (struct event){.kind = EV_PERST, .asserted = true},
                 "PERST# is asserted before the clocks come up");
    expect_event(i++,
                 (struct event){.kind = EV_CLEAR,
                                .address = lane_cfg,
                                .a = PCIE_PHY_LANE_CFG_REFCLK0REQ | PCIE_PHY_LANE_CFG_REFCLK1REQ},
                 "both refclk requests are cleared first");
    expect_event(
        i++,
        (struct event){.kind = EV_SET, .address = lane_cfg, .a = PCIE_PHY_LANE_CFG_REFCLK0REQ},
        "refclk 0 is requested");
    expect_event(i++,
                 (struct event){.kind = EV_POLL,
                                .address = lane_cfg,
                                .a = PCIE_PHY_LANE_CFG_REFCLK0ACK,
                                .b = PCIE_PHY_LANE_CFG_REFCLK0ACK},
                 "refclk 0 ack is waited for");
    expect_event(
        i++,
        (struct event){.kind = EV_SET, .address = lane_cfg, .a = PCIE_PHY_LANE_CFG_REFCLK1REQ},
        "refclk 1 is requested only after ack 0");
    expect_event(i++,
                 (struct event){.kind = EV_POLL,
                                .address = lane_cfg,
                                .a = PCIE_PHY_LANE_CFG_REFCLK1ACK,
                                .b = PCIE_PHY_LANE_CFG_REFCLK1ACK},
                 "refclk 1 ack is waited for");
    expect_event(
        i++, (struct event){.kind = EV_CLEAR, .address = lane_cfg, .a = PCIE_PHY_LANE_CFG_UNK14},
        "the undocumented bit 14 is cleared before REFCLKEN");
    expect_event(
        i++, (struct event){.kind = EV_SET, .address = lane_cfg, .a = PCIE_PHY_LANE_CFG_REFCLKEN0},
        "REFCLKEN bit 9 is set");
    expect_event(
        i++, (struct event){.kind = EV_SET, .address = lane_cfg, .a = PCIE_PHY_LANE_CFG_REFCLKEN1},
        "REFCLKEN bit 10 is set");
    expect_event(i++, (struct event){.kind = EV_DELAY, .a = 100},
                 "Tperst-clk is 100us and is waited after the refclk is up");
    expect_event(i++,
                 (struct event){
                     .kind = EV_SET, .address = PORT0_BASE + 0x82c, .a = PCIE_PORT_PERST_OFF},
                 "the internal PERST register is released");
    expect_event(i++, (struct event){.kind = EV_PERST, .asserted = false},
                 "the PERST# pad is released last");

    check_u32(fake.count, i, "release_perst emits exactly the expected events");
    if (fake.count != i)
        dump_trace();

    check(fake.refclk_enabled_while_perst_asserted,
          "the refclk was enabled while PERST# was still asserted");
    check(fake.perst_ever_released, "PERST# ends up released");
    check(!fake.perst_asserted, "PERST# is not left asserted");

    /* Phase 2. */
    fake.count = 0;
    fake.linksts &= ~PCIE_PORT_LINKSTS_UP;
    i = 0;

    ret = pcie_port_start_link(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "start_link succeeds once the link trains");

    expect_event(
        i++,
        (struct event){.kind = EV_SET, .address = lane_cfg, .a = PCIE_PHY_LANE_CFG_REFCLKCGEN},
        "refclk clock gating is enabled before LTSSM starts");
    expect_event(i++, (struct event){.kind = EV_READ, .address = PORT0_BASE + PCIE_PORT_LINKSTS},
                 "LINKSTS is checked before restarting LTSSM");
    expect_event(i++,
                 (struct event){.kind = EV_WRITE,
                                .address = PORT0_BASE + PCIE_PORT_LTSSMCTL,
                                .a = PCIE_PORT_LTSSMCTL_START},
                 "LTSSM training is started");
    expect_event(i++,
                 (struct event){.kind = EV_POLL,
                                .address = PORT0_BASE + PCIE_PORT_LINKSTS,
                                .a = PCIE_PORT_LINKSTS_UP,
                                .b = PCIE_PORT_LINKSTS_UP},
                 "the link is then polled for UP");
    check_u32(fake.count, i, "start_link emits exactly the expected events");
    if (fake.count != i)
        dump_trace();
}

/* Ordering invariants, stated independently of the exact trace above. */
static void test_ordering_invariants(void)
{
    struct pcie_port_bringup cfg = t602x_port0(true);
    int appclk = -1, perst_assert = -1, refclken = -1, delay = -1, perst_release = -1;
    int ret;

    reset_fake();
    active_cfg = &cfg;

    ret = pcie_port_release_perst(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "release_perst succeeds");

    for (unsigned i = 0; i < fake.count; i++) {
        const struct event *ev = &fake.trace[i];

        if (ev->kind == EV_SET && ev->address == PORT0_BASE + PCIE_PORT_APPCLK)
            appclk = (int)i;
        if (ev->kind == EV_PERST && ev->asserted)
            perst_assert = (int)i;
        if (ev->kind == EV_SET && ev->a == PCIE_PHY_LANE_CFG_REFCLKEN1)
            refclken = (int)i;
        if (ev->kind == EV_DELAY)
            delay = (int)i;
        if (ev->kind == EV_PERST && !ev->asserted)
            perst_release = (int)i;
    }

    check(appclk >= 0 && perst_assert >= 0 && refclken >= 0 && delay >= 0 && perst_release >= 0,
          "every ordering landmark is present");
    check(appclk < perst_assert, "APPCLK is enabled before PERST# is asserted");
    check(perst_assert < refclken, "PERST# is asserted before the refclk is enabled");
    check(refclken < delay, "Tperst-clk is measured from the refclk being enabled");
    check(delay < perst_release, "Tperst-clk elapses before PERST# is released");
}

/* No function-perst: behave exactly as before, never touching the pad. */
static void test_no_perst_gpio(void)
{
    struct pcie_port_bringup cfg = t602x_port0(false);
    int ret;

    reset_fake();
    active_cfg = &cfg;

    ret = pcie_port_release_perst(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "release_perst succeeds without a PERST# GPIO");

    for (unsigned i = 0; i < fake.count; i++)
        check(fake.trace[i].kind != EV_PERST, "no PERST# transition is emitted without a GPIO");

    check(fake.perst_asserted, "the pad is left exactly as firmware set it");

    /* The internal PERST register is still released, as m1n1 always did. */
    check(fake.perst & PCIE_PORT_PERST_OFF, "the internal PERST register is still released");
}

/* A port whose link is already up must not be reset out from under its owner. */
static void test_link_already_up_is_not_reset(void)
{
    struct pcie_port_bringup cfg = t602x_port0(true);
    int ret;

    reset_fake();
    active_cfg = &cfg;
    cfg.link_was_up = true;
    fake.linksts |= PCIE_PORT_LINKSTS_UP;

    ret = pcie_port_release_perst(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "release_perst succeeds on a live port");

    for (unsigned i = 0; i < fake.count; i++)
        check(fake.trace[i].kind != EV_PERST, "a live link is never PERST# cycled");

    ret = pcie_port_start_link(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "start_link succeeds on a live port");

    for (unsigned i = 0; i < fake.count; i++)
        check(!(fake.trace[i].kind == EV_WRITE &&
                fake.trace[i].address == PORT0_BASE + PCIE_PORT_LTSSMCTL),
              "LTSSM is not restarted under a live link");
}

/* Pre-T602x ports have no per-port PHY block; the refclk handshake is skipped. */
static void test_t81xx_has_no_port_phy(void)
{
    struct pcie_port_bringup cfg = {
        .port_base = PORT1_BASE,
        .port_phy_base = 0,
        .perst_reg = UINT64_C(0x814),
        .phy_ack_timeout_us = 50000,
        .link_up_timeout_us = PCIE_PERST_LINK_UP_TIMEOUT_US,
        .have_perst_gpio = true,
    };
    unsigned i = 0;
    int ret;

    pcie_perst_delays_from_adt(&cfg.delays, false, 0, false, 0);

    reset_fake();
    active_cfg = &cfg;
    fake.link_up_after_ltssm = true;

    ret = pcie_port_release_perst(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "release_perst succeeds without a port PHY");

    expect_event(i++,
                 (struct event){
                     .kind = EV_SET, .address = PORT1_BASE + PCIE_PORT_APPCLK, .a = PCIE_PORT_APPCLK_EN},
                 "APPCLK is still enabled first");
    expect_event(i++, (struct event){.kind = EV_PERST, .asserted = true},
                 "PERST# is still asserted");
    expect_event(i++, (struct event){.kind = EV_DELAY, .a = 100}, "Tperst-clk is still honoured");
    expect_event(i++,
                 (struct event){
                     .kind = EV_SET, .address = PORT1_BASE + 0x814, .a = PCIE_PORT_PERST_OFF},
                 "the pre-T602x PERST register offset is 0x814");
    expect_event(i++, (struct event){.kind = EV_PERST, .asserted = false},
                 "PERST# is released last");
    check_u32(fake.count, i, "no PHY accesses are emitted without a port PHY");
    if (fake.count != i)
        dump_trace();

    /* start_link must not touch a PHY that does not exist. */
    fake.count = 0;
    fake.linksts &= ~PCIE_PORT_LINKSTS_UP;
    ret = pcie_port_start_link(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_OK, "start_link succeeds without a port PHY");
    for (unsigned j = 0; j < fake.count; j++)
        check(fake.trace[j].a != PCIE_PHY_LANE_CFG_REFCLKCGEN,
              "REFCLKCGEN is not written without a port PHY");
}

static void test_link_down_is_reported(void)
{
    struct pcie_port_bringup cfg = t602x_port0(true);
    bool started = false;
    int ret;

    reset_fake();
    active_cfg = &cfg;
    fake.link_never_up = true;

    ret = pcie_port_start_link(&ops, NULL, &cfg);
    check(ret == PCIE_PORT_BRINGUP_ERR_LINK_DOWN, "a link that never trains is reported");

    for (unsigned i = 0; i < fake.count; i++) {
        if (fake.trace[i].kind == EV_POLL &&
            fake.trace[i].address == PORT0_BASE + PCIE_PORT_LINKSTS) {
            check_u32(fake.trace[i].timeout_us, PCIE_PERST_LINK_UP_TIMEOUT_US,
                      "the link poll is bounded at the Asahi default of 500ms");
            started = true;
        }
    }
    check(started, "the link is polled, not read once");
}

static void test_faults_are_propagated(void)
{
    struct pcie_port_bringup cfg = t602x_port0(true);

    reset_fake();
    active_cfg = &cfg;
    fake.refclk0_ack_stuck = true;
    check(pcie_port_release_perst(&ops, NULL, &cfg) == PCIE_PORT_BRINGUP_ERR_PHY_CLK0_ACK,
          "a stuck refclk 0 ack is reported");
    check(!fake.perst_ever_released, "PERST# stays asserted when the refclk fails");

    reset_fake();
    fake.refclk1_ack_stuck = true;
    check(pcie_port_release_perst(&ops, NULL, &cfg) == PCIE_PORT_BRINGUP_ERR_PHY_CLK1_ACK,
          "a stuck refclk 1 ack is reported");
    check(!fake.perst_ever_released, "PERST# stays asserted when the second refclk fails");

    reset_fake();
    fake.perst_assert_fails = true;
    check(pcie_port_release_perst(&ops, NULL, &cfg) == PCIE_PORT_BRINGUP_ERR_PERST_ASSERT,
          "a pad that cannot be asserted is reported");

    reset_fake();
    fake.perst_release_fails = true;
    check(pcie_port_release_perst(&ops, NULL, &cfg) == PCIE_PORT_BRINGUP_ERR_PERST_RELEASE_GPIO,
          "a pad that cannot be released is reported");
}

static void test_invalid_arguments(void)
{
    struct pcie_port_bringup cfg = t602x_port0(true);
    struct pcie_port_bringup_ops partial = ops;

    active_cfg = &cfg;
    reset_fake();

    check(pcie_port_release_perst(NULL, NULL, &cfg) == PCIE_PORT_BRINGUP_ERR_INVALID_ARGUMENT,
          "NULL ops is rejected");
    check(pcie_port_release_perst(&ops, NULL, NULL) == PCIE_PORT_BRINGUP_ERR_INVALID_ARGUMENT,
          "NULL cfg is rejected");
    check(pcie_port_start_link(NULL, NULL, &cfg) == PCIE_PORT_BRINGUP_ERR_INVALID_ARGUMENT,
          "NULL ops is rejected by start_link");

    /* Claiming a PERST# GPIO without supplying the callback must not proceed. */
    partial.perst_set = NULL;
    check(pcie_port_release_perst(&partial, NULL, &cfg) == PCIE_PORT_BRINGUP_ERR_INVALID_ARGUMENT,
          "have_perst_gpio without a perst_set callback is rejected");
    check_u32(fake.count, 0, "an invalid configuration touches nothing");

    /* Without the GPIO claim, the same partial table is fine. */
    cfg.have_perst_gpio = false;
    check(pcie_port_release_perst(&partial, NULL, &cfg) == PCIE_PORT_BRINGUP_OK,
          "a missing perst_set is fine when no GPIO is claimed");
}

/*
 * Unit conversion.  t-refclk-to-perst is microseconds, perst-to-config is
 * milliseconds; see the derivation in pcie.h.
 */
static void test_delay_units(void)
{
    struct pcie_perst_delays d;

    /* J414s: both properties read 100. */
    pcie_perst_delays_from_adt(&d, true, 100, true, 100);
    check_u32(d.refclk_to_perst_us, 100u, "t-refclk-to-perst = 100 means 100us");
    check_u32(d.perst_to_config_us, 100000u, "perst-to-config = 100 means 100ms");

    /* Absent properties fall back to what pcie-apple.c hardcodes. */
    pcie_perst_delays_from_adt(&d, false, 0, false, 0);
    check_u32(d.refclk_to_perst_us, PCIE_PERST_REFCLK_TO_PERST_MIN_US,
              "a missing t-refclk-to-perst falls back to 100us");
    check_u32(d.perst_to_config_us, PCIE_PERST_TO_CONFIG_MIN_US,
              "a missing perst-to-config falls back to 100ms");

    /* Larger declared values are honoured. */
    pcie_perst_delays_from_adt(&d, true, 250, true, 150);
    check_u32(d.refclk_to_perst_us, 250u, "a larger t-refclk-to-perst is honoured");
    check_u32(d.perst_to_config_us, 150000u, "a larger perst-to-config is honoured");

    /*
     * iMac21,1 pci-bridge1 (ASM3142 xHCI behind a power-enable GPIO) declares
     * t-refclk-to-perst = 200000, the largest value in any published Apple
     * Silicon ADT.  It must survive unclamped: this is the regression the cap
     * has to be chosen around, and it is also the strongest evidence that the
     * unit is microseconds (200 ms is plausible, 200 s is not).
     */
    pcie_perst_delays_from_adt(&d, true, 200000, true, 100);
    check_u32(d.refclk_to_perst_us, 200000u,
              "the iMac21,1 t-refclk-to-perst of 200000us is honoured, not truncated");
    check(200000u <= PCIE_PERST_REFCLK_TO_PERST_MAX_US,
          "the cap is above the largest published t-refclk-to-perst");

    /* Never go below the spec minimum, whatever the ADT claims. */
    pcie_perst_delays_from_adt(&d, true, 1, true, 1);
    check_u32(d.refclk_to_perst_us, PCIE_PERST_REFCLK_TO_PERST_MIN_US,
              "an under-spec t-refclk-to-perst is floored at 100us");
    check_u32(d.perst_to_config_us, PCIE_PERST_TO_CONFIG_MIN_US,
              "an under-spec perst-to-config is floored at 100ms");

    /* And never stall boot on a corrupt value. */
    pcie_perst_delays_from_adt(&d, true, UINT32_C(0xffffffff), true, UINT32_C(0xffffffff));
    check_u32(d.refclk_to_perst_us, PCIE_PERST_REFCLK_TO_PERST_MAX_US,
              "a corrupt t-refclk-to-perst is capped");
    check_u32(d.perst_to_config_us, PCIE_PERST_TO_CONFIG_MAX_US,
              "a corrupt perst-to-config is capped and does not overflow");

    /* Just under the overflow boundary of the millisecond scale. */
    pcie_perst_delays_from_adt(&d, true, 100, true, 1000);
    check_u32(d.perst_to_config_us, 1000000u, "1000ms scales to exactly the cap");
    pcie_perst_delays_from_adt(&d, true, 100, true, 1001);
    check_u32(d.perst_to_config_us, PCIE_PERST_TO_CONFIG_MAX_US, "1001ms is capped, not wrapped");

    /* A NULL output must not crash. */
    pcie_perst_delays_from_adt(NULL, true, 100, true, 100);
}

static void test_register_offsets(void)
{
    /* Cross-checked against pcie-apple.c's PORT_* / PORT_T602X_* definitions. */
    check_u32((u32)PCIE_PORT_LTSSMCTL, 0x80u, "PORT_LTSSMCTL is 0x80");
    check_u32(PCIE_PORT_LTSSMCTL_START, 0x1u, "PORT_LTSSMCTL_START is BIT(0)");
    check_u32((u32)PCIE_PORT_LINKSTS, 0x208u, "PORT_LINKSTS is 0x208");
    check_u32(PCIE_PORT_LINKSTS_UP, 0x1u, "PORT_LINKSTS_UP is BIT(0)");
    check_u32((u32)PCIE_PORT_APPCLK, 0x800u, "PORT_APPCLK is 0x800");
    check_u32(PCIE_PORT_APPCLK_EN, 0x1u, "PORT_APPCLK_EN is BIT(0)");
    check_u32(PCIE_PORT_PERST_OFF, 0x1u, "PORT_PERST_OFF is BIT(0)");
    check_u32(PCIE_PHY_LANE_CFG_REFCLK0REQ, 0x1u, "PHY_LANE_CFG_REFCLK0REQ is BIT(0)");
    check_u32(PCIE_PHY_LANE_CFG_REFCLK1REQ, 0x2u, "PHY_LANE_CFG_REFCLK1REQ is BIT(1)");
    check_u32(PCIE_PHY_LANE_CFG_REFCLK0ACK, 0x4u, "PHY_LANE_CFG_REFCLK0ACK is BIT(2)");
    check_u32(PCIE_PHY_LANE_CFG_REFCLK1ACK, 0x8u, "PHY_LANE_CFG_REFCLK1ACK is BIT(3)");
    check_u32(PCIE_PHY_LANE_CFG_REFCLKEN0 | PCIE_PHY_LANE_CFG_REFCLKEN1, 0x600u,
              "PHY_LANE_CFG_REFCLKEN is BIT(9) | BIT(10)");
    check_u32(PCIE_PHY_LANE_CFG_REFCLKCGEN, 0xc0000000u,
              "PHY_LANE_CFG_REFCLKCGEN is BIT(30) | BIT(31)");
}

int main(void)
{
    test_register_offsets();
    test_delay_units();
    test_t602x_full_sequence();
    test_ordering_invariants();
    test_no_perst_gpio();
    test_link_already_up_is_not_reset();
    test_t81xx_has_no_port_phy();
    test_link_down_is_reported();
    test_faults_are_propagated();
    test_invalid_arguments();

    check(!fake.overflow, "the trace buffer did not overflow");

    if (failures) {
        fprintf(stderr, "pcie-bringup: %d failure(s)\n", failures);
        return 1;
    }

    printf("pcie-bringup: PASS\n");
    return 0;
}
