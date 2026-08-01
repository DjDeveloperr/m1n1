/* SPDX-License-Identifier: MIT */
/*
 * Host tests for the EL2 half of the bulk channel (src/hv_xfer.c).
 *
 * These exist because everything here is guest-facing ABI or a guest-supplied
 * pointer, and both classes have cost this project boots before. Specifically:
 *
 *   * The doorbell register decode. `width` from hv_emulate_rw() is log2 of
 *     the access size, not the size -- getting that wrong makes every 32-bit
 *     MMIO access the guest driver issues land on the wrong register, and the
 *     symptom on hardware is an inscrutable zero.
 *   * xfer_validate_window(). This is the one place the guest hands EL2 a
 *     pointer. Every refusal is asserted: unaligned, unmapped, discontiguous,
 *     and -- the one that actually matters -- physically valid but pointing at
 *     MMIO rather than DRAM.
 *   * The range checks on GET/PUT, which are all that stand between a
 *     malformed length and a bulk write past the end of the window.
 *
 * The device is compiled into this translation unit so the static functions
 * are reachable; the m1n1 environment it needs is stubbed below.
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* u64 is `unsigned long` to match the aarch64-linux-gnu target, so the
 * device's %lx format strings type-check natively -- see src/hv_xfer.h. */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef unsigned long u64;
typedef int32_t s32;
typedef long s64;

#define BIT(_bit)         (UINT32_C(1) << (_bit))
#define GENMASK(h, l)     (((~0UL) << (l)) & (~0UL >> (63 - (h))))
#define PACKED            __attribute__((packed))
#define UNUSED(x)         (void)(x)
#define sysop(op)         __asm__ __volatile__("" ::: "memory")

/* ---- stubbed m1n1 environment ------------------------------------------ */

#define TEST_RAM_BASE 0x800000000ULL
#define TEST_RAM_SIZE 0x100000000ULL /* 4 GiB of "DRAM" */
#define TEST_PAGE     0x4000ULL

u64 ram_base = TEST_RAM_BASE;
u64 mem_size_actual = TEST_RAM_SIZE;

struct exc_info;

typedef enum { START_HV = 3 } uartproxy_boot_reason_t;
enum { HV_XFER = 9 };

typedef bool(hv_hook_t)(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width);

/* What the fake stage-2 table answers for a given IPA. */
static u64 (*stub_pt_walk)(u64 addr);
static u64 hv_pt_walk(u64 addr)
{
    return stub_pt_walk ? stub_pt_walk(addr) : 0;
}

static u64 sync_addr, sync_len;
static unsigned sync_calls;
static void dc_civac_range(void *addr, size_t length)
{
    sync_addr = (u64)addr;
    sync_len = length;
    sync_calls++;
}

static int stub_map_hw_rc = 0, stub_map_hook_rc = 0;
static u64 mapped_win, mapped_win_size, mapped_hook_base, unmapped_base;
static int hv_map_hw(u64 from, u64 to, u64 size)
{
    UNUSED(to);
    mapped_win = from;
    mapped_win_size = size;
    return stub_map_hw_rc;
}
static int hv_map_hook(u64 from, hv_hook_t *hook, u64 size)
{
    UNUSED(hook);
    UNUSED(size);
    mapped_hook_base = from;
    return stub_map_hook_rc;
}
static int hv_unmap(u64 from, u64 size)
{
    UNUSED(size);
    unmapped_base = from;
    return 0;
}

/* The host answer the next proxy event will produce. */
static s32 stub_host_status;
static u32 stub_host_result;
static unsigned host_calls;
static u32 seen_cmd, seen_tag, seen_off, seen_len;
static u64 seen_win, seen_win_size, seen_name_phys;

static void hv_exc_proxy(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type,
                         void *extra);

#define HV_XFER_HOST_TEST 1
#include "../../src/hv_xfer.c"

static void hv_exc_proxy(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type,
                         void *extra)
{
    struct hv_xfer_exc_info *info = extra;

    UNUSED(ctx);
    assert(reason == START_HV);
    assert(type == HV_XFER);
    /* EL2 must poison the status so a host that never answers reads as a
     * refusal, never as "succeeded and moved nothing". */
    assert(info->status == HV_XFER_ST_NODEV);

    host_calls++;
    seen_cmd = info->cmd;
    seen_tag = info->tag;
    seen_off = info->off;
    seen_len = info->len;
    seen_win = info->win_phys;
    seen_win_size = info->win_size;
    seen_name_phys = info->name_phys;

    info->status = stub_host_status;
    info->result = stub_host_result;
}

/* ---- helpers ------------------------------------------------------------ */

#define DOORBELL 0x200000000UL
#define WIN_SIZE 0x10000ULL

static u64 win_alloc;

static void reset_device(void)
{
    if (xfer_device) {
        free(xfer_device);
        xfer_device = NULL;
    }
    stub_pt_walk = NULL;
    stub_map_hw_rc = stub_map_hook_rc = 0;
    stub_host_status = HV_XFER_ST_OK;
    stub_host_result = 0;
    host_calls = sync_calls = 0;
    sync_addr = sync_len = 0;
}

static void map_default_device(void)
{
    reset_device();
    /* A window inside "DRAM", as the proxy heap would give. */
    win_alloc = TEST_RAM_BASE + 0x10000000ULL;
    assert(hv_map_xfer(DOORBELL, win_alloc, WIN_SIZE) == 0);
    assert(mapped_win == win_alloc && mapped_win_size == WIN_SIZE);
    assert(mapped_hook_base == DOORBELL);
}

/* width is log2(bytes) -- see hv_emulate_rw() in src/hv_vm.c. */
static u64 rd(u64 off, int width)
{
    u64 val = 0xdeadbeefdeadbeefULL;
    bool ok = handle_xfer(NULL, DOORBELL + off, &val, false, width);
    assert(ok);
    return val;
}

static void wr(u64 off, u64 val, int width)
{
    bool ok = handle_xfer(NULL, DOORBELL + off, &val, true, width);
    assert(ok);
}

static bool try_rd(u64 off, int width, u64 *out)
{
    *out = 0;
    return handle_xfer(NULL, DOORBELL + off, out, false, width);
}

#define W32 2
#define W64 3
#define W8  0

/* ---- tests -------------------------------------------------------------- */

static void test_identity_registers(void)
{
    map_default_device();

    assert(rd(HV_XFER_REG_ID, W32) == HV_XFER_ID);
    assert(rd(HV_XFER_REG_VERSION, W32) == HV_XFER_VERSION);
    assert(rd(HV_XFER_REG_PAGESIZE, W32) == HV_XFER_PAGE_SIZE);
    assert(rd(HV_XFER_REG_WIN_LO, W32) == (u32)win_alloc);
    assert(rd(HV_XFER_REG_WIN_HI, W32) == (u32)(win_alloc >> 32));
    assert(rd(HV_XFER_REG_WIN_SIZE, W32) == WIN_SIZE);
    assert(rd(HV_XFER_REG_MAX_XFER, W32) == WIN_SIZE);
    /* SET_WINDOW is advertised before any host has answered; HOST is not. */
    assert(rd(HV_XFER_REG_FEATURES, W32) == HV_XFER_FEAT_SET_WINDOW);

    /* A 64-bit read of an aligned pair composes both halves. */
    assert(rd(HV_XFER_REG_WIN_LO, W64) == win_alloc);

    /* An unimplemented register reads 0, never all-ones: all-ones is how a
     * MISSING device presents and a probing guest must be able to tell. */
    assert(rd(0x300, W32) == 0);

    printf("  identity registers            ok\n");
}

static void test_width_decoding(void)
{
    u64 v;

    map_default_device();

    /* width is log2(bytes). 4 bytes == width 2. If this were treated as a
     * byte count, offset 0 would decode as a 2-byte access and every guest
     * MMIO read would return the wrong half. */
    assert(rd(HV_XFER_REG_ID, W32) == HV_XFER_ID);

    /* 1- and 2-byte accesses to the register file are refused rather than
     * silently truncated. */
    assert(!try_rd(HV_XFER_REG_ID, 0, &v));
    assert(!try_rd(HV_XFER_REG_ID, 1, &v));
    /* ... and so is anything wider than 8 bytes. */
    assert(!try_rd(HV_XFER_REG_ID, 4, &v));

    /* Unaligned register access is refused. */
    assert(!try_rd(HV_XFER_REG_ID + 1, W32, &v));

    /* An access outside this device's page is not ours. */
    v = 0;
    assert(!handle_xfer(NULL, DOORBELL + HV_XFER_REGS_SIZE, &v, false, W32));

    printf("  access width decoding         ok\n");
}

static void test_name_field(void)
{
    map_default_device();

    /* The name field is a byte array and takes byte accesses. */
    const char *name = "AppleNvme.sys";
    for (size_t i = 0; i <= strlen(name); i++)
        wr(HV_XFER_REG_NAME + i, (u8)name[i], W8);

    assert(strcmp(xfer_device->name, name) == 0);
    assert(rd(HV_XFER_REG_NAME, W8) == (u8)'A');

    /* An 8-byte write lands little-endian, the way a guest memcpy would. */
    wr(HV_XFER_REG_NAME, 0x3837363534333231ULL, W64);
    assert(memcmp(xfer_device->name, "12345678", 8) == 0);

    /* The last legal byte works; one past it is refused, not wrapped. */
    u64 v;
    wr(HV_XFER_REG_NAME + HV_XFER_NAME_SIZE - 1, 0x5a, W8);
    assert(xfer_device->name[HV_XFER_NAME_SIZE - 1] == 0x5a);
    assert(!try_rd(HV_XFER_REG_NAME + HV_XFER_NAME_SIZE - 4, W64, &v));
    assert(!try_rd(HV_XFER_REG_NAME + HV_XFER_NAME_SIZE, W8, &v));

    printf("  name field                    ok\n");
}

static void test_command_round_trip(void)
{
    map_default_device();

    stub_host_status = HV_XFER_ST_OK;
    stub_host_result = 0x1234;

    wr(HV_XFER_REG_TAG, 7, W32);
    wr(HV_XFER_REG_OFF, 0x40, W32);
    wr(HV_XFER_REG_LEN, 0x100, W32);
    wr(HV_XFER_REG_CMD, HV_XFER_CMD_GET, W32);

    assert(host_calls == 1);
    assert(seen_cmd == HV_XFER_CMD_GET);
    assert(seen_tag == 7 && seen_off == 0x40 && seen_len == 0x100);
    assert(seen_win == win_alloc && seen_win_size == WIN_SIZE);
    assert(seen_name_phys == (u64)xfer_device->name);

    assert((s32)rd(HV_XFER_REG_STATUS, W32) == HV_XFER_ST_OK);
    assert(rd(HV_XFER_REG_RESULT, W32) == 0x1234);
    assert(rd(HV_XFER_REG_SEQ, W32) == 1);
    assert(rd(HV_XFER_REG_ERRORS, W32) == 0);
    /* Only after a host has actually answered is HOST advertised. */
    assert(rd(HV_XFER_REG_FEATURES, W32) & HV_XFER_FEAT_HOST);

    /* Cache maintenance covers what the host wrote, and runs after it. */
    assert(sync_calls == 1);
    assert(sync_addr == win_alloc + 0x40);
    assert(sync_len == 0x1234);

    printf("  command round trip            ok\n");
}

static void test_put_syncs_the_requested_length(void)
{
    map_default_device();

    stub_host_result = 0; /* a PUT reports what it consumed via status only */
    wr(HV_XFER_REG_OFF, 0, W32);
    wr(HV_XFER_REG_LEN, 0x200, W32);
    stub_host_result = 0x200;
    wr(HV_XFER_REG_CMD, HV_XFER_CMD_PUT, W32);

    assert(sync_calls == 1);
    assert(sync_addr == win_alloc && sync_len == 0x200);

    printf("  PUT cache maintenance         ok\n");
}

static void test_host_refusal_is_recorded(void)
{
    map_default_device();

    stub_host_status = HV_XFER_ST_IO;
    stub_host_result = 0xffff; /* must be discarded on failure */
    wr(HV_XFER_REG_LEN, 0x10, W32);
    wr(HV_XFER_REG_CMD, HV_XFER_CMD_GET, W32);

    assert((s32)rd(HV_XFER_REG_STATUS, W32) == HV_XFER_ST_IO);
    assert(rd(HV_XFER_REG_RESULT, W32) == 0);
    assert(rd(HV_XFER_REG_SEQ, W32) == 0);
    assert(rd(HV_XFER_REG_ERRORS, W32) == 1);
    assert(!(rd(HV_XFER_REG_FEATURES, W32) & HV_XFER_FEAT_HOST));

    printf("  host refusal accounting       ok\n");
}

static void test_range_checks(void)
{
    struct {
        u32 off, len;
    } bad[] = {
        {0, 0},                        /* zero length */
        {WIN_SIZE, 0x10},              /* offset at the end */
        {WIN_SIZE + 0x10, 0x10},       /* offset past the end */
        {0, WIN_SIZE + 1},             /* longer than the window */
        {WIN_SIZE - 8, 0x10},          /* straddles the end */
        {0xffffffff, 0x10},            /* offset overflow */
        {0x10, 0xffffffff},            /* length overflow */
    };

    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        map_default_device();
        wr(HV_XFER_REG_OFF, bad[i].off, W32);
        wr(HV_XFER_REG_LEN, bad[i].len, W32);
        wr(HV_XFER_REG_CMD, HV_XFER_CMD_GET, W32);
        assert((s32)rd(HV_XFER_REG_STATUS, W32) == HV_XFER_ST_RANGE);
        /* Refused before the host was ever told about it. */
        assert(host_calls == 0);
        assert(sync_calls == 0);
        assert(rd(HV_XFER_REG_ERRORS, W32) == 1);
    }

    printf("  GET/PUT range refusals        ok\n");
}

static void test_unknown_command(void)
{
    map_default_device();
    wr(HV_XFER_REG_CMD, 0x4242, W32);
    assert((s32)rd(HV_XFER_REG_STATUS, W32) == HV_XFER_ST_BADCMD);
    assert(host_calls == 0);
    printf("  unknown command               ok\n");
}

static void test_wide_write_to_cmd_is_refused(void)
{
    map_default_device();

    /* A 64-bit store covering CMD would also cover OFF, and there is no
     * ordering in which both "OFF was set first" and "this is one access" are
     * true. It must refuse, not guess. */
    wr(HV_XFER_REG_CMD, ((u64)0x1000 << 32) | HV_XFER_CMD_PING, W64);
    assert(host_calls == 0);
    assert((s32)rd(HV_XFER_REG_STATUS, W32) == HV_XFER_ST_INVAL);

    /* The paired 64-bit store to OFF/LEN, which does NOT include CMD, works
     * and lands in ascending order. */
    wr(HV_XFER_REG_OFF, ((u64)0x222 << 32) | 0x111, W64);
    assert(rd(HV_XFER_REG_OFF, W32) == 0x111);
    assert(rd(HV_XFER_REG_LEN, W32) == 0x222);

    printf("  wide CMD store refused        ok\n");
}

/* --- guest-supplied windows --------------------------------------------- */

#define PTE_VALID BIT(0)

static u64 guest_win_ipa;
static u64 guest_win_pa;
static u64 guest_win_size;
static bool guest_win_hole;      /* one page unmapped */
static bool guest_win_scramble;  /* one page discontiguous */

static u64 fake_pt_walk(u64 addr)
{
    if (addr < guest_win_ipa || addr >= guest_win_ipa + guest_win_size)
        return 0;
    u64 off = addr - guest_win_ipa;
    if (guest_win_hole && off == TEST_PAGE)
        return 0;
    u64 pa = guest_win_pa + off;
    if (guest_win_scramble && off == TEST_PAGE)
        pa += TEST_PAGE;
    return pa | PTE_VALID;
}

static void set_uwin(u64 ipa, u64 size)
{
    wr(HV_XFER_REG_UWIN_LO, (u32)ipa, W32);
    wr(HV_XFER_REG_UWIN_HI, (u32)(ipa >> 32), W32);
    wr(HV_XFER_REG_UWIN_SIZE, (u32)size, W32);
}

static s32 do_set_window(u64 ipa, u64 pa, u64 size)
{
    guest_win_ipa = ipa;
    guest_win_pa = pa;
    guest_win_size = size;
    stub_pt_walk = fake_pt_walk;
    set_uwin(ipa, size);
    wr(HV_XFER_REG_CMD, HV_XFER_CMD_SET_WINDOW, W32);
    return (s32)rd(HV_XFER_REG_STATUS, W32);
}

static void test_set_window_accepts_a_valid_guest_buffer(void)
{
    map_default_device();
    guest_win_hole = guest_win_scramble = false;

    u64 ipa = TEST_RAM_BASE + 0x40000000ULL;
    u64 pa = ipa; /* guest RAM is identity-mapped */
    assert(do_set_window(ipa, pa, 0x20000) == HV_XFER_ST_OK);

    /* The guest now sees ITS address in WIN_*, and transfers aim at the
     * translated physical one. */
    assert(rd(HV_XFER_REG_WIN_LO, W64) == ipa);
    assert(rd(HV_XFER_REG_WIN_SIZE, W32) == 0x20000);

    wr(HV_XFER_REG_OFF, 0, W32);
    wr(HV_XFER_REG_LEN, 0x1000, W32);
    stub_host_result = 0x1000;
    wr(HV_XFER_REG_CMD, HV_XFER_CMD_GET, W32);
    assert(seen_win == pa);
    assert(sync_addr == pa);

    /* RESET_WINDOW puts the m1n1-owned window back. */
    wr(HV_XFER_REG_CMD, HV_XFER_CMD_RESET_WINDOW, W32);
    assert(rd(HV_XFER_REG_WIN_LO, W64) == win_alloc);
    assert(rd(HV_XFER_REG_WIN_SIZE, W32) == WIN_SIZE);

    printf("  SET_WINDOW accept + reset     ok\n");
}

static void test_set_window_refusals(void)
{
    u64 good = TEST_RAM_BASE + 0x40000000ULL;

    /* Unaligned base. */
    map_default_device();
    guest_win_hole = guest_win_scramble = false;
    assert(do_set_window(good + 4, good + 4, 0x8000) == HV_XFER_ST_FAULT);

    /* Unaligned size. */
    map_default_device();
    assert(do_set_window(good, good, 0x8004) == HV_XFER_ST_FAULT);

    /* Zero size. */
    map_default_device();
    assert(do_set_window(good, good, 0) == HV_XFER_ST_FAULT);

    /* A hole in the middle: one page not mapped into the guest at all. */
    map_default_device();
    guest_win_hole = true;
    assert(do_set_window(good, good, 0x10000) == HV_XFER_ST_FAULT);
    guest_win_hole = false;

    /* Mapped, but not physically contiguous. */
    map_default_device();
    guest_win_scramble = true;
    assert(do_set_window(good, good, 0x10000) == HV_XFER_ST_FAULT);
    guest_win_scramble = false;

    /* THE ONE THAT MATTERS: perfectly mapped, perfectly contiguous, but
     * pointing at MMIO instead of DRAM. /arm-io is mapped HW too, so
     * contiguity alone would let a guest aim a bulk write at a device. */
    map_default_device();
    assert(do_set_window(0x230000000UL, 0x230000000UL, 0x8000) == HV_XFER_ST_FAULT);

    /* Just past the top of DRAM. */
    map_default_device();
    assert(do_set_window(TEST_RAM_BASE + TEST_RAM_SIZE - 0x4000,
                         TEST_RAM_BASE + TEST_RAM_SIZE - 0x4000, 0x8000) == HV_XFER_ST_FAULT);

    /* Larger than the hard cap. */
    map_default_device();
    assert(do_set_window(good, good, HV_XFER_MAX_WINDOW + TEST_PAGE) == HV_XFER_ST_FAULT);

    /* After every refusal the window is still the m1n1-owned one. */
    assert(rd(HV_XFER_REG_WIN_LO, W64) == win_alloc);

    printf("  SET_WINDOW refusals           ok\n");
}

static void test_map_refusals(void)
{
    u64 win = TEST_RAM_BASE + 0x10000000ULL;

    reset_device();
    assert(hv_map_xfer(DOORBELL + 4, win, WIN_SIZE) < 0); /* base unaligned */
    reset_device();
    assert(hv_map_xfer(DOORBELL, win + 4, WIN_SIZE) < 0); /* window unaligned */
    reset_device();
    assert(hv_map_xfer(DOORBELL, win, 0) < 0); /* empty window */
    reset_device();
    assert(hv_map_xfer(DOORBELL, win, WIN_SIZE + 4) < 0); /* size unaligned */
    reset_device();
    assert(hv_map_xfer(DOORBELL, win, HV_XFER_MAX_WINDOW + TEST_PAGE) < 0);
    reset_device();
    assert(hv_map_xfer(DOORBELL, TEST_RAM_BASE - TEST_PAGE, TEST_PAGE) < 0); /* below DRAM */
    reset_device();
    assert(hv_map_xfer(DOORBELL, TEST_RAM_BASE + TEST_RAM_SIZE, TEST_PAGE) < 0); /* above DRAM */

    /* A failed doorbell map unwinds the window mapping instead of leaving the
     * guest a window with no way to ring for it. */
    reset_device();
    stub_map_hook_rc = -1;
    unmapped_base = 0;
    assert(hv_map_xfer(DOORBELL, win, WIN_SIZE) < 0);
    assert(unmapped_base == win);
    assert(xfer_device == NULL);

    /* A failed window map never reaches the doorbell. */
    reset_device();
    stub_map_hw_rc = -1;
    mapped_hook_base = 0;
    assert(hv_map_xfer(DOORBELL, win, WIN_SIZE) < 0);
    assert(mapped_hook_base == 0);
    assert(xfer_device == NULL);

    /* Only one channel at a time. */
    map_default_device();
    assert(hv_map_xfer(DOORBELL + HV_XFER_REGS_SIZE, win, WIN_SIZE) < 0);

    printf("  hv_map_xfer refusals          ok\n");
}

static void test_no_device_is_inert(void)
{
    u64 val = 0;

    reset_device();
    /* With no device mapped, the hook claims nothing -- the fault falls
     * through to the normal unmapped-IPA path rather than answering with
     * plausible-looking zeroes. */
    assert(!handle_xfer(NULL, DOORBELL, &val, false, W32));

    printf("  unmapped device is inert      ok\n");
}

int main(void)
{
    printf("hv_xfer device tests\n");

    test_identity_registers();
    test_width_decoding();
    test_name_field();
    test_command_round_trip();
    test_put_syncs_the_requested_length();
    test_host_refusal_is_recorded();
    test_range_checks();
    test_unknown_command();
    test_wide_write_to_cmd_is_refused();
    test_set_window_accepts_a_valid_guest_buffer();
    test_set_window_refusals();
    test_map_refusals();
    test_no_device_is_inert();

    reset_device();
    printf("all hv_xfer device tests passed\n");
    return 0;
}
