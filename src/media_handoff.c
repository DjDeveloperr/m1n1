/* SPDX-License-Identifier: MIT */

/*
 * J414s Windows media profile preboot contract (MCA audio, AOP microphones,
 * ISP camera).
 *
 * WHY THIS FILE DOES ALMOST NOTHING TO THE HARDWARE
 * ------------------------------------------------
 * The wireless handoff (src/wireless_handoff.c) has to install a DART domain at
 * EL2 because pci.sys enables endpoint bus mastering before any KMDF provider
 * can run: there is a window in which the device can DMA and no driver exists.
 * The MTP handoff (src/mtp_handoff.c) has to boot the IOP because AppleMtpHid
 * takes over a transport that must already be advertising its endpoints.
 *
 * None of the three media devices has either property, and each Windows driver
 * explicitly owns the mutation the other two handoffs perform in firmware:
 *
 *   - AppleMcaAudio raises its own PMGR domains through its pmgr_east _CRS
 *     window (ps_sio, ps_audio_p, ps_sio_adma, ps_i2c1, ps_i2c3, then ps_mca0/1/2
 *     per cluster AFTER the NCO channel runs; never ps_mca3), and installs its
 *     own translating dart-sio SID-2 domain, deliberately refusing to fall back
 *     to bypass.
 *   - AppleAopAudio never writes dart-aop at all.  It reads streams 0 (AOP) and
 *     10 (AOP ADMAC) and arms capture only if both are BYPASS_DART with
 *     TRANSLATE_ENABLE clear.  It also decides at runtime whether the AOP core
 *     is already released or must be cold-booted.
 *   - AppleIsp raises its own ps_isp_* chain, and ADOPTS dart-isp0's inherited
 *     TTBR rather than installing one, because installing a fresh L1 would
 *     unmap the coprocessor's own text with no recovery short of a power cycle.
 *
 * So the useful preboot work is not mutation, it is EVIDENCE: three of the
 * drivers' load-bearing preconditions are only checkable from EL2, before the
 * guest exists, and are currently ASSUMED.  This file checks them and prints
 * them.  With flags == 0 it performs no register write of any kind.
 *
 * WHAT IT DELIBERATELY DOES NOT DO
 * --------------------------------
 *   - No interrupt work.  The media profile publishes zero new GSIVs and makes
 *     no CSRT change; every driver reaches first light by polling.  Nothing here
 *     touches AIC masking.
 *   - No writes to dart-sio, dart-aop or dart-isp0.  In particular the ISP's
 *     inherited page table must survive, and power-gating a DART discards it.
 *   - No power-domain lowering, ever, for the same reason.
 *   - Nothing that could clock, unmute or power a speaker amplifier.  The only
 *     write this file can perform is the MCA clock-mux select, which chooses
 *     which NCO feeds a cluster.  It does not set MCLK_EN, does not raise any
 *     ps_mcaN, does not touch the mca-switch serialisers, and does not talk to
 *     i2c1/i2c3 where the SN012776 amplifiers live.
 *   - Nothing to USB, ANS/NVMe, PCIe or their DARTs.  Every address this file
 *     can touch is validated against a pinned J414s literal first, so a layout
 *     change is a refusal rather than a stray write.
 */

#include "../config.h"

/*
 * This include list is itself part of the contract, and
 * tests/python/test_media_handoff_contract.py pins it verbatim.  dart.h,
 * dapf.h and aic.h are absent because nothing here may program a DART, a DAPF
 * filter, or an interrupt; usb.h, pcie.h and nvme.h are absent because nothing
 * here may go near the boot path.
 */
#include "adt.h"
#include "media_handoff.h"
#include "platform_identity.h"
#include "pmgr.h"
#include "types.h"
#include "utils.h"
#include "xnuboot.h"

#if defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && defined(ENABLE_J414S_WINDOWS_MEDIA_HANDOFF)

#define MCA_SWITCH_PATH  "/arm-io/mca-switch"
#define NCO_PATH         "/arm-io/nco"
#define ADMAC_SIO_PATH   "/arm-io/admac-sio"
#define DART_SIO_PATH    "/arm-io/dart-sio"
#define AOP_PATH         "/arm-io/aop"
#define DART_AOP_PATH    "/arm-io/dart-aop"
#define ADMAC_AOP_PATH   "/arm-io/admac-aop-audio"
#define ISP0_PATH        "/arm-io/isp0"
#define DART_ISP0_PATH   "/arm-io/dart-isp0"

/*
 * Pinned J414s/T6020 physical layout.
 *
 * Every one of these was read out of the pinned live ADT capture
 * (sha256 93d96b4a...), not out of a datasheet or a header.  They are checked
 * before anything is read or written, for exactly the reason mtp_handoff.c
 * pins its own map: the Windows ACPI _CRS resources publish these same
 * addresses, so a machine whose ADT says something else needs a deliberate
 * port, not a best-effort probe against whatever block happens to be there.
 */
#define J414S_MCA_SWITCH_BASE   0x39b600000ULL
#define J414S_MCA_SWITCH_SIZE   0x10000ULL
#define J414S_MCA_CLUSTER_BASE  0x39b500000ULL
#define J414S_MCA_CLUSTER_SIZE  0x20000ULL
#define J414S_MCA_CLKMUX_BASE   0x28e03807cULL
#define J414S_MCA_CLKMUX_SIZE   0x18ULL
#define J414S_NCO_BASE          0x28e03c000ULL
#define J414S_NCO_SIZE          0x14000ULL
#define J414S_ADMAC_SIO_BASE    0x39b400000ULL
#define J414S_ADMAC_SIO_SIZE    0x34000ULL
#define J414S_DART_SIO_BASE     0x39b008000ULL
#define J414S_DART_SIO_SIZE     0x4000ULL
#define J414S_AOP_ASC_BASE      0x2a6400000ULL
#define J414S_AOP_ASC_SIZE      0x6c000ULL
#define J414S_AOP_MBOX_BASE     0x2a6050000ULL
#define J414S_AOP_MBOX_SIZE     0x4000ULL
#define J414S_DART_AOP_BASE     0x2a6808000ULL
#define J414S_DART_AOP_SIZE     0x4000ULL
#define J414S_ADMAC_AOP_BASE    0x2a6980000ULL
#define J414S_ADMAC_AOP_SIZE    0x34000ULL
#define J414S_ISP_COPROC_BASE   0x384000000ULL
#define J414S_ISP_COPROC_SIZE   0x4000000ULL
#define J414S_ISP_PMGR_BASE     0x290280000ULL
#define J414S_ISP_PMGR_SIZE     0x4034ULL
#define J414S_DART_ISP0_BASE    0x3860e8000ULL
#define J414S_DART_ISP0_SIZE    0x4000ULL

/*
 * AOP SRAM window -- a literal, and it MUST stay a literal.
 *
 * /arm-io/aop reg[2] holds the raw value 0x2a6c00000, which is already the
 * absolute physical address (the AOP's __TEXT segment-range starts there).  It
 * is ALSO a legal bus address inside /arm-io's first range (parent 0x200000000
 * <- bus 0x0, size 0x400000000), so adt_get_reg() dutifully "translates" it to
 * 0x4a6c00000 -- an address that is not the AOP and not anything else.  reg[0]
 * and reg[1] on the same node are ordinary bus addresses and translate
 * correctly, so the array cannot be trusted uniformly.
 *
 * Verified offline against the pinned ADT: segment-ranges[0].phys == 0x2a6c00000
 * with remap == phys, and segment-ranges[1] continues at 0x2a6c8b000; the two
 * on-chip segments together end at 0x2a6d2e000, inside +0x250000.  This mirrors
 * /arm-io/mtp, whose fixed IOP buffer window at 0x2a9c00000 is likewise pinned
 * rather than derived (see J414S_MTP_FIXED_BUFFER_BASE in src/mtp_handoff.c).
 */
#define J414S_AOP_SRAM_BASE 0x2a6c00000ULL
#define J414S_AOP_SRAM_SIZE 0x250000ULL

/* t8110 DART registers.  Same offsets src/dart.c uses; it does not export them. */
#define DART_T8110_PARAMS1           0x000
#define DART_T8110_PARAMS1_LOG2_PAGE GENMASK(27, 24)
#define DART_T8110_PARAMS3           0x008
#define DART_T8110_PARAMS3_PA_WIDTH  GENMASK(29, 24)
#define DART_T8110_PARAMS4           0x00c
#define DART_T8110_PARAMS4_SID_COUNT GENMASK(8, 0)
#define DART_T8110_ERROR             0x100
#define DART_T8110_ERROR_FLAG        BIT(31)
#define DART_T8110_ERROR_STREAMS     0x1c0
#define DART_T8110_PROTECT           0x200
#define DART_T8110_PROTECT_TTBR_TCR  BIT(0)
#define DART_T8110_ENABLE_STREAMS    0xc00
#define DART_T8110_TCR(sid)          (0x1000 + 4 * (sid))
#define DART_T8110_TCR_TRANSLATE     BIT(0)
#define DART_T8110_TCR_BYPASS_DART   BIT(1)
#define DART_T8110_TCR_BYPASS_DAPF   BIT(2)
#define DART_T8110_TTBR(sid)         (0x1400 + 4 * (sid))
#define DART_T8110_TTBR_VALID        BIT(0)
#define DART_T8110_TTBR_ADDR         GENMASK(29, 2)
#define DART_T8110_TTBR_SHIFT        14

/* Streams the Windows drivers care about, from the ADT iommu-mapper reg values. */
#define SIO_DART_SID_ADMAC 2  /* /arm-io/dart-sio/mapper-admac      reg = 2  */
#define AOP_DART_SID_CORE  0  /* /arm-io/dart-aop/mapper-aop        reg = 0  */
#define AOP_DART_SID_ADMAC 10 /* /arm-io/dart-aop/mapper-aop-admac  reg = 10 */
#define ISP_DART_SID_MAIN  0  /* /arm-io/dart-isp0/mapper-isp0      reg = 0  */

/* MCA clock mux encoding, identical to clk_set_mca_muxes() in src/clk.c. */
#define MCA_CLK_MUX      GENMASK(27, 24)
#define MCA_CLK_NCO_BASE 5
#define MCA_CLK_NUM_NCOS 5

enum media_handoff_error {
    MEDIA_HANDOFF_OK = 0,
    MEDIA_ERR_IDENTITY = -1,
    MEDIA_ERR_FLAGS = -2,
    MEDIA_ERR_ADT = -3,
    MEDIA_ERR_LAYOUT = -4,
    MEDIA_ERR_SEGMENTS = -5,
    MEDIA_ERR_ISP_CARVEOUT = -6,
    MEDIA_ERR_DART_POWER = -7,
    MEDIA_ERR_MCA_MUX_READBACK = -8,
};

struct media_reg_expect {
    const char *path;
    u32 index;
    u64 base;
    u64 size;
    const char *what;
};

/*
 * The complete set of physical windows this file is allowed to observe.  Note
 * what is NOT here: i2c1/i2c2/i2c3 (the amplifier and jack-codec buses) are
 * validated only for presence below, never opened, because reading a live I2C
 * transaction register is a mutation of the bus state machine.
 */
static const struct media_reg_expect media_expected_regs[] = {
    {MCA_SWITCH_PATH, 0, J414S_MCA_SWITCH_BASE, J414S_MCA_SWITCH_SIZE, "mca-switch i2s/tdm"},
    {MCA_SWITCH_PATH, 1, J414S_MCA_CLUSTER_BASE, J414S_MCA_CLUSTER_SIZE, "mca-switch clusters"},
    {MCA_SWITCH_PATH, 2, J414S_MCA_CLKMUX_BASE, J414S_MCA_CLKMUX_SIZE, "mca clock muxes"},
    {NCO_PATH, 0, J414S_NCO_BASE, J414S_NCO_SIZE, "nco"},
    {ADMAC_SIO_PATH, 0, J414S_ADMAC_SIO_BASE, J414S_ADMAC_SIO_SIZE, "admac-sio"},
    {DART_SIO_PATH, 0, J414S_DART_SIO_BASE, J414S_DART_SIO_SIZE, "dart-sio"},
    {AOP_PATH, 0, J414S_AOP_ASC_BASE, J414S_AOP_ASC_SIZE, "aop asc"},
    {AOP_PATH, 1, J414S_AOP_MBOX_BASE, J414S_AOP_MBOX_SIZE, "aop mailbox"},
    {DART_AOP_PATH, 0, J414S_DART_AOP_BASE, J414S_DART_AOP_SIZE, "dart-aop"},
    {ADMAC_AOP_PATH, 0, J414S_ADMAC_AOP_BASE, J414S_ADMAC_AOP_SIZE, "aop admac"},
    {ISP0_PATH, 0, J414S_ISP_COPROC_BASE, J414S_ISP_COPROC_SIZE, "isp coproc"},
    {ISP0_PATH, 1, J414S_ISP_PMGR_BASE, J414S_ISP_PMGR_SIZE, "isp pmgr_east"},
    {DART_ISP0_PATH, 0, J414S_DART_ISP0_BASE, J414S_DART_ISP0_SIZE, "dart-isp0 DARTLLT"},
};

static u64 media_mca_clkmux_base;

static int media_check_reg(const struct media_reg_expect *expect)
{
    int path[8];
    u64 base = 0;
    u64 size = 0;

    if (adt_path_offset_trace(adt, expect->path, path) < 0) {
        printf("media-handoff: missing ADT node %s\n", expect->path);
        return MEDIA_ERR_ADT;
    }
    if (adt_get_reg(adt, path, "reg", expect->index, &base, &size) < 0) {
        printf("media-handoff: %s has no reg[%u]\n", expect->path, expect->index);
        return MEDIA_ERR_ADT;
    }
    if (base != expect->base || size < expect->size) {
        printf("media-handoff: %s reg[%u] is %#lx/+%#lx, expected %#lx/+%#lx (%s)\n", expect->path,
               expect->index, base, size, expect->base, expect->size, expect->what);
        return MEDIA_ERR_LAYOUT;
    }

    printf("media-handoff: %-24s reg[%u] %#011lx +%#lx  %s\n", expect->path, expect->index, base,
           size, expect->what);
    return MEDIA_HANDOFF_OK;
}

static int media_check_layout(void)
{
    static const char *const presence_only[] = {"/arm-io/i2c1", "/arm-io/i2c2", "/arm-io/i2c3",
                                                "/arm-io/mca0", "/arm-io/mca2", NULL};

    for (size_t i = 0; i < ARRAY_SIZE(media_expected_regs); i++) {
        int status = media_check_reg(&media_expected_regs[i]);
        if (status)
            return status;
        if (media_expected_regs[i].base == J414S_MCA_CLKMUX_BASE)
            media_mca_clkmux_base = media_expected_regs[i].base;
    }

    for (const char *const *path = presence_only; *path; path++) {
        if (adt_path_offset(adt, *path) < 0) {
            printf("media-handoff: missing ADT node %s\n", *path);
            return MEDIA_ERR_ADT;
        }
    }

    return MEDIA_HANDOFF_OK;
}

static bool media_ranges_overlap(u64 a_base, u64 a_size, u64 b_base, u64 b_size)
{
    if (!a_size || !b_size)
        return false;
    return a_base < b_base + b_size && b_base < a_base + a_size;
}

/*
 * Fold an ASC nub's "segment-ranges" into one [lo, hi) extent, counting only
 * segments that live in DRAM (i.e. at or above the DRAM base).  On-chip SRAM
 * segments -- the AOP's __TEXT/__DATA at 0x2a6c00000 -- are not memory anybody
 * can allocate and must not widen the extent.
 */
static int media_segment_extent(const char *path, u64 dram_base, u64 *lo_out, u64 *hi_out,
                                bool verbose)
{
    const struct adt_segment_ranges *seg;
    u32 length = 0;
    int node = adt_path_offset(adt, path);
    u64 lo = ~0ULL;
    u64 hi = 0;

    if (node < 0)
        return MEDIA_ERR_ADT;

    seg = adt_getprop(adt, node, "segment-ranges", &length);
    if (!seg || !length || (length % sizeof(*seg)) != 0) {
        printf("media-handoff: %s has no parseable segment-ranges (len %u)\n", path, length);
        return MEDIA_ERR_SEGMENTS;
    }

    for (u32 i = 0; i < length / sizeof(*seg); i++) {
        bool in_dram = seg[i].phys >= dram_base;

        if (verbose)
            printf("media-handoff:   %s seg%u phys=%#lx iova=%#lx remap=%#lx size=%#x %s\n", path,
                   i, seg[i].phys, seg[i].iova, seg[i].remap, seg[i].size,
                   in_dram ? "DRAM" : "SRAM");

        if (!in_dram)
            continue;
        if (seg[i].phys < lo)
            lo = seg[i].phys;
        if (seg[i].phys + seg[i].size > hi)
            hi = seg[i].phys + seg[i].size;
    }

    if (hi == 0)
        return MEDIA_ERR_SEGMENTS;

    *lo_out = lo;
    *hi_out = hi;
    return MEDIA_HANDOFF_OK;
}

/*
 * The one invariant only m1n1 can prove.
 *
 * iBoot places the ISP firmware image in DRAM and marks the node pre-loaded;
 * AppleIsp resets and starts the core against that image and never reloads it.
 * The image therefore has to survive both m1n1 and Windows, and the only reason
 * it currently does is a layout accident: it ends below boot_args.phys_base,
 * so it was never inside anybody's allocatable window in the first place.
 *
 * docs/j414s-camera-isp-bringup.md calls that margin a live invariant rather
 * than a build constant, and it is right to: nothing enforces it.  m1n1 is the
 * only component that sees both the ADT segment-ranges and the real boot_args,
 * before any of the memory has been handed out -- the same argument that puts
 * wlan_range_is_claimed() here rather than in Mu.
 *
 * The test is deliberately against m1n1's OWN window and not the guest's.  The
 * hypervisor hands the guest phys_base = m1n1's heap_top, so the guest window
 * is a strict subset of this one; proving the carveout is outside m1n1's window
 * proves it is outside the guest's, and additionally catches the case where
 * m1n1's own allocator could scribble on it.
 */
static int media_check_isp_carveout(void)
{
    u64 dram_base = ALIGN_DOWN(cur_boot_args.phys_base, BIT(32));
    u64 window_base = cur_boot_args.phys_base;
    u64 window_size = cur_boot_args.mem_size;
    u64 lo = 0;
    u64 hi = 0;
    int status = media_segment_extent(ISP0_PATH, dram_base, &lo, &hi, true);

    if (status) {
        printf("media-handoff: cannot bound the ISP firmware carveout; refusing\n");
        return MEDIA_ERR_ISP_CARVEOUT;
    }

    if (media_ranges_overlap(lo, hi - lo, window_base, window_size)) {
        printf("media-handoff: ISP firmware carveout %#lx..%#lx OVERLAPS allocatable DRAM "
               "%#lx..%#lx; the camera image would be destroyed\n",
               lo, hi, window_base, window_base + window_size);
        return MEDIA_ERR_ISP_CARVEOUT;
    }

    printf("media-handoff: ISP firmware carveout %#lx..%#lx (%#lx bytes) is %#lx below "
           "boot_args.phys_base %#lx -- outside every allocatable byte\n",
           lo, hi, hi - lo, window_base - hi, window_base);
    return MEDIA_HANDOFF_OK;
}

/*
 * Same audit for the AOP, reported and not enforced.
 *
 * Two of the AOP's four segments are in DRAM (__ETEXT and __OS_LOG) while its
 * text and data are in on-chip SRAM.  The AOP is running, so anything it was
 * told to write to a DRAM segment is live DMA into memory m1n1 believes it
 * owns.  This has evidently been survivable -- it is the state every boot on
 * this machine has already run in -- so failing the whole handoff over it would
 * be a regression, not a fix.  It is printed with exact numbers so the next
 * boot log settles whether the region needs excluding.
 */
static void media_audit_aop_segments(void)
{
    u64 dram_base = ALIGN_DOWN(cur_boot_args.phys_base, BIT(32));
    u64 lo = 0;
    u64 hi = 0;

    if (media_segment_extent(AOP_PATH, dram_base, &lo, &hi, true)) {
        printf("media-handoff: AOP has no DRAM segments\n");
        return;
    }

    if (media_ranges_overlap(lo, hi - lo, cur_boot_args.phys_base, cur_boot_args.mem_size))
        printf("media-handoff: WARNING: AOP DRAM segments %#lx..%#lx fall INSIDE allocatable "
               "DRAM %#lx..%#lx; a running AOP writes there\n",
               lo, hi, cur_boot_args.phys_base, cur_boot_args.phys_base + cur_boot_args.mem_size);
    else
        printf("media-handoff: AOP DRAM segments %#lx..%#lx are outside allocatable DRAM\n", lo,
               hi);
}

static void media_dart_report_stream(const char *label, u64 regs, u32 sid)
{
    u32 tcr = read32(regs + DART_T8110_TCR(sid));
    u32 ttbr = read32(regs + DART_T8110_TTBR(sid));
    u32 enabled = read32(regs + DART_T8110_ENABLE_STREAMS + 4 * (sid >> 5));
    bool stream_on = !!(enabled & BIT(sid & 0x1f));
    u64 l1 = ((u64)FIELD_GET(DART_T8110_TTBR_ADDR, ttbr)) << DART_T8110_TTBR_SHIFT;

    printf("media-handoff:   %s sid %-2u tcr=%#010x%s%s%s ttbr=%#010x%s l1=%#lx streams=%#010x "
           "(%s)\n",
           label, sid, tcr, (tcr & DART_T8110_TCR_TRANSLATE) ? " TRANSLATE" : "",
           (tcr & DART_T8110_TCR_BYPASS_DART) ? " BYPASS_DART" : "",
           (tcr & DART_T8110_TCR_BYPASS_DAPF) ? " BYPASS_DAPF" : "", ttbr,
           (ttbr & DART_T8110_TTBR_VALID) ? " VALID" : "",
           (ttbr & DART_T8110_TTBR_VALID) ? l1 : 0, enabled,
           stream_on ? "enabled" : "disabled");
}

static void media_dart_report(const char *label, u64 regs)
{
    u32 params1 = read32(regs + DART_T8110_PARAMS1);
    u32 params3 = read32(regs + DART_T8110_PARAMS3);
    u32 params4 = read32(regs + DART_T8110_PARAMS4);
    u32 protect = read32(regs + DART_T8110_PROTECT);
    u32 error = read32(regs + DART_T8110_ERROR);
    u32 error_streams = read32(regs + DART_T8110_ERROR_STREAMS);

    printf("media-handoff: %s @ %#lx page=2^%lu pa_width=%lu sids=%lu protect=%#x%s "
           "error=%#x%s streams=%#x\n",
           label, regs, FIELD_GET(DART_T8110_PARAMS1_LOG2_PAGE, params1),
           FIELD_GET(DART_T8110_PARAMS3_PA_WIDTH, params3),
           FIELD_GET(DART_T8110_PARAMS4_SID_COUNT, params4), protect,
           (protect & DART_T8110_PROTECT_TTBR_TCR) ? " LOCKED" : "", error,
           (error & DART_T8110_ERROR_FLAG) ? " FAULT" : "", error_streams);
}

/*
 * dart-aop: the one measurement that turns AppleAopAudio's largest documented
 * assumption into a fact.
 *
 * The driver arms PDM capture only if BOTH stream 0 (the AOP core) and stream
 * 10 (the AOP ADMAC) read as BYPASS_DART with TRANSLATE_ENABLE clear, because
 * it treats DVA == PA and never programs a page table.  Whether that is true on
 * this machine has never been checked; the ADT says the AOP's own __ETEXT
 * segment has remap != phys, which is what a TRANSLATING stream 0 looks like.
 *
 * This is read-only on purpose.  Forcing stream 0 to bypass would change the
 * meaning of every address a RUNNING coprocessor is already using.  Even
 * stream 10, which is idle, is left alone: putting it in bypass would create an
 * unconstrained physical-DMA agent, and the DMA-capable-device-in-bypass
 * hazard is one this project has already had to live with on the USB DARTs.
 *
 * /arm-io/dart-aop has neither clock-gates nor power-gates in the ADT, so these
 * reads need no PMGR work and cannot be gated off.
 */
static void media_probe_dart_aop(void)
{
    u64 regs = J414S_DART_AOP_BASE;
    u32 tcr_core = 0;
    u32 tcr_admac = 0;
    bool bypass_ok;

    media_dart_report("dart-aop", regs);
    media_dart_report_stream("dart-aop", regs, AOP_DART_SID_CORE);
    media_dart_report_stream("dart-aop", regs, AOP_DART_SID_ADMAC);

    tcr_core = read32(regs + DART_T8110_TCR(AOP_DART_SID_CORE));
    tcr_admac = read32(regs + DART_T8110_TCR(AOP_DART_SID_ADMAC));
    bypass_ok = !(tcr_core & DART_T8110_TCR_TRANSLATE) &&
                !(tcr_admac & DART_T8110_TCR_TRANSLATE) &&
                (tcr_core & DART_T8110_TCR_BYPASS_DART) &&
                (tcr_admac & DART_T8110_TCR_BYPASS_DART);

    printf("media-handoff: AppleAopAudio DVA==PA precondition (streams %d and %d both "
           "BYPASS_DART, TRANSLATE clear): %s\n",
           AOP_DART_SID_CORE, AOP_DART_SID_ADMAC, bypass_ok ? "HOLDS" : "DOES NOT HOLD");
    if (!bypass_ok)
        printf("media-handoff: AopAudio will refuse to arm capture; m1n1 will NOT force it, "
               "because stream %d belongs to a running coprocessor\n",
               AOP_DART_SID_CORE);
}

/*
 * dart-sio and dart-isp0 both carry PMGR gates (SIO-DART id 502, ISP-SYS-DART
 * id 511, both virtual devices whose parents are the real rails), so their
 * registers cannot be read until those gates are up.
 *
 * The gate is raised and then deliberately NOT lowered.  src/dapf.c does lower
 * it, which is correct for its own transient use, but is exactly wrong here:
 * power-gating a DART discards TTBR/TCR, and AppleIsp adopts dart-isp0's
 * INHERITED translation instead of installing its own.  A probe that wiped that
 * table would break the camera it was supposed to be measuring.
 */
static int media_probe_gated_dart(const char *path, const char *label, u64 regs, const u32 *sids,
                                  u32 sid_count)
{
    if (pmgr_adt_power_enable(path) < 0) {
        printf("media-handoff: could not power %s; not probing\n", path);
        return MEDIA_ERR_DART_POWER;
    }

    media_dart_report(label, regs);
    for (u32 i = 0; i < sid_count; i++)
        media_dart_report_stream(label, regs, sids[i]);

    return MEDIA_HANDOFF_OK;
}

static int media_probe_gated_darts(void)
{
    static const u32 sio_sids[] = {SIO_DART_SID_ADMAC};
    static const u32 isp_sids[] = {ISP_DART_SID_MAIN};
    int status;

    printf("media-handoff: probing gated DARTs; their PMGR gates are left RAISED so no "
           "inherited translation is discarded\n");

    status = media_probe_gated_dart(DART_SIO_PATH, "dart-sio", J414S_DART_SIO_BASE, sio_sids,
                                    ARRAY_SIZE(sio_sids));
    if (status)
        return status;
    printf("media-handoff: AppleMcaAudio installs its own translating domain on dart-sio "
           "sid %d; a VALID ttbr above means it will refuse rather than steal it\n",
           SIO_DART_SID_ADMAC);

    /*
     * Only DARTLLT (instance 0) is probed.  dart-isp0 has six reg windows --
     * three translation instances (DARTLLT/DARTBULK/DARTRT at reg 0/1/2) and
     * three SMMU/DAPF shadows -- and the shadows are not DART register files.
     */
    status = media_probe_gated_dart(DART_ISP0_PATH, "dart-isp0/DARTLLT", J414S_DART_ISP0_BASE,
                                    isp_sids, ARRAY_SIZE(isp_sids));
    if (status)
        return status;
    printf("media-handoff: AppleIsp ADOPTS the ttbr above on dart-isp0 sid %d; an invalid "
           "ttbr means the inherited camera page table is already gone\n",
           ISP_DART_SID_MAIN);

    return MEDIA_HANDOFF_OK;
}

/*
 * The only write in this file.
 *
 * clk_set_mca_muxes() (src/clk.c) programs these six registers on every Linux
 * boot, from clk_init() in kboot_boot().  The Windows profile never runs
 * kboot_boot(), so on this boot path the MCA clock muxes have never been
 * programmed by m1n1 at all.  AppleMcaAudio can do it itself when Mu publishes
 * the mux window as an optional _CRS resource; this exists for the case where
 * it does not, and to make the two boot paths comparable.
 *
 * Safety: this selects WHICH NCO feeds each MCA cluster.  It does not enable
 * MCLK, does not raise ps_mca0..3, does not start a serialiser, and does not
 * touch i2c1/i2c3.  No amplifier can be driven by any value written here.  The
 * window is bounded to the ADT-declared 0x18 bytes and verified to be the
 * pinned J414s address before the first store, and every register is read back.
 */
static int media_set_mca_clock_muxes(void)
{
    u64 base = media_mca_clkmux_base;
    u32 count = J414S_MCA_CLKMUX_SIZE / 4;

    if (base != J414S_MCA_CLKMUX_BASE)
        return MEDIA_ERR_LAYOUT;

    for (u32 i = 0; i < count; i++) {
        u32 want = FIELD_PREP(MCA_CLK_MUX, MCA_CLK_NCO_BASE + min(MCA_CLK_NUM_NCOS - 1, i));

        printf("media-handoff: mca clkmux[%u] @ %#lx: %#010x ->", i, base + 4 * i,
               read32(base + 4 * i));
        mask32(base + 4 * i, MCA_CLK_MUX, want);
        printf(" %#010x\n", read32(base + 4 * i));

        if ((read32(base + 4 * i) & MCA_CLK_MUX) != want) {
            printf("media-handoff: mca clkmux[%u] read-back failed\n", i);
            return MEDIA_ERR_MCA_MUX_READBACK;
        }
    }

    printf("media-handoff: %u MCA clock muxes selected (NCO %d..%d); no MCLK enabled, no "
           "ps_mcaN raised, no amplifier touched\n",
           count, MCA_CLK_NCO_BASE, MCA_CLK_NCO_BASE + MCA_CLK_NUM_NCOS - 1);
    return MEDIA_HANDOFF_OK;
}

int media_handoff_init(u32 flags)
{
    int status;

    media_mca_clkmux_base = 0;

    if (flags & ~(u32)MEDIA_HANDOFF_FLAG_ALL) {
        printf("media-handoff: unknown flags %#x\n", flags);
        return MEDIA_ERR_FLAGS;
    }

    if (!platform_is_j414s()) {
        printf("media-handoff: exact J414s platform identity mismatch\n");
        return MEDIA_ERR_IDENTITY;
    }

    printf("media-handoff: J414s media profile, flags %#x (%s)\n", flags,
           flags ? "opt-in actions selected" : "read-only census");

    status = media_check_layout();
    if (status)
        return status;

    status = media_check_isp_carveout();
    if (status)
        return status;

    media_audit_aop_segments();
    media_probe_dart_aop();

    if (flags & MEDIA_HANDOFF_FLAG_PROBE_GATED_DARTS) {
        status = media_probe_gated_darts();
        if (status)
            return status;
    } else {
        printf("media-handoff: dart-sio and dart-isp0 not probed (needs "
               "MEDIA_HANDOFF_FLAG_PROBE_GATED_DARTS)\n");
    }

    if (flags & MEDIA_HANDOFF_FLAG_MCA_CLOCK_MUXES) {
        status = media_set_mca_clock_muxes();
        if (status)
            return status;
    } else {
        printf("media-handoff: MCA clock muxes left as firmware set them (needs "
               "MEDIA_HANDOFF_FLAG_MCA_CLOCK_MUXES)\n");
    }

    printf("media-handoff: complete; AOP SRAM window %#llx/+%#llx, no interrupt published, "
           "no DART written, no speaker path enabled\n",
           (unsigned long long)J414S_AOP_SRAM_BASE, (unsigned long long)J414S_AOP_SRAM_SIZE);
    return MEDIA_HANDOFF_OK;
}

#else

int media_handoff_init(u32 flags)
{
    UNUSED(flags);
    return 0;
}

#endif
