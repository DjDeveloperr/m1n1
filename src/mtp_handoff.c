/* SPDX-License-Identifier: MIT */

/*
 * J414s preboot MTP/DockChannel handoff for the Windows native-AIC profile.
 *
 * The MTP IOP advertises its HID endpoints by emitting INIT packets on
 * DockChannel.  Reading even one RX data register consumes that state, so
 * this file may only inspect RX_COUNT.  It must never configure thresholds,
 * clear IRQ flags, change masks, or drain the FIFO: AppleMtpHid owns all of
 * that after ExitBootServices.
 */

#include "../config.h"

#include "adt.h"
#include "asc.h"
#include "dapf.h"
#include "dart.h"
#include "iova.h"
#include "mtp_handoff.h"
#include "pmgr.h"
#include "rtkit.h"
#include "soc.h"
#include "string.h"
#include "utils.h"

#if defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && defined(ENABLE_J414S_WINDOWS_MTP_HANDOFF)

#define MTP_PATH             "/arm-io/mtp"
#define MTP_DART_PATH        "/arm-io/dart-mtp"
#define MTP_DOCKCHANNEL_PATH "/arm-io/dockchannel-mtp"

/* DockChannel index 1 is the MTP transport on J414s. */
#define MTP_DOCKCHANNEL_INDEX 1

/*
 * J414s' physical resource layout.  The values are checked before any state
 * is changed because the Windows ACPI resources are intentionally published
 * at these physical addresses too; a different layout needs an explicit port
 * rather than a best-effort boot with the wrong device behind the driver.
 */
#define J414S_MTP_IRQ_BASE      0x2a9b14000ULL
#define J414S_MTP_CONFIG_BASE   0x2a9b30000ULL
#define J414S_MTP_DATA_BASE     0x2a9b34000ULL
#define J414S_MTP_APERTURE_SIZE 0x1000

#define DOCKCHANNEL_DATA_OFFSET 0x4000
#define DOCKCHANNEL_RX_COUNT    0x2c

/*
 * IOP-owned fixed RTKit buffer window.
 *
 * This is deliberately a literal, not an ADT "reg" read.  The live J414s
 * /arm-io/mtp regs are 0x2a9400000/+0x6c000 (the ASC block asc_init() maps)
 * and 0x2a9050000/+0x4000; neither contains the IOP's fixed buffers.  On
 * hardware the first fixed request was ep 0x1 msg 0x101002a9ca8000, i.e. a
 * 4 KiB crashlog buffer at 0x2a9ca8000 -- inside this window and inside
 * nothing else the ADT regs describe.
 *
 * The window itself is pinned upstream: Asahi Linux t602x-die0.dtsi at commit
 * e8efe09d4f378992c890d181d65e2ed8d8cb1194 gives the mtp node
 * reg-names "asc", "sram" with sram = 0x2a9c00000/0x100000, and
 * drivers/soc/apple/rtkit-helper.c at the same commit accepts a nonzero-IOVA
 * buffer request only if it is fully contained in that "sram" resource --
 * exactly the admission rule rtkit_set_phys_window() enforces here.
 *
 * The ADT's own description of this region is the "segment-ranges" property
 * on the MTP IOP nub (the iBoot-preloaded firmware carveout).  We parse and
 * log it below as verification evidence, but do not derive the admission
 * window from it yet: the declared TEXT/DATA segments can end below the
 * IOP's heap allocations (the live crashlog request sits at +0xa8000), while
 * the Linux binding pins the full 1 MiB carveout.
 */
#define J414S_MTP_FIXED_BUFFER_BASE 0x2a9c00000ULL
#define J414S_MTP_FIXED_BUFFER_SIZE 0x100000ULL

/*
 * t8110 DART registers needed to leave stream 1 provably inert if the
 * handoff rolls back.  Offsets mirror src/dart.c (DART_T8110_TCR_OFF,
 * DART_T8110_TLB_CMD, DART_T8110_PROTECT, DART_T8110_DISABLE_STREAMS);
 * dart.c does not export them and dart_shutdown() leaves the stream in
 * BYPASS, which is the one state a failed handoff must not persist.
 */
#define MTP_DART_T8110_TLB_CMD              0x80
#define MTP_DART_T8110_TLB_CMD_BUSY         BIT(31)
#define MTP_DART_T8110_TLB_CMD_OP_FLUSH_SID (1 << 8)
#define MTP_DART_T8110_PROTECT              0x200
#define MTP_DART_T8110_PROTECT_TTBR_TCR     BIT(0)
#define MTP_DART_T8110_DISABLE_STREAMS      0xc20
#define MTP_DART_T8110_TCR(sid)             (0x1000 + 4 * (sid))

/*
 * Keep RTKit's boot-time DART mappings out of the low/null IOVA region and
 * leave a compact, isolated window for the system endpoint buffers it maps.
 */
#define MTP_IOVA_WINDOW_BASE SZ_32M
#define MTP_IOVA_WINDOW_SIZE 0x10000000ULL
#define MTP_READY_TIMEOUT    (3 * USEC_PER_SEC)

struct mtp_handoff_state {
    asc_dev_t *asc;
    dart_dev_t *dart;
    iova_domain_t *iovad;
    rtkit_dev_t *rtkit;
    u64 irq_base;
    u64 config_base;
    u64 data_base;
    u64 sram_base;
    u64 sram_size;
    u64 dart_regs;
    u32 initial_rx_count;
    bool ready;
};

/*
 * One record of the ADT "segment-ranges" property carried by the MTP IOP nub
 * (compatible "iop-nub,rtbuddy-v2").  Verified against a live T6050 dump:
 * three 32-byte records whose (phys, size) tuples were TEXT 0x294c00000/
 * +0x55000, DATA 0x294c55000/+0x6c000 (the contiguous preloaded carveout)
 * and OS_LOG 0x1000d6a0000/+0x3000 (DRAM), with remap == phys throughout and
 * segment-names "__TEXT;__DATA;__OS_LOG".
 */
struct mtp_segment_range {
    u64 phys;
    u64 iop_va;
    u64 remap;
    u32 size;
    u32 flags;
} PACKED;

static struct mtp_handoff_state mtp_handoff;

static bool mtp_power_enable_if_gated(const char *path)
{
    int node = adt_path_offset(adt, path);

    if (node < 0) {
        printf("mtp-handoff: missing ADT node %s\n", path);
        return false;
    }

    /*
     * Some MTP-related nodes have no PMGR gates.  On J414s /arm-io/mtp carries
     * a clock-gates property that is present but empty, so testing existence
     * alone sent it to pmgr, which requires at least one 32-bit entry and
     * failed with "Error getting /arm-io/mtp clock-gates".  An empty property
     * means the same thing as an absent one: nothing here to power up.
     */
    u32 gates_len = 0;
    if (!adt_getprop(adt, node, "clock-gates", &gates_len) || gates_len < sizeof(u32))
        return true;

    if (pmgr_adt_power_enable(path) < 0) {
        printf("mtp-handoff: could not power %s\n", path);
        return false;
    }

    return true;
}

/*
 * Log the IOP nub's declared firmware carveout.  Read-only ADT evidence: the
 * next hardware capture tells us whether the J414s segment layout matches the
 * pinned 1 MiB window so a follow-up can derive it instead of pinning it.
 * Never fails the handoff.
 */
static void mtp_log_segment_ranges(void)
{
    int node = adt_path_offset(adt, MTP_PATH);
    if (node < 0)
        return;

    u32 len = 0;
    const struct mtp_segment_range *seg = adt_getprop(adt, node, "segment-ranges", &len);
    if (!seg) {
        node = adt_first_child_offset(adt, node);
        if (node >= 0)
            seg = adt_getprop(adt, node, "segment-ranges", &len);
    }

    if (!seg) {
        printf("mtp-handoff: no segment-ranges property on %s or its nub\n", MTP_PATH);
        return;
    }
    if (!len || (len % sizeof(*seg)) != 0) {
        printf("mtp-handoff: unparsed segment-ranges (len %u)\n", len);
        return;
    }

    for (u32 i = 0; i < len / sizeof(*seg); i++)
        printf("mtp-handoff: IOP segment %u phys=%#lx iova=%#lx remap=%#lx size=%#x "
               "flags=%#x\n",
               i, seg[i].phys, seg[i].iop_va, seg[i].remap, seg[i].size, seg[i].flags);

    if (seg[0].phys != J414S_MTP_FIXED_BUFFER_BASE)
        printf("mtp-handoff: WARNING: carveout starts at %#lx, fixed-buffer window "
               "pinned at %#llx\n",
               seg[0].phys, J414S_MTP_FIXED_BUFFER_BASE);
}

static bool mtp_handoff_get_resources(void)
{
    int dockchannel_path[8];
    u64 irq_size;
    u64 config_size;

    if (adt_path_offset_trace(adt, MTP_DOCKCHANNEL_PATH, dockchannel_path) < 0 ||
        adt_get_reg(adt, dockchannel_path, "reg", 1, &mtp_handoff.irq_base, &irq_size) < 0 ||
        adt_get_reg(adt, dockchannel_path, "reg", 2, &mtp_handoff.config_base,
                    &config_size) < 0) {
        printf("mtp-handoff: incomplete DockChannel ADT resources\n");
        return false;
    }

    mtp_handoff.data_base = mtp_handoff.config_base + DOCKCHANNEL_DATA_OFFSET;

    if (irq_size < J414S_MTP_APERTURE_SIZE || config_size < J414S_MTP_APERTURE_SIZE ||
        mtp_handoff.irq_base != J414S_MTP_IRQ_BASE ||
        mtp_handoff.config_base != J414S_MTP_CONFIG_BASE ||
        mtp_handoff.data_base != J414S_MTP_DATA_BASE) {
        printf("mtp-handoff: unexpected J414s DockChannel map irq=%#lx/+%#lx "
               "config=%#lx/+%#lx data=%#lx\n",
               mtp_handoff.irq_base, irq_size, mtp_handoff.config_base, config_size,
               mtp_handoff.data_base);
        return false;
    }

    /* See the J414S_MTP_FIXED_BUFFER_* comment for why this is not an ADT read. */
    mtp_handoff.sram_base = J414S_MTP_FIXED_BUFFER_BASE;
    mtp_handoff.sram_size = J414S_MTP_FIXED_BUFFER_SIZE;

    mtp_log_segment_ranges();

    return true;
}

/*
 * dart_shutdown() leaves the stream TCR in BYPASS_DAPF|BYPASS_DART.  That is
 * acceptable for m1n1's own transient users, but this rollback runs with an
 * MTP IOP that was started and then clamped without a quiesce handshake.  A
 * bypassed stream would let any late or future IOP access reach physical
 * memory unfiltered.  Force the stream to the blocked state instead: TCR 0
 * (neither translate nor bypass), stream disabled, TLB flushed.
 */
static void mtp_handoff_block_dart_stream(void)
{
    if (!mtp_handoff.dart_regs)
        return;

    if (read32(mtp_handoff.dart_regs + MTP_DART_T8110_PROTECT) &
        MTP_DART_T8110_PROTECT_TTBR_TCR) {
        printf("mtp-handoff: DART locked; cannot block stream %d\n", MTP_DOCKCHANNEL_INDEX);
        return;
    }

    write32(mtp_handoff.dart_regs + MTP_DART_T8110_TCR(MTP_DOCKCHANNEL_INDEX), 0);
    write32(mtp_handoff.dart_regs + MTP_DART_T8110_DISABLE_STREAMS,
            BIT(MTP_DOCKCHANNEL_INDEX));
    write32(mtp_handoff.dart_regs + MTP_DART_T8110_TLB_CMD,
            MTP_DART_T8110_TLB_CMD_OP_FLUSH_SID | MTP_DOCKCHANNEL_INDEX);
    if (poll32(mtp_handoff.dart_regs + MTP_DART_T8110_TLB_CMD, MTP_DART_T8110_TLB_CMD_BUSY,
               0, 100))
        printf("mtp-handoff: DART TLB flush did not complete\n");

    printf("mtp-handoff: DART stream %d left blocked\n", MTP_DOCKCHANNEL_INDEX);
}

static void mtp_handoff_rollback(void)
{
    /* Stop DMA-producing firmware before releasing any RTKit/DART state. */
    if (mtp_handoff.asc)
        asc_cpu_stop(mtp_handoff.asc);
    if (mtp_handoff.rtkit)
        rtkit_free(mtp_handoff.rtkit);
    if (mtp_handoff.asc) {
        asc_free(mtp_handoff.asc);
    }
    if (mtp_handoff.iovad)
        iovad_shutdown(mtp_handoff.iovad, mtp_handoff.dart);
    if (mtp_handoff.dart) {
        dart_shutdown(mtp_handoff.dart);
        mtp_handoff_block_dart_stream();
    }

    memset(&mtp_handoff, 0, sizeof(mtp_handoff));
}

void mtp_handoff_init(void)
{
    if (mtp_handoff.ready)
        return;

    if (chip_id != T6020 || !adt_is_compatible(adt, 0, "J414sAP"))
        return;

    printf("mtp-handoff: preparing J414s MTP for Windows DockChannel ownership\n");

    if (!mtp_handoff_get_resources() || !mtp_power_enable_if_gated(MTP_PATH) ||
        !mtp_power_enable_if_gated(MTP_DART_PATH) ||
        !mtp_power_enable_if_gated(MTP_DOCKCHANNEL_PATH))
        goto fail;

    /*
     * Program DAPF first.  dapf_init() transiently powers a gated DART down
     * once it has programmed the filter, so the persistent power enable above
     * is repeated afterwards before touching the stream's page tables.
     */
    if (dapf_init(MTP_DART_PATH, MTP_DOCKCHANNEL_INDEX) < 0 ||
        !mtp_power_enable_if_gated(MTP_DART_PATH)) {
        printf("mtp-handoff: DAPF setup failed\n");
        goto fail;
    }

    /*
     * Record the DART MMIO base and the stream's cold TCR before dart_init_adt
     * programs it: the TCR value is the capture that tells us what state a
     * clean rollback should ideally restore, and the base is what lets the
     * rollback force the stream to blocked.
     */
    int dart_path[8];
    if (adt_path_offset_trace(adt, MTP_DART_PATH, dart_path) < 0 ||
        adt_get_reg(adt, dart_path, "reg", 0, &mtp_handoff.dart_regs, NULL) < 0) {
        printf("mtp-handoff: DART stream %d setup failed\n", MTP_DOCKCHANNEL_INDEX);
        goto fail;
    }
    printf("mtp-handoff: DART stream %d cold TCR=%#x\n", MTP_DOCKCHANNEL_INDEX,
           read32(mtp_handoff.dart_regs + MTP_DART_T8110_TCR(MTP_DOCKCHANNEL_INDEX)));

    mtp_handoff.dart = dart_init_adt(MTP_DART_PATH, 0, MTP_DOCKCHANNEL_INDEX, false);
    if (!mtp_handoff.dart) {
        printf("mtp-handoff: DART stream %d setup failed\n", MTP_DOCKCHANNEL_INDEX);
        goto fail;
    }

    mtp_handoff.iovad =
        iovad_init(MTP_IOVA_WINDOW_BASE, MTP_IOVA_WINDOW_BASE + MTP_IOVA_WINDOW_SIZE);
    if (!mtp_handoff.iovad) {
        printf("mtp-handoff: IOVA allocator setup failed\n");
        goto fail;
    }

    mtp_handoff.asc = asc_init(MTP_PATH);
    if (!mtp_handoff.asc) {
        printf("mtp-handoff: MTP ASC setup failed\n");
        goto fail;
    }

    mtp_handoff.rtkit = rtkit_init("mtp-handoff", mtp_handoff.asc, mtp_handoff.dart,
                                   mtp_handoff.iovad, NULL, false);
    if (!mtp_handoff.rtkit ||
        !rtkit_set_phys_window(mtp_handoff.rtkit, mtp_handoff.sram_base,
                               mtp_handoff.sram_size) ||
        !rtkit_boot_timed(mtp_handoff.rtkit, MTP_READY_TIMEOUT)) {
        printf("mtp-handoff: MTP RTKit boot failed\n");
        goto fail;
    }
    /*
     * RX_COUNT is the sole DockChannel register polled. It is non-consuming;
     * reading RX_8/RX_32 would steal INIT from the Windows driver. Do not
     * acknowledge IRQs or set masks/thresholds here. Requiring queued data
     * closes the race between AP=ON and Windows taking transport ownership.
     */
    u64 timeout = timeout_calculate(MTP_READY_TIMEOUT);
    do {
        mtp_handoff.initial_rx_count =
            read32(mtp_handoff.data_base + DOCKCHANNEL_RX_COUNT);
        if (mtp_handoff.initial_rx_count)
            break;
    } while (!timeout_expired(timeout));

    if (!mtp_handoff.initial_rx_count) {
        printf("mtp-handoff: no DockChannel INIT data after RTKit AP reached ON\n");
        goto fail;
    }
    mtp_handoff.ready = true;

    printf("mtp-handoff: RTKit ready; DockChannel[%d] RX=%u, FIFO preserved "
           "(irq=%#lx config=%#lx data=%#lx sram=%#lx/+%#lx)\n",
           MTP_DOCKCHANNEL_INDEX, mtp_handoff.initial_rx_count, mtp_handoff.irq_base,
           mtp_handoff.config_base, mtp_handoff.data_base, mtp_handoff.sram_base,
           mtp_handoff.sram_size);
    return;

fail:
    mtp_handoff_rollback();
    printf("mtp-handoff: disabled after setup failure; Windows will not receive partial state\n");
}

#else

void mtp_handoff_init(void)
{
}

#endif
