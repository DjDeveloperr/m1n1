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
/*
 * Floor only.  The real window comes from the ADT: J414s reports the MTP ASC
 * aperture as 0x2a9400000/+0x6c000.  The previous literals (0x2a9c00000/1 MiB)
 * matched nothing on this machine.
 */
#define J414S_MTP_SRAM_MIN_SIZE SZ_16K

#define DOCKCHANNEL_DATA_OFFSET 0x4000
#define DOCKCHANNEL_RX_COUNT    0x2c

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
    u32 initial_rx_count;
    bool ready;
};

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

static bool mtp_handoff_get_resources(void)
{
    int dockchannel_path[8];
    int mtp_path[8];
    u64 irq_size;
    u64 config_size;

    if (adt_path_offset_trace(adt, MTP_DOCKCHANNEL_PATH, dockchannel_path) < 0 ||
        adt_get_reg(adt, dockchannel_path, "reg", 1, &mtp_handoff.irq_base, &irq_size) < 0 ||
        adt_get_reg(adt, dockchannel_path, "reg", 2, &mtp_handoff.config_base,
                    &config_size) < 0) {
        printf("mtp-handoff: incomplete DockChannel ADT resources\n");
        return false;
    }

    /*
     * reg 0 is the MTP ASC's own aperture -- the same one asc_init() maps -- and
     * it is where the IOP's fixed RTKit system buffers live.  On J414s the live
     * ADT reports 0x2a9400000/+0x6c000, far larger than the register block
     * itself, which is the embedded SRAM.  reg 1 (0x2a9050000/+0x4000) is a
     * separate 16 KiB block and is not the buffer aperture; reading it here was
     * what made this handoff reject the machine it was written for.
     */
    if (adt_path_offset_trace(adt, MTP_PATH, mtp_path) < 0 ||
        adt_get_reg(adt, mtp_path, "reg", 0, &mtp_handoff.sram_base,
                    &mtp_handoff.sram_size) < 0) {
        printf("mtp-handoff: incomplete MTP SRAM ADT resource\n");
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

    /*
     * The window is whatever the ADT says the ASC aperture is; do not pin it to
     * a literal.  The board check above already refuses a non-J414s machine, and
     * rtkit only honours a fixed buffer address that falls inside this window --
     * anything else still has to survive DART translation.  Keep a floor so a
     * malformed ADT cannot hand us a degenerate window.
     */
    if (!mtp_handoff.sram_base || mtp_handoff.sram_size < J414S_MTP_SRAM_MIN_SIZE) {
        printf("mtp-handoff: unusable J414s MTP SRAM map %#lx/+%#lx\n",
               mtp_handoff.sram_base, mtp_handoff.sram_size);
        return false;
    }

    return true;
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
    if (mtp_handoff.dart)
        dart_shutdown(mtp_handoff.dart);

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
