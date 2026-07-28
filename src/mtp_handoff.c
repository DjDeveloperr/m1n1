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
#define J414S_MTP_IRQ_BASE    0x2a9b14000ULL
#define J414S_MTP_CONFIG_BASE 0x2a9b28000ULL
#define J414S_MTP_DATA_BASE   0x2a9b2c000ULL

#define DOCKCHANNEL_DATA_OFFSET 0x4000
#define DOCKCHANNEL_RX_COUNT    0x2c

/*
 * Keep RTKit's boot-time DART mappings out of the low/null IOVA region and
 * leave a compact, isolated window for the system endpoint buffers it maps.
 */
#define MTP_IOVA_WINDOW_BASE SZ_32M
#define MTP_IOVA_WINDOW_SIZE 0x10000000ULL

struct mtp_handoff_state {
    asc_dev_t *asc;
    dart_dev_t *dart;
    iova_domain_t *iovad;
    rtkit_dev_t *rtkit;
    u64 irq_base;
    u64 config_base;
    u64 data_base;
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

    /* Some MTP-related nodes have no PMGR clock-gates property. */
    if (!adt_getprop(adt, node, "clock-gates", NULL))
        return true;

    if (pmgr_adt_power_enable(path) < 0) {
        printf("mtp-handoff: could not power %s\n", path);
        return false;
    }

    return true;
}

static bool mtp_handoff_get_dockchannel_resources(void)
{
    int path[8];
    u64 irq_size;
    u64 config_size;

    if (adt_path_offset_trace(adt, MTP_DOCKCHANNEL_PATH, path) < 0 ||
        adt_get_reg(adt, path, "reg", 1, &mtp_handoff.irq_base, &irq_size) < 0 ||
        adt_get_reg(adt, path, "reg", 2, &mtp_handoff.config_base, &config_size) < 0) {
        printf("mtp-handoff: incomplete DockChannel ADT resources\n");
        return false;
    }

    mtp_handoff.data_base = mtp_handoff.config_base + DOCKCHANNEL_DATA_OFFSET;

    if (irq_size < 8 || config_size < 8 || mtp_handoff.irq_base != J414S_MTP_IRQ_BASE ||
        mtp_handoff.config_base != J414S_MTP_CONFIG_BASE ||
        mtp_handoff.data_base != J414S_MTP_DATA_BASE) {
        printf("mtp-handoff: unexpected J414s DockChannel map irq=%#lx/+%#lx "
               "config=%#lx/+%#lx data=%#lx\n",
               mtp_handoff.irq_base, irq_size, mtp_handoff.config_base, config_size,
               mtp_handoff.data_base);
        return false;
    }

    return true;
}

static void mtp_handoff_rollback(void)
{
    if (mtp_handoff.rtkit) {
        rtkit_quiesce(mtp_handoff.rtkit);
        rtkit_free(mtp_handoff.rtkit);
    }
    if (mtp_handoff.asc) {
        asc_cpu_stop(mtp_handoff.asc);
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

    if (!mtp_handoff_get_dockchannel_resources() || !mtp_power_enable_if_gated(MTP_PATH) ||
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
    if (!mtp_handoff.rtkit || !rtkit_boot(mtp_handoff.rtkit)) {
        printf("mtp-handoff: MTP RTKit boot failed\n");
        goto fail;
    }

    /*
     * This is the sole DockChannel register read.  RX_COUNT is a non-consuming
     * availability observation; reading RX_8/RX_32 would steal INIT from the
     * Windows driver.  Do not acknowledge IRQs or set masks/thresholds here.
     */
    mtp_handoff.initial_rx_count = read32(mtp_handoff.data_base + DOCKCHANNEL_RX_COUNT);
    mtp_handoff.ready = true;

    printf("mtp-handoff: RTKit ready; DockChannel[%d] RX=%u, FIFO preserved "
           "(irq=%#lx config=%#lx data=%#lx)\n",
           MTP_DOCKCHANNEL_INDEX, mtp_handoff.initial_rx_count, mtp_handoff.irq_base,
           mtp_handoff.config_base, mtp_handoff.data_base);
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
