/* SPDX-License-Identifier: MIT */

/*
 * Persistent J414s BCM4388 preboot DART handoff for Windows.
 *
 * pci.sys enables endpoint bus mastering before any KMDF provider can run.
 * This code therefore installs SID 1 at EL2, before Mu: the inherited domain
 * translates only the APCIE MSI doorbell page and faults every client DMA
 * address. Mu reserves/publishes the table carveout; AppleDart.sys validates
 * the live registers and every table entry read-only before adopting it.
 */

#include "../config.h"

#include "adt.h"
#include "pcie.h"
#include "platform_identity.h"
#include "string.h"
#include "types.h"
#include "utils.h"
#include "wireless_handoff.h"
#include "wireless_handoff_abi.h"
#include "xnuboot.h"

#if defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && defined(ENABLE_J414S_WINDOWS_WIRELESS_HANDOFF)

#define WLAN_DART_PATH "/arm-io/dart-apcie0"

#define WLAN_DART0_BASE 0x594000000ULL
#define WLAN_DART0_SIZE 0x4000ULL
#define WLAN_ECAM_BASE  0x580000000ULL
#define WLAN_WIFI_ID    0x443414e4U
#define WLAN_BT_ID      0x5f7214e4U

#define WLAN_DART_PARAMS1              0x000
#define WLAN_DART_PARAMS1_LOG2_PAGE    GENMASK(27, 24)
#define WLAN_DART_PARAMS3              0x008
#define WLAN_DART_PARAMS3_PA_WIDTH     GENMASK(29, 24)
#define WLAN_DART_PARAMS4              0x00c
#define WLAN_DART_PARAMS4_SID_COUNT    GENMASK(8, 0)
#define WLAN_DART_TLB_CMD              0x080
#define WLAN_DART_TLB_CMD_BUSY         BIT(31)
#define WLAN_DART_TLB_CMD_FLUSH_SID1   0x101
#define WLAN_DART_ERROR                0x100
#define WLAN_DART_ERROR_STREAMS        0x1c0
#define WLAN_DART_PROTECT              0x200
#define WLAN_DART_PROTECT_TTBR_TCR     BIT(0)
#define WLAN_DART_ENABLE_STREAMS       0xc00
#define WLAN_DART_DISABLE_STREAMS      0xc20
#define WLAN_DART_TCR(sid)             (0x1000 + 4 * (sid))
#define WLAN_DART_TCR_TRANSLATE_ENABLE BIT(0)
#define WLAN_DART_TTBR(sid)            (0x1400 + 4 * (sid))
#define WLAN_DART_TTBR_VALID           BIT(0)
#define WLAN_DART_TTBR_ADDR            GENMASK(29, 2)
#define WLAN_DART_TTBR_SHIFT           14

#define WLAN_SID             1
#define WLAN_DART_PAGE_SHIFT 14
#define WLAN_DART_PAGE_SIZE  (1UL << WLAN_DART_PAGE_SHIFT)
#define WLAN_DART_PTE_OFFSET GENMASK(39, 10)
#define WLAN_DART_PTE_VALID  BIT(0)
#define WLAN_DART_PTE_SP_END GENMASK(51, 40)

#define WLAN_MSI_DOORBELL_IOVA 0xfffff000ULL
#define WLAN_MSI_DOORBELL_PAGE 0xffffc000ULL
#define WLAN_MSI_L1_INDEX      127
#define WLAN_MSI_L2_INDEX      2047

/* Runtime reservation geometry, published by the paired Mu DRT0 profile. */
#define WLAN_PT_CARVEOUT_SIZE 0x10000ULL
#define WLAN_PT_ALIGNMENT     0x4000ULL

_Static_assert(WLAN_PT_CARVEOUT_SIZE == WIRELESS_HANDOFF_V2_RESERVATION_SIZE,
               "runtime and ABI reservation size");

enum wlan_handoff_error {
    WLAN_HANDOFF_OK = 0,
    WLAN_ERR_ADT = -1,
    WLAN_ERR_LAYOUT = -2,
    WLAN_ERR_DART_LOCKED = -3,
    WLAN_ERR_DART_BUSY = -4,
    WLAN_ERR_SID1_LIVE = -5,
    WLAN_ERR_BUS_MASTER = -6,
    WLAN_ERR_PARAMS = -7,
    WLAN_ERR_READBACK = -8,
    WLAN_ERR_FLUSH = -9,
    WLAN_ERR_PORT_SETUP = -10,
    WLAN_ERR_PREEXISTING_FAULT = -11,
    WLAN_ERR_IDENTITY = -12,
    WLAN_ERR_ENDPOINT_ID = -13,
    WLAN_ERR_RESERVATION = -14,
};

static u64 wlan_dart_regs;
static bool wlan_wrote_dart;
static u64 wlan_pt_carveout_phys;

#define WLAN_PT_L1_PHYS     (wlan_pt_carveout_phys + 0x0000)
#define WLAN_PT_MSI_L2_PHYS (wlan_pt_carveout_phys + 0x4000)
#define WLAN_DESCRIPTOR_PHYS \
    (wlan_pt_carveout_phys + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET)

static int wlan_validate_reservation(u64 base, u64 size)
{
    u64 ram_base = ALIGN_DOWN(cur_boot_args.phys_base, BIT(32));
    u64 physical_top = ram_base + mem_size_actual;
    u64 guest_top = cur_boot_args.phys_base + cur_boot_args.mem_size;

    if (size != WLAN_PT_CARVEOUT_SIZE || (base & (WLAN_PT_ALIGNMENT - 1)) ||
        base > ~0ULL - size)
        return WLAN_ERR_RESERVATION;

    /*
     * top_of_memory_alloc() leaves a 16-KiB guard below its first result.
     * Requiring this range above the reduced boot_args top proves Mu cannot
     * allocate it as SystemMemory. The physical-top bound proves real DRAM.
     */
    if (base < guest_top + SZ_16K || base + size > physical_top)
        return WLAN_ERR_RESERVATION;

    return WLAN_HANDOFF_OK;
}

static u32 wlan_pci_command(u32 function)
{
    u64 config = WLAN_ECAM_BASE + (1ULL << 20) + ((u64)function << 12);

    return read32(config + 4) & 0xffff;
}

static u32 wlan_pci_identity(u32 function)
{
    u64 config = WLAN_ECAM_BASE + (1ULL << 20) + ((u64)function << 12);

    return read32(config);
}

static int wlan_check_endpoints_quiescent(void)
{
    static const u32 expected_identity[2] = {WLAN_WIFI_ID, WLAN_BT_ID};

    for (u32 function = 0; function < 2; function++) {
        u32 identity = wlan_pci_identity(function);
        u32 command = wlan_pci_command(function);

        if (identity != expected_identity[function]) {
            printf("wlan-handoff: function %u identity %#x, expected %#x\n", function,
                   identity, expected_identity[function]);
            return WLAN_ERR_ENDPOINT_ID;
        }
        if (command & BIT(2)) {
            printf("wlan-handoff: function %u already bus-mastering (cmd %#x)\n", function,
                   command);
            return WLAN_ERR_BUS_MASTER;
        }
    }
    return WLAN_HANDOFF_OK;
}

static int wlan_check_dart_quiescent(void)
{
    u32 params1 = read32(wlan_dart_regs + WLAN_DART_PARAMS1);
    u32 params3 = read32(wlan_dart_regs + WLAN_DART_PARAMS3);
    u32 params4 = read32(wlan_dart_regs + WLAN_DART_PARAMS4);
    u32 log2_page = FIELD_GET(WLAN_DART_PARAMS1_LOG2_PAGE, params1);
    u32 pa_width = FIELD_GET(WLAN_DART_PARAMS3_PA_WIDTH, params3);
    u32 sid_count = FIELD_GET(WLAN_DART_PARAMS4_SID_COUNT, params4);

    if (log2_page != WLAN_DART_PAGE_SHIFT || pa_width < WLAN_DART_PAGE_SHIFT ||
        pa_width > 63 || sid_count <= WLAN_SID) {
        printf("wlan-handoff: unexpected DART params page=%u pa=%u sids=%u\n", log2_page,
               pa_width, sid_count);
        return WLAN_ERR_PARAMS;
    }
    if ((WLAN_PT_L1_PHYS >> pa_width) != 0)
        return WLAN_ERR_PARAMS;
    if (read32(wlan_dart_regs + WLAN_DART_PROTECT) & WLAN_DART_PROTECT_TTBR_TCR)
        return WLAN_ERR_DART_LOCKED;
    if (read32(wlan_dart_regs + WLAN_DART_TLB_CMD) & WLAN_DART_TLB_CMD_BUSY)
        return WLAN_ERR_DART_BUSY;
    /*
     * "Live" means the stream can actually translate, which requires a VALID
     * TTBR and the stream enabled.  It is NOT simply a non-zero TCR.
     *
     * Measured on J414s once the WLAN rail is up and the link trains: every
     * SID reads TCR = WLAN_DART_TCR_TRANSLATE_ENABLE with TTBR = 0 and
     * ENABLE_STREAMS = 0.  That uniformity across all SIDs is the signature of
     * the DART's reset default, not of anybody's configuration -- and with no
     * valid TTBR and the stream disabled the endpoint cannot DMA at all, so
     * there is nothing to clobber.  Refusing on raw TCR != 0 made the handoff
     * unreachable on this machine (it returned WLAN_ERR_SID1_LIVE forever).
     *
     * Still fail closed on anything that could be a real translation: a valid
     * TTBR, an enabled stream, or TCR bits beyond translate-enable.
     */
    u32 sid_tcr = read32(wlan_dart_regs + WLAN_DART_TCR(WLAN_SID));
    u32 sid_ttbr = read32(wlan_dart_regs + WLAN_DART_TTBR(WLAN_SID));
    u32 enabled_streams = read32(wlan_dart_regs + WLAN_DART_ENABLE_STREAMS);

    if ((sid_ttbr & WLAN_DART_TTBR_VALID) || (enabled_streams & BIT(WLAN_SID)) ||
        (sid_tcr & ~WLAN_DART_TCR_TRANSLATE_ENABLE) != 0) {
        printf("wlan-handoff: SID%d is live (tcr=%#x ttbr=%#x streams=%#x)\n", WLAN_SID, sid_tcr,
               sid_ttbr, enabled_streams);
        return WLAN_ERR_SID1_LIVE;
    }
    if (read32(wlan_dart_regs + WLAN_DART_ERROR) != 0 ||
        read32(wlan_dart_regs + WLAN_DART_ERROR_STREAMS) != 0)
        return WLAN_ERR_PREEXISTING_FAULT;
    return WLAN_HANDOFF_OK;
}

static u64 wlan_encode_pte(u64 physical)
{
    return FIELD_PREP(WLAN_DART_PTE_SP_END, 0xfff) |
           FIELD_PREP(WLAN_DART_PTE_OFFSET, physical >> WLAN_DART_PAGE_SHIFT) |
           WLAN_DART_PTE_VALID;
}

static u32 wlan_encode_ttbr(u64 physical)
{
    return WLAN_DART_TTBR_VALID |
           FIELD_PREP(WLAN_DART_TTBR_ADDR, physical >> WLAN_DART_TTBR_SHIFT);
}

static void wlan_build_tables(void)
{
    u64 *l1 = (u64 *)WLAN_PT_L1_PHYS;
    u64 *msi_l2 = (u64 *)WLAN_PT_MSI_L2_PHYS;

    memset((void *)wlan_pt_carveout_phys, 0, WLAN_PT_CARVEOUT_SIZE);
    msi_l2[WLAN_MSI_L2_INDEX] = wlan_encode_pte(WLAN_MSI_DOORBELL_PAGE);
    l1[WLAN_MSI_L1_INDEX] = wlan_encode_pte(WLAN_PT_MSI_L2_PHYS);
    dma_wmb();
}

static int wlan_publish_descriptor(u64 reservation_size)
{
    struct wireless_handoff_descriptor_v2 descriptor = {
        .signature = WIRELESS_HANDOFF_V2_SIGNATURE,
        .version = WIRELESS_HANDOFF_V2_VERSION,
        .structure_size = sizeof(descriptor),
        .flags = WIRELESS_HANDOFF_V2_FLAG_INSTALLED,
        .sid = WIRELESS_HANDOFF_V2_SID,
        .page_shift = WIRELESS_HANDOFF_V2_PAGE_SHIFT,
        .reservation_base = wlan_pt_carveout_phys,
        .reservation_size = reservation_size,
        .guest_memory_top = cur_boot_args.phys_base + cur_boot_args.mem_size,
        .physical_memory_top = ALIGN_DOWN(cur_boot_args.phys_base, BIT(32)) +
                               mem_size_actual,
        .dart_base = WLAN_DART0_BASE,
        .l1_physical = WLAN_PT_L1_PHYS,
        .msi_l2_physical = WLAN_PT_MSI_L2_PHYS,
        .descriptor_physical = WLAN_DESCRIPTOR_PHYS,
    };

    descriptor.l1_crc32 = wireless_handoff_v2_crc32(
        (const void *)WLAN_PT_L1_PHYS, WIRELESS_HANDOFF_V2_PAGE_SIZE);
    descriptor.msi_l2_crc32 = wireless_handoff_v2_crc32(
        (const void *)WLAN_PT_MSI_L2_PHYS, WIRELESS_HANDOFF_V2_PAGE_SIZE);
    descriptor.descriptor_crc32 = 0;
    descriptor.descriptor_crc32 = wireless_handoff_v2_crc32(
        &descriptor, sizeof(descriptor));
    memcpy((void *)WLAN_DESCRIPTOR_PHYS, &descriptor, sizeof(descriptor));
    dma_wmb();
    return wireless_handoff_v2_descriptor_validate(
        (const void *)WLAN_DESCRIPTOR_PHYS,
        (const void *)wlan_pt_carveout_phys,
        wlan_pt_carveout_phys, reservation_size,
        cur_boot_args.phys_base + cur_boot_args.mem_size);
}

static int wlan_flush_sid1(void)
{
    dma_wmb();
    write32(wlan_dart_regs + WLAN_DART_TLB_CMD, WLAN_DART_TLB_CMD_FLUSH_SID1);
    if (poll32(wlan_dart_regs + WLAN_DART_TLB_CMD, WLAN_DART_TLB_CMD_BUSY, 0, 100))
        return WLAN_ERR_FLUSH;
    return WLAN_HANDOFF_OK;
}

static void wlan_block_sid1(void)
{
    if (!wlan_dart_regs || !wlan_wrote_dart)
        return;
    if (read32(wlan_dart_regs + WLAN_DART_PROTECT) & WLAN_DART_PROTECT_TTBR_TCR) {
        printf("wlan-handoff: DART locked during rollback; SID 1 state unknown\n");
        return;
    }

    write32(wlan_dart_regs + WLAN_DART_DISABLE_STREAMS, BIT(WLAN_SID));
    write32(wlan_dart_regs + WLAN_DART_TCR(WLAN_SID), 0);
    write32(wlan_dart_regs + WLAN_DART_TTBR(WLAN_SID), 0);
    dma_wmb();
    write32(wlan_dart_regs + WLAN_DART_TLB_CMD, WLAN_DART_TLB_CMD_FLUSH_SID1);
    if (poll32(wlan_dart_regs + WLAN_DART_TLB_CMD, WLAN_DART_TLB_CMD_BUSY, 0, 100))
        printf("wlan-handoff: rollback flush timed out\n");
    if (read32(wlan_dart_regs + WLAN_DART_TCR(WLAN_SID)) != 0 ||
        read32(wlan_dart_regs + WLAN_DART_TTBR(WLAN_SID)) != 0)
        printf("wlan-handoff: rollback readback failed\n");
}

static int wlan_mmio_read32(void *context, u64 address, u32 *value)
{
    UNUSED(context);
    *value = read32(address);
    return 0;
}

static int wlan_mmio_write32(void *context, u64 address, u32 value)
{
    UNUSED(context);
    write32(address, value);
    return 0;
}

static const struct pcie_t602x_mmio_ops wlan_mmio_ops = {
    .read32 = wlan_mmio_read32,
    .write32 = wlan_mmio_write32,
};

int wireless_handoff_init(u64 reservation_base, u64 reservation_size)
{
    int adt_path[8];
    u64 dart_base;
    u64 dart_size;
    int status;

    wlan_dart_regs = 0;
    wlan_wrote_dart = false;
    wlan_pt_carveout_phys = 0;
    if (!platform_is_j414s()) {
        printf("wlan-handoff: exact J414s platform identity mismatch\n");
        return WLAN_ERR_IDENTITY;
    }

    status = wlan_validate_reservation(reservation_base, reservation_size);
    if (status) {
        printf("wlan-handoff: invalid reservation %#llx+%#llx\n",
               (unsigned long long)reservation_base, (unsigned long long)reservation_size);
        return status;
    }
    wlan_pt_carveout_phys = reservation_base;

    if (adt_path_offset_trace(adt, WLAN_DART_PATH, adt_path) < 0 ||
        adt_get_reg(adt, adt_path, "reg", 0, &dart_base, &dart_size) < 0)
        return WLAN_ERR_ADT;
    if (dart_base != WLAN_DART0_BASE || dart_size < WLAN_DART0_SIZE)
        return WLAN_ERR_LAYOUT;
    wlan_dart_regs = dart_base;

    status = wlan_check_endpoints_quiescent();
    if (status)
        return status;
    status = wlan_check_dart_quiescent();
    if (status)
        return status;

    wlan_build_tables();
    wlan_wrote_dart = true;
    write32(wlan_dart_regs + WLAN_DART_TCR(WLAN_SID), 0);
    write32(wlan_dart_regs + WLAN_DART_TTBR(WLAN_SID), 0);
    write32(wlan_dart_regs + WLAN_DART_ENABLE_STREAMS, BIT(WLAN_SID));
    dma_wmb();
    write32(wlan_dart_regs + WLAN_DART_TTBR(WLAN_SID), wlan_encode_ttbr(WLAN_PT_L1_PHYS));
    write32(wlan_dart_regs + WLAN_DART_TCR(WLAN_SID), WLAN_DART_TCR_TRANSLATE_ENABLE);
    dma_wmb();

    if (read32(wlan_dart_regs + WLAN_DART_TTBR(WLAN_SID)) !=
            wlan_encode_ttbr(WLAN_PT_L1_PHYS) ||
        read32(wlan_dart_regs + WLAN_DART_TCR(WLAN_SID)) !=
            WLAN_DART_TCR_TRANSLATE_ENABLE) {
        status = WLAN_ERR_READBACK;
        goto fail;
    }
    status = wlan_flush_sid1();
    if (status)
        goto fail;

    status = pcie_t602x_bcm4388_setup_port0(&wlan_mmio_ops, NULL);
    if (status != PCIE_T602X_BCM4388_OK) {
        printf("wlan-handoff: RID/MSI setup failed: %d\n", status);
        status = WLAN_ERR_PORT_SETUP;
        goto fail;
    }

    status = wlan_publish_descriptor(reservation_size);
    if (status) {
        printf("wlan-handoff: ABI v2 descriptor validation failed: %d\n", status);
        status = WLAN_ERR_READBACK;
        goto fail;
    }

    printf("wlan-handoff: SID 1 deny-all domain installed, ABI v2 descriptor %#llx, "
           "reservation %#llx+%#llx, L1 %#llx, MSI L2 %#llx\n",
           (unsigned long long)WLAN_DESCRIPTOR_PHYS,
           (unsigned long long)wlan_pt_carveout_phys, (unsigned long long)reservation_size,
           (unsigned long long)WLAN_PT_L1_PHYS, (unsigned long long)WLAN_PT_MSI_L2_PHYS);
    return WLAN_HANDOFF_OK;

fail:
    wlan_block_sid1();
    printf("wlan-handoff: failed (%d); Windows PCI0 profile is forbidden\n", status);
    return status;
}

#else

int wireless_handoff_init(u64 reservation_base, u64 reservation_size)
{
    UNUSED(reservation_base);
    UNUSED(reservation_size);
    return 0;
}

#endif
