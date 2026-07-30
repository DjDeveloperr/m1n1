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
#include "mcc.h"
#include "memory.h"
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
/*
 * ECAM: base + (bus << 20) + (device << 15) + (function << 12).
 *
 * Both BCM4388 functions are on ONE device: bus 1, device 0, function 0 is
 * Wi-Fi (14e4:4434) and function 1 is Bluetooth (14e4:5f72) -- verified by
 * live bus enumeration, not inferred.  Port 0's root port is bus 0 device 0
 * (which is why src/pcie.c derives its config base as
 * controller_config_base + (port << 15)), and it must already carry
 * secondary bus 1 or no configuration request reaches either function.
 */
#define WLAN_ROOT_PORT_CONFIG      WLAN_ECAM_BASE
#define WLAN_ENDPOINT_BUS          1U
#define WLAN_PCI_BRIDGE_BUS_NUMBER 0x18
#define WLAN_PCI_BRIDGE_SECONDARY  GENMASK(15, 8)
#define WLAN_PCI_BRIDGE_SUBORDINATE GENMASK(23, 16)
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
/* A latched fault is FLAG only; the lower fields are stale residue. */
#define WLAN_DART_ERROR_FLAG           BIT(31)
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
    WLAN_ERR_RESERVATION_NOT_CANONICAL = -15,
    WLAN_ERR_RESERVATION_CLAIMED = -16,
    WLAN_ERR_TABLE_READBACK = -17,
    WLAN_ERR_BUS_ROUTING = -18,
};

static u64 wlan_dart_regs;
static bool wlan_wrote_dart;
static u64 wlan_pt_carveout_phys;

#define WLAN_PT_L1_PHYS     (wlan_pt_carveout_phys + 0x0000)
#define WLAN_PT_MSI_L2_PHYS (wlan_pt_carveout_phys + 0x4000)
#define WLAN_DESCRIPTOR_PHYS \
    (wlan_pt_carveout_phys + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET)

static u64 wlan_physical_memory_top(void)
{
    return ALIGN_DOWN(cur_boot_args.phys_base, BIT(32)) + mem_size_actual;
}

/*
 * The single address both sides must agree on, bit for bit.
 *
 * This handoff is installed at EL2 before Mu exists, so there is no channel
 * over which Mu could hand m1n1 a base and none over which m1n1 could hand Mu
 * one.  Both sides therefore *derive* the same address from the same
 * boot_args inputs:
 *
 *     phys_top = ALIGN_DOWN(boot_args.phys_base, 4 GiB) + mem_size_actual
 *     base     = ALIGN_DOWN(phys_top - 0x10000, 0x4000)
 *
 * Mu's copy of this formula is NtasiDeriveWirelessReservation() in
 * mu-j414s-windows-unified,
 * Silicon/Apple/T602XFamilyPkg/Library/MemoryInitPeiLib/MemoryInitPeiLib.c:
 *
 *     PhysTop       = (SystemMemoryBase & ~(SIZE_4GB - 1)) + MemSizeActual;
 *     CandidateBase = (PhysTop - RESERVATION_SIZE) & ~(PAGE_SIZE - 1);
 *
 * with SystemMemoryBase/SystemMemorySize taken verbatim from the boot_args
 * m1n1 hands the guest (EarlySetup(),
 * Silicon/Apple/AppleSiliconPkg/PrePi/AdtParser.c sets
 * *SystemMemoryBase = BootArgs->phys_base and *SystemMemorySize =
 * BootArgs->mem_size) and MemSizeActual read from the same struct.
 *
 * The two agree only because ALIGN_DOWN(x, 4 GiB) collapses m1n1's own
 * phys_base and the hypervisor's rewritten guest phys_base (HV.load_raw()
 * sets tba.phys_base = u.heap_top) onto the same ram_base.  That is a
 * property of the current memory layout, not an invariant, so the host side
 * (proxyclient/m1n1/wireless_handoff.py) computes the derivation from BOTH
 * structs and refuses to launch if they ever disagree, and this function
 * refuses any base that is not the canonical one.  A caller that derives the
 * reservation any other way is rejected outright rather than silently
 * installing a deny-all domain at an address Mu will never look at.
 */
static u64 wlan_canonical_reservation_base(void)
{
    u64 physical_top = wlan_physical_memory_top();

    if (physical_top <= WLAN_PT_CARVEOUT_SIZE)
        return 0;

    return ALIGN_DOWN(physical_top - WLAN_PT_CARVEOUT_SIZE, WLAN_PT_ALIGNMENT);
}

static bool wlan_ranges_overlap(u64 a_base, u64 a_size, u64 b_base, u64 b_size)
{
    if (!a_size || !b_size)
        return false;
    return a_base < b_base + b_size && b_base < a_base + a_size;
}

/*
 * Prove the agreed address is not already owned by firmware.
 *
 * This is the one check Mu structurally cannot perform (see the UNVERIFIED
 * CAVEAT in NtasiDeriveWirelessReservation()): the TrustZone bounds live in
 * privileged MCC registers and the firmware-owned DRAM windows live in the
 * ADT's /defaults pmap-io-ranges, neither of which a guest can read.  m1n1
 * can read both, so m1n1 is where the derived address gets validated.
 *
 * Two classes of claim matter, and both are fatal:
 *
 *   - MCC TZ carveouts.  mcc_unmap_carveouts() removes these from m1n1's own
 *     page tables, so a memset() into one is a data abort at EL2 with no
 *     console recovery -- exactly the silent-hang class this project keeps
 *     eliminating.
 *   - pmap-io-ranges entries.  Those windows stay mapped (some as Normal-NC),
 *     so a write there succeeds and silently corrupts live firmware state.
 *     Measured on J414s: eight such windows sit in the top 4 MiB of DRAM,
 *     the highest ending at 0x103fffbc000, and one of them holds the disp0
 *     DART's real-time L1 table.
 *
 * Fails closed when the evidence is missing: if MCC never enumerated a
 * carveout, or /defaults has no pmap-io-ranges, this code cannot prove the
 * range is free and must not write to it.
 */
static bool wlan_range_is_claimed(u64 base, u64 size)
{
    const u32 *ranges;
    u32 length = 0;
    int node;

    if (mcc_carveout_count == 0) {
        printf("wlan-handoff: no MCC carveouts enumerated; cannot prove %#llx+%#llx is free\n",
               (unsigned long long)base, (unsigned long long)size);
        return true;
    }

    for (size_t index = 0; index < mcc_carveout_count; index++) {
        if (wlan_ranges_overlap(base, size, mcc_carveouts[index].base,
                                mcc_carveouts[index].size)) {
            printf("wlan-handoff: reservation %#llx+%#llx overlaps TZ carveout %#llx+%#llx\n",
                   (unsigned long long)base, (unsigned long long)size,
                   (unsigned long long)mcc_carveouts[index].base,
                   (unsigned long long)mcc_carveouts[index].size);
            return true;
        }
    }

    node = adt_path_offset(adt, "/defaults");
    if (node < 0) {
        printf("wlan-handoff: no /defaults node; cannot prove %#llx+%#llx is free\n",
               (unsigned long long)base, (unsigned long long)size);
        return true;
    }
    ranges = adt_getprop(adt, node, "pmap-io-ranges", &length);
    if (!ranges || length < 24) {
        printf("wlan-handoff: no pmap-io-ranges; cannot prove %#llx+%#llx is free\n",
               (unsigned long long)base, (unsigned long long)size);
        return true;
    }

    /* Six u32 per entry: base_lo, base_hi, size_lo, size_hi, flags, unused. */
    for (u32 entry = 0; (entry + 6) * 4 <= length; entry += 6) {
        u64 range_base = ranges[entry] | ((u64)ranges[entry + 1] << 32);
        u64 range_size = ranges[entry + 2] | ((u64)ranges[entry + 3] << 32);

        if (wlan_ranges_overlap(base, size, range_base, range_size)) {
            printf("wlan-handoff: reservation %#llx+%#llx overlaps firmware range %#llx+%#llx\n",
                   (unsigned long long)base, (unsigned long long)size,
                   (unsigned long long)range_base, (unsigned long long)range_size);
            return true;
        }
    }

    return false;
}

static int wlan_validate_reservation(u64 base, u64 size)
{
    u64 physical_top = wlan_physical_memory_top();
    u64 guest_top = cur_boot_args.phys_base + cur_boot_args.mem_size;
    u64 canonical = wlan_canonical_reservation_base();

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

    if (!canonical || base != canonical) {
        printf("wlan-handoff: reservation %#llx is not the canonical derivation %#llx "
               "(phys_base %#llx, mem_size_actual %#llx, phys_top %#llx); Mu would look "
               "elsewhere\n",
               (unsigned long long)base, (unsigned long long)canonical,
               (unsigned long long)cur_boot_args.phys_base, (unsigned long long)mem_size_actual,
               (unsigned long long)physical_top);
        return WLAN_ERR_RESERVATION_NOT_CANONICAL;
    }

    if (wlan_range_is_claimed(base, size))
        return WLAN_ERR_RESERVATION_CLAIMED;

    return WLAN_HANDOFF_OK;
}

static u64 wlan_endpoint_config(u32 function)
{
    return WLAN_ECAM_BASE + ((u64)WLAN_ENDPOINT_BUS << 20) + ((u64)function << 12);
}

static u32 wlan_pci_command(u32 function)
{
    return read32(wlan_endpoint_config(function) + 4) & 0xffff;
}

static u32 wlan_pci_identity(u32 function)
{
    return read32(wlan_endpoint_config(function));
}

/*
 * Prove the endpoint is addressable before believing anything read from it.
 *
 * A PCI-to-PCI bridge forwards a configuration request only when the target
 * bus falls in [secondary, subordinate].  A root port left at
 * secondary = subordinate = 0 forwards nothing, and every config read of the
 * endpoint returns 0xffffffff -- indistinguishable, from here, from a missing
 * or wrong device.  m1n1 used to leave the root ports exactly like that, so
 * this check reported "function 0 identity 0xffffffff" and the handoff failed
 * with WLAN_ERR_ENDPOINT_ID even though the silicon was fine and the link was
 * up.  src/pcie.c now programs the bus numbers on the wireless profile
 * (pcie_program_bridge_bus_numbers()); this verifies the routing actually
 * exists so a genuine identity mismatch and an unroutable bus can never again
 * be confused for each other.
 */
static int wlan_check_bus_routing(void)
{
    u32 bus_number = read32(WLAN_ROOT_PORT_CONFIG + WLAN_PCI_BRIDGE_BUS_NUMBER);
    u32 secondary = FIELD_GET(WLAN_PCI_BRIDGE_SECONDARY, bus_number);
    u32 subordinate = FIELD_GET(WLAN_PCI_BRIDGE_SUBORDINATE, bus_number);

    if (bus_number == 0xffffffffU) {
        printf("wlan-handoff: APCIE port 0 root port is not responding (bus reg %#x); "
               "was pcie_init_wireless() called?\n",
               bus_number);
        return WLAN_ERR_BUS_ROUTING;
    }
    if (secondary != WLAN_ENDPOINT_BUS || subordinate < WLAN_ENDPOINT_BUS) {
        printf("wlan-handoff: APCIE port 0 does not route bus %u (secondary %u, "
               "subordinate %u); endpoint config space is unreachable\n",
               WLAN_ENDPOINT_BUS, secondary, subordinate);
        return WLAN_ERR_BUS_ROUTING;
    }
    return WLAN_HANDOFF_OK;
}

static int wlan_check_endpoints_quiescent(void)
{
    /*
     * Index is the PCI function number of bus 1 device 0: function 0 is Wi-Fi,
     * function 1 is the Bluetooth function of the same device.  Both IDs are
     * little-endian (device << 16) | vendor, i.e. 14e4:4434 and 14e4:5f72.
     */
    static const u32 expected_identity[2] = {WLAN_WIFI_ID, WLAN_BT_ID};
    int status = wlan_check_bus_routing();

    if (status)
        return status;

    for (u32 function = 0; function < 2; function++) {
        u32 identity = wlan_pci_identity(function);
        u32 command = wlan_pci_command(function);

        if (identity != expected_identity[function]) {
            printf("wlan-handoff: function %u identity %#x, expected %#x\n", function,
                   identity, expected_identity[function]);
            return WLAN_ERR_ENDPOINT_ID;
        }
        /*
         * The quiescence rule is exactly one bit: Bus Master Enable
         * (COMMAND bit 2) must be clear, because a bus-mastering endpoint can
         * DMA through SID 1 while this code is rewriting its translation
         * tables.  Memory/IO decode, assigned BARs, and anything else an
         * earlier enumerator left behind are all accepted -- command is NOT
         * required to be zero.  Mu and Windows both enumerate and assign BARs,
         * and neither sets Bus Master Enable before pci.sys does.
         */
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
    /*
     * A latched fault is indicated by ERROR.FLAG (bit 31), not by the register
     * being non-zero.  The lower fields (SID, WRITE_nREAD, and the fault-type
     * bits) retain residue from whatever transaction last set them and are
     * meaningless while FLAG is clear -- m1n1's own DART driver gates on
     * exactly this bit (see proxyclient/m1n1/hw/dart8110.py, which only
     * reports when ERROR.reg.FLAG is set).
     *
     * Measured on J414s: ERROR reads 0x10700000 (FLAG=0, SID=7,
     * WRITE_nREAD=1, no fault-type bits) with ERROR_STREAMS = 0, and the word
     * is not write-1-to-clear.  Testing the whole word therefore rejected the
     * handoff permanently over stale residue with no fault present.
     */
    u32 dart_error = read32(wlan_dart_regs + WLAN_DART_ERROR);
    u32 dart_error_streams = read32(wlan_dart_regs + WLAN_DART_ERROR_STREAMS);

    if ((dart_error & WLAN_DART_ERROR_FLAG) || dart_error_streams != 0) {
        printf("wlan-handoff: pre-existing DART fault (error=%#x streams=%#x)\n", dart_error,
               dart_error_streams);
        return WLAN_ERR_PREEXISTING_FAULT;
    }
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

/*
 * Push a just-written reservation range all the way out to DRAM.
 *
 * dma_wmb() ("dmb oshst") is an ORDERING barrier and nothing more: it never
 * moves a dirty line out of the CPU's caches.  m1n1 maps this DRAM as
 * MAIR_IDX_NORMAL write-back (mmu_add_default_mappings() identity-maps
 * ram_base..ram_base+mem_size_actual), so every store below lands in the data
 * cache and, with only a dmb, may stay there indefinitely.
 *
 * That is fatal for this specific consumer.  Mu validates the descriptor from
 * MemoryInitPeiLib's NtasiValidateWirelessHandoffV2() -- which runs BEFORE
 * ArmConfigureMmu(), i.e. with SCTLR_EL1.M clear, so all of its loads are
 * Device-nGnRnE, bypass the data cache entirely and see raw DRAM.  Dirty lines
 * in m1n1's cache are invisible to it: it reads zeroes and withholds wireless.
 *
 * Clean AND invalidate to the Point of Coherency (not just clean) is
 * deliberate.  It leaves nothing cached, so the read-back verification and CRC
 * computations that follow necessarily re-fetch from DRAM -- a successful
 * validation then proves the bytes really landed there rather than proving
 * m1n1 can read its own cache.
 *
 * The trailing "dsb sy" is required: CACHE_RANGE_OP() in src/memory.c issues
 * the "dc civac" loop with no completion barrier of its own.
 */
static void wlan_publish_range(u64 address, size_t length)
{
    sysop("dsb ish");
    dc_civac_range((void *)address, length);
    sysop("dsb sy");
}

static int wlan_build_tables(void)
{
    volatile u64 *l1 = (volatile u64 *)WLAN_PT_L1_PHYS;
    volatile u64 *msi_l2 = (volatile u64 *)WLAN_PT_MSI_L2_PHYS;
    u64 expected_l1 = wlan_encode_pte(WLAN_PT_MSI_L2_PHYS);
    u64 expected_msi_l2 = wlan_encode_pte(WLAN_MSI_DOORBELL_PAGE);

    memset((void *)wlan_pt_carveout_phys, 0, WLAN_PT_CARVEOUT_SIZE);
    msi_l2[WLAN_MSI_L2_INDEX] = expected_msi_l2;
    l1[WLAN_MSI_L1_INDEX] = expected_l1;

    /*
     * The whole 64 KiB, not just the two live PTEs: the all-zero client L2
     * page and the descriptor page's zero padding are both read back by
     * AppleDart.sys, and the two table pages are CRC-covered in full.
     */
    wlan_publish_range(wlan_pt_carveout_phys, WLAN_PT_CARVEOUT_SIZE);

    if (l1[WLAN_MSI_L1_INDEX] != expected_l1 || msi_l2[WLAN_MSI_L2_INDEX] != expected_msi_l2 ||
        l1[0] != 0 || msi_l2[0] != 0) {
        printf("wlan-handoff: table read-back from DRAM failed (l1[%d]=%#llx msi_l2[%d]=%#llx)\n",
               WLAN_MSI_L1_INDEX, (unsigned long long)l1[WLAN_MSI_L1_INDEX], WLAN_MSI_L2_INDEX,
               (unsigned long long)msi_l2[WLAN_MSI_L2_INDEX]);
        return WLAN_ERR_TABLE_READBACK;
    }

    return WLAN_HANDOFF_OK;
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
        .physical_memory_top = wlan_physical_memory_top(),
        .dart_base = WLAN_DART0_BASE,
        .l1_physical = WLAN_PT_L1_PHYS,
        .msi_l2_physical = WLAN_PT_MSI_L2_PHYS,
        .descriptor_physical = WLAN_DESCRIPTOR_PHYS,
    };

    /*
     * CRC byte ranges, exactly as docs/apple-bcm-wireless-dart-handoff-abi-spec.md
     * specifies and as Mu's NtasiValidateWirelessHandoffV2() recomputes them:
     *   l1_crc32       reservation + 0x0000, 0x4000 bytes (the whole L1 page)
     *   msi_l2_crc32   reservation + 0x4000, 0x4000 bytes (the whole MSI L2 page)
     *   descriptor_crc32  all 96 descriptor bytes with descriptor_crc32 itself
     *                     zeroed in place -- the field is not excluded from the
     *                     length.
     * Both table pages were already cleaned to DRAM by wlan_build_tables(), so
     * these reads observe the same bytes Mu and Windows will.
     */
    descriptor.l1_crc32 = wireless_handoff_v2_crc32(
        (const void *)WLAN_PT_L1_PHYS, WIRELESS_HANDOFF_V2_PAGE_SIZE);
    descriptor.msi_l2_crc32 = wireless_handoff_v2_crc32(
        (const void *)WLAN_PT_MSI_L2_PHYS, WIRELESS_HANDOFF_V2_PAGE_SIZE);
    descriptor.descriptor_crc32 = 0;
    descriptor.descriptor_crc32 = wireless_handoff_v2_crc32(
        &descriptor, sizeof(descriptor));
    memcpy((void *)WLAN_DESCRIPTOR_PHYS, &descriptor, sizeof(descriptor));
    /*
     * The whole descriptor page, so the CRC-covered struct and the zero
     * padding behind it reach DRAM together.  The validation immediately below
     * then reads the descriptor back out of DRAM, not out of the cache.
     */
    wlan_publish_range(wlan_pt_carveout_phys + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET,
                       WIRELESS_HANDOFF_V2_PAGE_SIZE);
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

    status = wlan_build_tables();
    if (status)
        return status;

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
    printf("wlan-handoff: canonical derivation confirmed (phys_top %#llx, guest_top %#llx); "
           "Mu must derive %#llx\n",
           (unsigned long long)wlan_physical_memory_top(),
           (unsigned long long)(cur_boot_args.phys_base + cur_boot_args.mem_size),
           (unsigned long long)wlan_canonical_reservation_base());
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
