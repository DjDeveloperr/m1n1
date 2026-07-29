/* SPDX-License-Identifier: MIT */

#include "bcm4388_handoff.h"

#ifdef BCM4388_HANDOFF_HOST_TEST
#include <stddef.h>
#include <string.h>
#else
#include "string.h"
#endif

#define BCM4388_HANDOFF_PTE_OFFSET_MASK        UINT64_C(0x000000fffffc00)
#define BCM4388_HANDOFF_PTE_SP_END             UINT64_C(0x000fff0000000000)
#define BCM4388_HANDOFF_PTE_VALID              UINT64_C(0x0000000000000001)
#define BCM4388_HANDOFF_DESCRIPTOR_HEADER_SIZE UINT16_C(16)

_Static_assert(sizeof(struct bcm4388_handoff_endpoint_v1) == 32, "BCM4388 endpoint wire size");
_Static_assert(sizeof(struct bcm4388_handoff_descriptor_v1) == 384,
               "BCM4388 handoff descriptor wire size");
_Static_assert(BCM4388_HANDOFF_CLIENT_IOVA_BASE / BCM4388_HANDOFF_L2_WINDOW_SIZE ==
                   BCM4388_HANDOFF_CLIENT_L1_INDEX,
               "client L1 index");
_Static_assert(BCM4388_HANDOFF_MSI_PAGE_IOVA / BCM4388_HANDOFF_L2_WINDOW_SIZE ==
                   BCM4388_HANDOFF_MSI_L1_INDEX,
               "MSI L1 index");
_Static_assert((BCM4388_HANDOFF_MSI_PAGE_IOVA % BCM4388_HANDOFF_L2_WINDOW_SIZE) /
                       BCM4388_HANDOFF_PAGE_SIZE ==
                   BCM4388_HANDOFF_MSI_L2_INDEX,
               "MSI L2 index");
_Static_assert(BCM4388_HANDOFF_MSI_PAGE_IOVA + UINT64_C(0x3000) ==
                   BCM4388_HANDOFF_MSI_DOORBELL_IOVA,
               "MSI doorbell page");

struct bcm4388_handoff_preflight {
    u32 wifi_identity;
    u32 wifi_command;
    u32 bluetooth_identity;
    u32 bluetooth_command;
    u32 params1;
    u32 params2;
    u32 params3;
    u32 params4;
    u32 protect;
    u32 tcr;
    u32 ttbr;
    u32 tlb;
    u32 faults[5];
    u32 sid_count;
    u32 pa_width;
    u32 encoded_ttbr;
};

static const u64 dart_fault_offsets[5] = {
    BCM4388_HANDOFF_DART_FAULT0,        BCM4388_HANDOFF_DART_FAULT1,
    BCM4388_HANDOFF_DART_FAULT_ADDR_LO, BCM4388_HANDOFF_DART_FAULT_ADDR_HI,
    BCM4388_HANDOFF_DART_FAULT_STATUS,
};

static u64 endpoint_config_address(u32 bus, u32 device, u32 function)
{
    return BCM4388_HANDOFF_ECAM_BASE + ((u64)bus << 20) + ((u64)device << 15) +
           ((u64)function << 12);
}

static int io_read32(const struct bcm4388_handoff_io *io, u64 address, u32 *value)
{
    if (io->read32(io->context, address, value) != 0)
        return BCM4388_HANDOFF_ERR_IO;
    return BCM4388_HANDOFF_OK;
}

static int io_write32(const struct bcm4388_handoff_io *io, u64 address, u32 value)
{
    if (io->write32(io->context, address, value) != 0)
        return BCM4388_HANDOFF_ERR_IO;
    return BCM4388_HANDOFF_OK;
}

static int io_barrier(const struct bcm4388_handoff_io *io)
{
    if (io->barrier(io->context) != 0)
        return BCM4388_HANDOFF_ERR_IO;
    return BCM4388_HANDOFF_OK;
}

static int read_exact(const struct bcm4388_handoff_io *io, u64 address, u32 expected)
{
    u32 value = 0;
    int status = io_read32(io, address, &value);

    if (status != BCM4388_HANDOFF_OK)
        return status;
    return value == expected ? BCM4388_HANDOFF_OK : BCM4388_HANDOFF_ERR_READBACK;
}

static bool add_overflows(u64 base, u64 length)
{
    return length == 0 || base > UINT64_MAX - length;
}

static bool ranges_overlap(u64 first, u64 first_length, u64 second, u64 second_length)
{
    return first < second + second_length && second < first + first_length;
}

static bool page_is_zero(const void *page)
{
    const u64 *words = page;

    for (u32 index = 0; index < BCM4388_HANDOFF_TABLE_ENTRIES; index++) {
        if (words[index] != 0)
            return false;
    }
    return true;
}

static void zero_page(void *page)
{
    memset(page, 0, (size_t)BCM4388_HANDOFF_PAGE_SIZE);
}

static int validate_pages(const struct bcm4388_handoff_pages *pages)
{
    const u64 physical[4] = {
        pages->descriptor_physical,
        pages->l1_physical,
        pages->client_l2_physical,
        pages->msi_l2_physical,
    };
    const uintptr_t virtual_address[4] = {
        (uintptr_t)pages->descriptor_page,
        (uintptr_t)pages->l1,
        (uintptr_t)pages->client_l2,
        (uintptr_t)pages->msi_l2,
    };

    if (!pages->descriptor_page || !pages->l1 || !pages->client_l2 || !pages->msi_l2)
        return BCM4388_HANDOFF_ERR_ARGUMENT;

    for (u32 index = 0; index < 4; index++) {
        if (((physical[index] | (u64)virtual_address[index]) &
             (BCM4388_HANDOFF_PAGE_SIZE - UINT64_C(1))) != 0)
            return BCM4388_HANDOFF_ERR_ALIGNMENT;
        if (add_overflows(physical[index], BCM4388_HANDOFF_PAGE_SIZE))
            return BCM4388_HANDOFF_ERR_RANGE;
    }

    for (u32 first = 0; first < 4; first++) {
        for (u32 second = first + 1; second < 4; second++) {
            if (virtual_address[first] == virtual_address[second] ||
                ranges_overlap(physical[first], BCM4388_HANDOFF_PAGE_SIZE, physical[second],
                               BCM4388_HANDOFF_PAGE_SIZE))
                return BCM4388_HANDOFF_ERR_RANGE;
        }
    }

    if (!page_is_zero(pages->descriptor_page) || !page_is_zero(pages->l1) ||
        !page_is_zero(pages->client_l2) || !page_is_zero(pages->msi_l2))
        return BCM4388_HANDOFF_ERR_TABLE_NOT_EMPTY;
    return BCM4388_HANDOFF_OK;
}

static int encode_table_pointer(u64 physical, u64 *entry)
{
    u64 page;
    u64 field;

    if (!entry)
        return BCM4388_HANDOFF_ERR_ARGUMENT;
    if ((physical & (BCM4388_HANDOFF_PAGE_SIZE - UINT64_C(1))) != 0)
        return BCM4388_HANDOFF_ERR_ALIGNMENT;
    page = physical >> BCM4388_HANDOFF_PAGE_SHIFT;
    field = (page << 10) & BCM4388_HANDOFF_PTE_OFFSET_MASK;
    if ((field >> 10) != page)
        return BCM4388_HANDOFF_ERR_RANGE;
    *entry = field | BCM4388_HANDOFF_PTE_VALID;
    return BCM4388_HANDOFF_OK;
}

static int encode_leaf(u64 physical, u64 *entry)
{
    int status = encode_table_pointer(physical, entry);

    if (status != BCM4388_HANDOFF_OK)
        return status;
    *entry |= BCM4388_HANDOFF_PTE_SP_END;
    return BCM4388_HANDOFF_OK;
}

static int encode_ttbr(u64 physical, u32 pa_width, u32 *encoded)
{
    u64 page;
    u64 maximum_page = BCM4388_HANDOFF_DART_TTBR_ADDRESS >> 2;

    if (!encoded)
        return BCM4388_HANDOFF_ERR_ARGUMENT;
    if ((physical & (BCM4388_HANDOFF_PAGE_SIZE - UINT64_C(1))) != 0)
        return BCM4388_HANDOFF_ERR_ALIGNMENT;
    if (pa_width < BCM4388_HANDOFF_PAGE_SHIFT || pa_width > 63 || (physical >> pa_width) != 0)
        return BCM4388_HANDOFF_ERR_RANGE;
    page = physical >> BCM4388_HANDOFF_PAGE_SHIFT;
    if (page > maximum_page)
        return BCM4388_HANDOFF_ERR_RANGE;
    *encoded =
        (u32)((page << 2) & BCM4388_HANDOFF_DART_TTBR_ADDRESS) | BCM4388_HANDOFF_DART_TTBR_VALID;
    return BCM4388_HANDOFF_OK;
}

static int read_endpoint(const struct bcm4388_handoff_io *io, u32 bus, u32 device, u32 function,
                         u32 expected_identity, u32 *identity, u32 *command)
{
    const u64 config = endpoint_config_address(bus, device, function);
    u32 command_second = 0;
    int status;

    status = io_read32(io, config, identity);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    if (*identity != expected_identity)
        return BCM4388_HANDOFF_ERR_IDENTITY;

    status = io_read32(io, config + UINT64_C(4), command);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_read32(io, config + UINT64_C(4), &command_second);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    if (((*command | command_second) & BCM4388_HANDOFF_PCI_COMMAND_BME) != 0)
        return BCM4388_HANDOFF_ERR_BME_ACTIVE;
    if (((*command ^ command_second) & UINT32_C(0x0000ffff)) != 0)
        return BCM4388_HANDOFF_ERR_BME_UNSTABLE;
    return BCM4388_HANDOFF_OK;
}

static int read_faults(const struct bcm4388_handoff_io *io, u32 faults[5])
{
    for (u32 index = 0; index < 5; index++) {
        int status =
            io_read32(io, BCM4388_HANDOFF_DART_BASE + dart_fault_offsets[index], &faults[index]);
        if (status != BCM4388_HANDOFF_OK)
            return status;
        if (faults[index] != 0)
            return BCM4388_HANDOFF_ERR_DART_FAULT;
    }
    return BCM4388_HANDOFF_OK;
}

static int require_msi_disabled_twice(const struct bcm4388_handoff_io *io)
{
    const u64 address = BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET;
    u32 first = 0;
    u32 second = 0;
    int status;

    status = io_read32(io, address, &first);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_read32(io, address, &second);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    return first == 0 && second == 0 ? BCM4388_HANDOFF_OK : BCM4388_HANDOFF_ERR_MSI_ENABLED;
}

static int preflight_rids(const struct bcm4388_handoff_io *io)
{
    const u64 base = BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;

    for (u32 index = 0; index < PCIE_T602X_PORT_RID2SID_ENTRY_COUNT; index++) {
        u32 value = 0;
        int status = io_read32(io, base + (u64)index * UINT64_C(4), &value);

        if (status != BCM4388_HANDOFF_OK)
            return status;
        if (index < 2 && value != 0)
            return BCM4388_HANDOFF_ERR_RID_OCCUPIED;
        if (index >= 2 && ((value & UINT32_C(0x0000ffff)) == BCM4388_HANDOFF_WIFI_RID ||
                           (value & UINT32_C(0x0000ffff)) == BCM4388_HANDOFF_BT_RID))
            return BCM4388_HANDOFF_ERR_RID_DUPLICATE;
    }
    return BCM4388_HANDOFF_OK;
}

static int validate_unique_rids(const struct bcm4388_handoff_io *io, u32 *rid0_after,
                                u32 *rid1_after)
{
    const u64 base = BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;
    u32 wifi_count = 0;
    u32 bluetooth_count = 0;

    for (u32 index = 0; index < PCIE_T602X_PORT_RID2SID_ENTRY_COUNT; index++) {
        u32 value = 0;
        int status = io_read32(io, base + (u64)index * UINT64_C(4), &value);

        if (status != BCM4388_HANDOFF_OK)
            return status;
        if (index == 0)
            *rid0_after = value;
        if (index == 1)
            *rid1_after = value;
        if ((value & UINT32_C(0x0000ffff)) == BCM4388_HANDOFF_WIFI_RID)
            wifi_count++;
        if ((value & UINT32_C(0x0000ffff)) == BCM4388_HANDOFF_BT_RID)
            bluetooth_count++;
    }

    if (*rid0_after != PCIE_T602X_BCM4388_WIFI_RID2SID ||
        *rid1_after != PCIE_T602X_BCM4388_BLUETOOTH_RID2SID)
        return BCM4388_HANDOFF_ERR_READBACK;
    if (wifi_count != 1 || bluetooth_count != 1)
        return BCM4388_HANDOFF_ERR_RID_DUPLICATE;
    return BCM4388_HANDOFF_OK;
}

static int read_preflight(const struct bcm4388_handoff_io *io,
                          const struct bcm4388_handoff_pages *pages,
                          struct bcm4388_handoff_preflight *preflight)
{
    u32 page_shift;
    u32 msi_first;
    u32 msi_second;
    int status;

    memset(preflight, 0, sizeof(*preflight));
    status = read_endpoint(io, BCM4388_HANDOFF_WIFI_BUS, BCM4388_HANDOFF_WIFI_DEVICE,
                           BCM4388_HANDOFF_WIFI_FUNCTION,
                           (BCM4388_HANDOFF_WIFI_DEVICE_ID << 16) | BCM4388_HANDOFF_WIFI_VENDOR,
                           &preflight->wifi_identity, &preflight->wifi_command);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = read_endpoint(io, BCM4388_HANDOFF_BT_BUS, BCM4388_HANDOFF_BT_DEVICE,
                           BCM4388_HANDOFF_BT_FUNCTION,
                           (BCM4388_HANDOFF_BT_DEVICE_ID << 16) | BCM4388_HANDOFF_BT_VENDOR,
                           &preflight->bluetooth_identity, &preflight->bluetooth_command);
    if (status != BCM4388_HANDOFF_OK)
        return status;

    status =
        io_read32(io, BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, &msi_first);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status =
        io_read32(io, BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, &msi_second);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    if (msi_first != 0 || msi_second != 0)
        return BCM4388_HANDOFF_ERR_MSI_ENABLED;

    status = preflight_rids(io);
    if (status != BCM4388_HANDOFF_OK)
        return status;

    status = io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS1,
                       &preflight->params1);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS2,
                       &preflight->params2);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS3,
                       &preflight->params3);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS4,
                       &preflight->params4);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PROTECT,
                       &preflight->protect);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status =
        io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1, &preflight->tcr);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status =
        io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1, &preflight->ttbr);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TLB_COMMAND,
                       &preflight->tlb);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = read_faults(io, preflight->faults);
    if (status != BCM4388_HANDOFF_OK)
        return status;

    page_shift = (preflight->params1 & BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_MASK) >>
                 BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_SHIFT;
    preflight->pa_width = (preflight->params3 & BCM4388_HANDOFF_DART_PARAMS3_PA_WIDTH_MASK) >>
                          BCM4388_HANDOFF_DART_PARAMS3_PA_WIDTH_SHIFT;
    preflight->sid_count = preflight->params4 & BCM4388_HANDOFF_DART_PARAMS4_SID_COUNT_MASK;
    if (page_shift != BCM4388_HANDOFF_PAGE_SHIFT ||
        preflight->pa_width < BCM4388_HANDOFF_PAGE_SHIFT || preflight->pa_width > 63 ||
        preflight->sid_count <= BCM4388_HANDOFF_SID)
        return BCM4388_HANDOFF_ERR_DART_PARAMS;
    if ((preflight->protect & BCM4388_HANDOFF_DART_PROTECT_TCR_TTBR) != 0)
        return BCM4388_HANDOFF_ERR_DART_PROTECTED;
    if (preflight->tcr != 0 || preflight->ttbr != 0)
        return BCM4388_HANDOFF_ERR_DART_OWNED;
    if ((preflight->tlb & BCM4388_HANDOFF_DART_TLB_BUSY) != 0)
        return BCM4388_HANDOFF_ERR_DART_BUSY;

    for (u32 index = 0; index < 4; index++) {
        const u64 physical[] = {
            pages->descriptor_physical,
            pages->l1_physical,
            pages->client_l2_physical,
            pages->msi_l2_physical,
        };
        if ((physical[index] >> preflight->pa_width) != 0)
            return BCM4388_HANDOFF_ERR_RANGE;
    }
    return encode_ttbr(pages->l1_physical, preflight->pa_width, &preflight->encoded_ttbr);
}

static int build_tables(const struct bcm4388_handoff_pages *pages)
{
    u64 client_pointer = 0;
    u64 msi_pointer = 0;
    u64 msi_leaf = 0;
    int status;

    status = encode_table_pointer(pages->client_l2_physical, &client_pointer);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = encode_table_pointer(pages->msi_l2_physical, &msi_pointer);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = encode_leaf(BCM4388_HANDOFF_MSI_PAGE_PHYSICAL, &msi_leaf);
    if (status != BCM4388_HANDOFF_OK)
        return status;

    pages->l1[BCM4388_HANDOFF_CLIENT_L1_INDEX] = client_pointer;
    pages->l1[BCM4388_HANDOFF_MSI_L1_INDEX] = msi_pointer;
    pages->msi_l2[BCM4388_HANDOFF_MSI_L2_INDEX] = msi_leaf;
    return BCM4388_HANDOFF_OK;
}

static int flush_sid1(const struct bcm4388_handoff_io *io, u32 poll_attempts, u32 *poll_reads)
{
    u32 command = 0;
    int status;

    status = io_barrier(io);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TLB_COMMAND,
                        BCM4388_HANDOFF_DART_TLB_FLUSH_SID1);
    if (status != BCM4388_HANDOFF_OK)
        return status;

    for (u32 poll = 0; poll < poll_attempts; poll++) {
        status =
            io_read32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TLB_COMMAND, &command);
        *poll_reads = poll + 1;
        if (status != BCM4388_HANDOFF_OK)
            return status;
        if ((command & BCM4388_HANDOFF_DART_TLB_BUSY) == 0)
            return BCM4388_HANDOFF_OK;
    }
    return BCM4388_HANDOFF_ERR_TLB_TIMEOUT;
}

static int install_dart(const struct bcm4388_handoff_io *io, u32 encoded_ttbr, u32 poll_attempts,
                        u32 *poll_reads)
{
    int status;

    status = io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_STREAM_DISABLE,
                        BCM4388_HANDOFF_DART_STREAM_SID1);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1, 0);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1, 0);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1, 0);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1, 0);
    if (status != BCM4388_HANDOFF_OK)
        return status;

    status = io_barrier(io);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status =
        io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1, encoded_ttbr);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status =
        read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1, encoded_ttbr);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1,
                        BCM4388_HANDOFF_DART_TCR_TRANSLATE);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1,
                        BCM4388_HANDOFF_DART_TCR_TRANSLATE);
    if (status != BCM4388_HANDOFF_OK)
        return status;

    status = flush_sid1(io, poll_attempts, poll_reads);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    status = io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_STREAM_ENABLE,
                        BCM4388_HANDOFF_DART_STREAM_SID1);
    if (status != BCM4388_HANDOFF_OK)
        return status;
    return io_barrier(io);
}

static int verify_bme_clear(const struct bcm4388_handoff_io *io, u32 *wifi_command,
                            u32 *bluetooth_command)
{
    u32 identity = 0;
    int status = read_endpoint(io, BCM4388_HANDOFF_WIFI_BUS, BCM4388_HANDOFF_WIFI_DEVICE,
                               BCM4388_HANDOFF_WIFI_FUNCTION,
                               (BCM4388_HANDOFF_WIFI_DEVICE_ID << 16) | BCM4388_HANDOFF_WIFI_VENDOR,
                               &identity, wifi_command);

    if (status != BCM4388_HANDOFF_OK)
        return status;
    return read_endpoint(io, BCM4388_HANDOFF_BT_BUS, BCM4388_HANDOFF_BT_DEVICE,
                         BCM4388_HANDOFF_BT_FUNCTION,
                         (BCM4388_HANDOFF_BT_DEVICE_ID << 16) | BCM4388_HANDOFF_BT_VENDOR,
                         &identity, bluetooth_command);
}

u32 bcm4388_handoff_crc32(const void *data, u32 length)
{
    const u8 *bytes = data;
    u32 crc = UINT32_C(0xffffffff);

    if (!data && length != 0)
        return 0;
    for (u32 index = 0; index < length; index++) {
        crc ^= bytes[index];
        for (u32 bit = 0; bit < 8; bit++) {
            u32 mask = UINT32_C(0) - (crc & UINT32_C(1));
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    return ~crc;
}

static bool descriptor_ranges_valid(const struct bcm4388_handoff_descriptor_v1 *descriptor)
{
    const u64 physical[4] = {
        descriptor->descriptor_physical,
        descriptor->l1_physical,
        descriptor->client_l2_physical,
        descriptor->msi_l2_physical,
    };
    const u64 length[4] = {
        descriptor->descriptor_length,
        descriptor->l1_length,
        descriptor->client_l2_length,
        descriptor->msi_l2_length,
    };

    for (u32 index = 0; index < 4; index++) {
        if (length[index] != BCM4388_HANDOFF_PAGE_SIZE ||
            (physical[index] & (BCM4388_HANDOFF_PAGE_SIZE - UINT64_C(1))) != 0 ||
            add_overflows(physical[index], length[index]))
            return false;
    }
    for (u32 first = 0; first < 4; first++) {
        for (u32 second = first + 1; second < 4; second++) {
            if (ranges_overlap(physical[first], length[first], physical[second], length[second]))
                return false;
        }
    }
    return true;
}

int bcm4388_handoff_descriptor_validate(const struct bcm4388_handoff_descriptor_v1 *descriptor)
{
    struct bcm4388_handoff_descriptor_v1 copy;
    u32 expected_ttbr = 0;
    u32 checksum;

    if (!descriptor)
        return BCM4388_HANDOFF_ERR_ARGUMENT;
    if (descriptor->signature != BCM4388_HANDOFF_SIGNATURE ||
        descriptor->version != BCM4388_HANDOFF_VERSION ||
        descriptor->header_size != BCM4388_HANDOFF_DESCRIPTOR_HEADER_SIZE ||
        descriptor->total_size != sizeof(*descriptor) || descriptor->generation == 0 ||
        descriptor->flags != BCM4388_HANDOFF_REQUIRED_FLAGS ||
        descriptor->sid != BCM4388_HANDOFF_SID || descriptor->chip_id != BCM4388_HANDOFF_CHIP_ID ||
        descriptor->page_shift != BCM4388_HANDOFF_PAGE_SHIFT ||
        descriptor->sid_count <= BCM4388_HANDOFF_SID ||
        descriptor->pa_width < BCM4388_HANDOFF_PAGE_SHIFT || descriptor->pa_width > 63 ||
        descriptor->ecam_base != BCM4388_HANDOFF_ECAM_BASE ||
        descriptor->port_base != BCM4388_HANDOFF_PORT_BASE ||
        descriptor->dart_base != BCM4388_HANDOFF_DART_BASE)
        return BCM4388_HANDOFF_ERR_DESCRIPTOR;

    if (descriptor->wifi.identity !=
            ((BCM4388_HANDOFF_WIFI_DEVICE_ID << 16) | BCM4388_HANDOFF_WIFI_VENDOR) ||
        descriptor->wifi.segment != 0 || descriptor->wifi.bus != BCM4388_HANDOFF_WIFI_BUS ||
        descriptor->wifi.device != BCM4388_HANDOFF_WIFI_DEVICE ||
        descriptor->wifi.function != BCM4388_HANDOFF_WIFI_FUNCTION ||
        descriptor->wifi.vendor_id != BCM4388_HANDOFF_WIFI_VENDOR ||
        descriptor->wifi.device_id != BCM4388_HANDOFF_WIFI_DEVICE_ID ||
        descriptor->wifi.rid != BCM4388_HANDOFF_WIFI_RID ||
        (descriptor->wifi.command_before & BCM4388_HANDOFF_PCI_COMMAND_BME) != 0 ||
        (descriptor->wifi.command_after & BCM4388_HANDOFF_PCI_COMMAND_BME) != 0 ||
        descriptor->bluetooth.identity !=
            ((BCM4388_HANDOFF_BT_DEVICE_ID << 16) | BCM4388_HANDOFF_BT_VENDOR) ||
        descriptor->bluetooth.segment != 0 || descriptor->bluetooth.bus != BCM4388_HANDOFF_BT_BUS ||
        descriptor->bluetooth.device != BCM4388_HANDOFF_BT_DEVICE ||
        descriptor->bluetooth.function != BCM4388_HANDOFF_BT_FUNCTION ||
        descriptor->bluetooth.vendor_id != BCM4388_HANDOFF_BT_VENDOR ||
        descriptor->bluetooth.device_id != BCM4388_HANDOFF_BT_DEVICE_ID ||
        descriptor->bluetooth.rid != BCM4388_HANDOFF_BT_RID ||
        (descriptor->bluetooth.command_before & BCM4388_HANDOFF_PCI_COMMAND_BME) != 0 ||
        (descriptor->bluetooth.command_after & BCM4388_HANDOFF_PCI_COMMAND_BME) != 0)
        return BCM4388_HANDOFF_ERR_DESCRIPTOR;

    if (encode_ttbr(descriptor->l1_physical, descriptor->pa_width, &expected_ttbr) !=
            BCM4388_HANDOFF_OK ||
        !descriptor_ranges_valid(descriptor) ||
        descriptor->client_iova_base != BCM4388_HANDOFF_CLIENT_IOVA_BASE ||
        descriptor->client_iova_limit != BCM4388_HANDOFF_CLIENT_IOVA_LIMIT ||
        descriptor->wifi_iova_base != BCM4388_HANDOFF_WIFI_IOVA_BASE ||
        descriptor->wifi_iova_limit != BCM4388_HANDOFF_WIFI_IOVA_LIMIT ||
        descriptor->bluetooth_iova_base != BCM4388_HANDOFF_BT_IOVA_BASE ||
        descriptor->bluetooth_iova_limit != BCM4388_HANDOFF_BT_IOVA_LIMIT ||
        descriptor->msi_doorbell_iova != BCM4388_HANDOFF_MSI_DOORBELL_IOVA ||
        descriptor->msi_page_iova != BCM4388_HANDOFF_MSI_PAGE_IOVA ||
        descriptor->msi_page_physical != BCM4388_HANDOFF_MSI_PAGE_PHYSICAL ||
        ((descriptor->params1 & BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_MASK) >>
         BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_SHIFT) != BCM4388_HANDOFF_PAGE_SHIFT ||
        ((descriptor->params3 & BCM4388_HANDOFF_DART_PARAMS3_PA_WIDTH_MASK) >>
         BCM4388_HANDOFF_DART_PARAMS3_PA_WIDTH_SHIFT) != descriptor->pa_width ||
        (descriptor->params4 & BCM4388_HANDOFF_DART_PARAMS4_SID_COUNT_MASK) !=
            descriptor->sid_count ||
        (descriptor->protect_before & BCM4388_HANDOFF_DART_PROTECT_TCR_TTBR) != 0 ||
        (descriptor->tlb_before & BCM4388_HANDOFF_DART_TLB_BUSY) != 0 ||
        descriptor->tcr_before != 0 || descriptor->ttbr_before != 0 ||
        descriptor->tcr_after != BCM4388_HANDOFF_DART_TCR_TRANSLATE ||
        descriptor->ttbr_after != expected_ttbr ||
        descriptor->rid0_after != PCIE_T602X_BCM4388_WIFI_RID2SID ||
        descriptor->rid1_after != PCIE_T602X_BCM4388_BLUETOOTH_RID2SID ||
        descriptor->msi_config_after != 0 || descriptor->poll_reads == 0 ||
        descriptor->l1_crc32 == 0 || descriptor->client_l2_crc32 == 0 ||
        descriptor->msi_l2_crc32 == 0)
        return BCM4388_HANDOFF_ERR_DESCRIPTOR;
    for (u32 index = 0; index < 5; index++) {
        if (descriptor->faults_before[index] != 0 || descriptor->faults_after[index] != 0)
            return BCM4388_HANDOFF_ERR_DESCRIPTOR;
    }
    if (descriptor->reserved != 0)
        return BCM4388_HANDOFF_ERR_DESCRIPTOR;

    memcpy(&copy, descriptor, sizeof(copy));
    checksum = copy.checksum;
    copy.checksum = 0;
    if (bcm4388_handoff_crc32(&copy, (u32)sizeof(copy)) != checksum)
        return BCM4388_HANDOFF_ERR_DESCRIPTOR;
    return BCM4388_HANDOFF_OK;
}

static void fill_endpoint(struct bcm4388_handoff_endpoint_v1 *endpoint, u32 identity,
                          u32 command_before, u32 command_after, u32 bus, u32 function, u32 vendor,
                          u32 device_id, u32 rid)
{
    endpoint->identity = identity;
    endpoint->command_before = command_before;
    endpoint->command_after = command_after;
    endpoint->segment = 0;
    endpoint->rid = (u16)rid;
    endpoint->vendor_id = (u16)vendor;
    endpoint->device_id = (u16)device_id;
    endpoint->bus = (u8)bus;
    endpoint->device = 0;
    endpoint->function = (u8)function;
}

static int publish_descriptor(const struct bcm4388_handoff_pages *pages,
                              const struct bcm4388_handoff_preflight *preflight, u64 generation,
                              u32 poll_reads, u32 wifi_command_after, u32 bluetooth_command_after,
                              u32 rid0_after, u32 rid1_after, const u32 faults_after[5])
{
    struct bcm4388_handoff_descriptor_v1 descriptor = {0};
    struct bcm4388_handoff_descriptor_v1 *published = pages->descriptor_page;

    descriptor.signature = BCM4388_HANDOFF_SIGNATURE;
    descriptor.version = BCM4388_HANDOFF_VERSION;
    descriptor.header_size = BCM4388_HANDOFF_DESCRIPTOR_HEADER_SIZE;
    descriptor.total_size = sizeof(descriptor);
    descriptor.generation = generation;
    descriptor.flags = BCM4388_HANDOFF_REQUIRED_FLAGS;
    descriptor.sid = BCM4388_HANDOFF_SID;
    descriptor.chip_id = BCM4388_HANDOFF_CHIP_ID;
    descriptor.page_shift = BCM4388_HANDOFF_PAGE_SHIFT;
    descriptor.sid_count = preflight->sid_count;
    descriptor.pa_width = preflight->pa_width;
    descriptor.ecam_base = BCM4388_HANDOFF_ECAM_BASE;
    descriptor.port_base = BCM4388_HANDOFF_PORT_BASE;
    descriptor.dart_base = BCM4388_HANDOFF_DART_BASE;

    fill_endpoint(&descriptor.wifi, preflight->wifi_identity, preflight->wifi_command,
                  wifi_command_after, BCM4388_HANDOFF_WIFI_BUS, BCM4388_HANDOFF_WIFI_FUNCTION,
                  BCM4388_HANDOFF_WIFI_VENDOR, BCM4388_HANDOFF_WIFI_DEVICE_ID,
                  BCM4388_HANDOFF_WIFI_RID);
    fill_endpoint(&descriptor.bluetooth, preflight->bluetooth_identity,
                  preflight->bluetooth_command, bluetooth_command_after, BCM4388_HANDOFF_BT_BUS,
                  BCM4388_HANDOFF_BT_FUNCTION, BCM4388_HANDOFF_BT_VENDOR,
                  BCM4388_HANDOFF_BT_DEVICE_ID, BCM4388_HANDOFF_BT_RID);

    descriptor.descriptor_physical = pages->descriptor_physical;
    descriptor.descriptor_length = BCM4388_HANDOFF_PAGE_SIZE;
    descriptor.l1_physical = pages->l1_physical;
    descriptor.l1_length = BCM4388_HANDOFF_PAGE_SIZE;
    descriptor.client_l2_physical = pages->client_l2_physical;
    descriptor.client_l2_length = BCM4388_HANDOFF_PAGE_SIZE;
    descriptor.msi_l2_physical = pages->msi_l2_physical;
    descriptor.msi_l2_length = BCM4388_HANDOFF_PAGE_SIZE;
    descriptor.client_iova_base = BCM4388_HANDOFF_CLIENT_IOVA_BASE;
    descriptor.client_iova_limit = BCM4388_HANDOFF_CLIENT_IOVA_LIMIT;
    descriptor.wifi_iova_base = BCM4388_HANDOFF_WIFI_IOVA_BASE;
    descriptor.wifi_iova_limit = BCM4388_HANDOFF_WIFI_IOVA_LIMIT;
    descriptor.bluetooth_iova_base = BCM4388_HANDOFF_BT_IOVA_BASE;
    descriptor.bluetooth_iova_limit = BCM4388_HANDOFF_BT_IOVA_LIMIT;
    descriptor.msi_doorbell_iova = BCM4388_HANDOFF_MSI_DOORBELL_IOVA;
    descriptor.msi_page_iova = BCM4388_HANDOFF_MSI_PAGE_IOVA;
    descriptor.msi_page_physical = BCM4388_HANDOFF_MSI_PAGE_PHYSICAL;

    descriptor.params1 = preflight->params1;
    descriptor.params2 = preflight->params2;
    descriptor.params3 = preflight->params3;
    descriptor.params4 = preflight->params4;
    descriptor.protect_before = preflight->protect;
    descriptor.tcr_before = preflight->tcr;
    descriptor.ttbr_before = preflight->ttbr;
    descriptor.tlb_before = preflight->tlb;
    memcpy(descriptor.faults_before, preflight->faults, sizeof(descriptor.faults_before));
    descriptor.tcr_after = BCM4388_HANDOFF_DART_TCR_TRANSLATE;
    descriptor.ttbr_after = preflight->encoded_ttbr;
    descriptor.rid0_after = rid0_after;
    descriptor.rid1_after = rid1_after;
    descriptor.msi_config_after = 0;
    descriptor.poll_reads = poll_reads;
    descriptor.l1_crc32 = bcm4388_handoff_crc32(pages->l1, (u32)BCM4388_HANDOFF_PAGE_SIZE);
    descriptor.client_l2_crc32 =
        bcm4388_handoff_crc32(pages->client_l2, (u32)BCM4388_HANDOFF_PAGE_SIZE);
    descriptor.msi_l2_crc32 = bcm4388_handoff_crc32(pages->msi_l2, (u32)BCM4388_HANDOFF_PAGE_SIZE);
    memcpy(descriptor.faults_after, faults_after, sizeof(descriptor.faults_after));
    descriptor.checksum = bcm4388_handoff_crc32(&descriptor, (u32)sizeof(descriptor));

    memcpy(published, &descriptor, sizeof(descriptor));
    return bcm4388_handoff_descriptor_validate(published);
}

static int rollback_transaction(struct bcm4388_handoff_result *result,
                                const struct bcm4388_handoff_io *io,
                                const struct bcm4388_handoff_pages *pages,
                                struct pcie_t602x_bcm4388_rid_transaction *rid_transaction,
                                u32 poll_attempts)
{
    struct pcie_t602x_mmio_ops pcie_ops = {
        .read32 = io->read32,
        .write32 = io->write32,
    };
    u32 ignored_polls = 0;
    int clean = 1;

    result->phase = BCM4388_HANDOFF_PHASE_ROLLBACK;
    result->rollback_attempted = 1;
    if (pcie_t602x_bcm4388_disable_port0_msi(&pcie_ops, io->context) != PCIE_T602X_BCM4388_OK)
        clean = 0;
    if (pcie_t602x_bcm4388_rollback_port0_rids(&pcie_ops, io->context, rid_transaction) !=
        PCIE_T602X_BCM4388_OK)
        clean = 0;
    if (io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_STREAM_DISABLE,
                   BCM4388_HANDOFF_DART_STREAM_SID1) != BCM4388_HANDOFF_OK)
        clean = 0;
    if (io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1, 0) !=
        BCM4388_HANDOFF_OK)
        clean = 0;
    if (io_write32(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1, 0) !=
        BCM4388_HANDOFF_OK)
        clean = 0;
    if (io_barrier(io) != BCM4388_HANDOFF_OK)
        clean = 0;
    if (flush_sid1(io, poll_attempts, &ignored_polls) != BCM4388_HANDOFF_OK)
        clean = 0;
    if (read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1, 0) !=
        BCM4388_HANDOFF_OK)
        clean = 0;
    if (read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1, 0) !=
        BCM4388_HANDOFF_OK)
        clean = 0;

    zero_page(pages->descriptor_page);
    zero_page(pages->l1);
    zero_page(pages->client_l2);
    zero_page(pages->msi_l2);
    if (io_barrier(io) != BCM4388_HANDOFF_OK)
        clean = 0;

    result->rollback_succeeded = clean;
    result->rollback_status = clean ? BCM4388_HANDOFF_OK : BCM4388_HANDOFF_ERR_ROLLBACK;
    return result->rollback_status;
}

static int fail_after_write(struct bcm4388_handoff_result *result,
                            const struct bcm4388_handoff_io *io,
                            const struct bcm4388_handoff_pages *pages,
                            struct pcie_t602x_bcm4388_rid_transaction *rid_transaction,
                            u32 poll_attempts, int primary_status)
{
    result->primary_status = primary_status;
    result->installed = 0;
    if (rollback_transaction(result, io, pages, rid_transaction, poll_attempts) !=
        BCM4388_HANDOFF_OK)
        return BCM4388_HANDOFF_ERR_ROLLBACK;
    return primary_status;
}

int bcm4388_legacy_dormant_handoff_install(struct bcm4388_handoff_result *result,
                                           const struct bcm4388_handoff_io *io,
                                           const struct bcm4388_handoff_pages *pages,
                                           u64 generation, u32 poll_attempts)
{
    struct bcm4388_handoff_preflight preflight;
    struct pcie_t602x_mmio_ops pcie_ops;
    struct pcie_t602x_bcm4388_rid_transaction rid_transaction = {0};
    u32 wifi_command_after = 0;
    u32 bluetooth_command_after = 0;
    u32 rid0_after = 0;
    u32 rid1_after = 0;
    u32 faults_after[5];
    int status;

    if (!result)
        return BCM4388_HANDOFF_ERR_ARGUMENT;
    memset(result, 0, sizeof(*result));
    result->primary_status = BCM4388_HANDOFF_ERR_ARGUMENT;
    if (!io || !io->read32 || !io->write32 || !io->barrier || !pages || generation == 0 ||
        poll_attempts == 0 || poll_attempts > BCM4388_HANDOFF_MAX_POLL_ATTEMPTS)
        return BCM4388_HANDOFF_ERR_ARGUMENT;

    result->phase = BCM4388_HANDOFF_PHASE_PREFLIGHT;
    status = validate_pages(pages);
    if (status != BCM4388_HANDOFF_OK) {
        result->primary_status = status;
        return status;
    }
    status = read_preflight(io, pages, &preflight);
    if (status != BCM4388_HANDOFF_OK) {
        result->primary_status = status;
        return status;
    }

    result->phase = BCM4388_HANDOFF_PHASE_TABLES;
    status = build_tables(pages);
    if (status != BCM4388_HANDOFF_OK) {
        result->primary_status = status;
        zero_page(pages->l1);
        zero_page(pages->client_l2);
        zero_page(pages->msi_l2);
        return status;
    }
    status = io_barrier(io);
    if (status != BCM4388_HANDOFF_OK) {
        result->primary_status = status;
        zero_page(pages->l1);
        zero_page(pages->client_l2);
        zero_page(pages->msi_l2);
        return status;
    }

    result->phase = BCM4388_HANDOFF_PHASE_DART;
    result->writes_started = 1;
    status = install_dart(io, preflight.encoded_ttbr, poll_attempts, &result->poll_reads);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);

    pcie_ops.read32 = io->read32;
    pcie_ops.write32 = io->write32;
    result->phase = BCM4388_HANDOFF_PHASE_RIDS;
    status = pcie_t602x_bcm4388_disable_port0_msi(&pcie_ops, io->context);
    if (status != PCIE_T602X_BCM4388_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts,
                                BCM4388_HANDOFF_ERR_IO);
    status = pcie_t602x_bcm4388_route_port0_rids(&pcie_ops, io->context, &rid_transaction);
    if (status != PCIE_T602X_BCM4388_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts,
                                BCM4388_HANDOFF_ERR_READBACK);
    status = validate_unique_rids(io, &rid0_after, &rid1_after);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);
    status = verify_bme_clear(io, &wifi_command_after, &bluetooth_command_after);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);
    status = require_msi_disabled_twice(io);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);
    status = read_faults(io, faults_after);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);
    status = read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1,
                        BCM4388_HANDOFF_DART_TCR_TRANSLATE);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);
    status = read_exact(io, BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1,
                        preflight.encoded_ttbr);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);

    result->phase = BCM4388_HANDOFF_PHASE_DESCRIPTOR;
    status =
        publish_descriptor(pages, &preflight, generation, result->poll_reads, wifi_command_after,
                           bluetooth_command_after, rid0_after, rid1_after, faults_after);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);
    status = io_barrier(io);
    if (status != BCM4388_HANDOFF_OK)
        return fail_after_write(result, io, pages, &rid_transaction, poll_attempts, status);

    result->phase = BCM4388_HANDOFF_PHASE_COMPLETE;
    result->primary_status = BCM4388_HANDOFF_OK;
    result->rollback_status = BCM4388_HANDOFF_OK;
    result->installed = 1;
    return BCM4388_HANDOFF_OK;
}
