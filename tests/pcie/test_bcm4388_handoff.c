/* SPDX-License-Identifier: MIT */

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bcm4388_handoff.h"

#define MAX_WRITES 512

struct write_event {
    u64 address;
    u32 value;
    int completed;
};

struct mock_hardware {
    u32 wifi_identity;
    u32 wifi_command;
    u32 wifi_command_second;
    u32 wifi_command_reads;
    u32 wifi_command_second_after;

    u32 bluetooth_identity;
    u32 bluetooth_command;

    u32 msi_config;
    u32 msi_reads;
    u32 msi_flip_read;
    u32 rid2sid[PCIE_T602X_PORT_RID2SID_ENTRY_COUNT];

    u32 params1;
    u32 params2;
    u32 params3;
    u32 params4;
    u32 tlb_command;
    u32 fault0;
    u32 fault1;
    u32 fault_address_lo;
    u32 fault_address_hi;
    u32 fault_status;
    u32 protect;
    u32 stream_enable;
    u32 stream_disable;
    u32 tcr_sid1;
    u32 ttbr_sid1;

    u32 first_flush_busy_reads;
    u32 later_flush_busy_reads;
    u32 flush_busy_remaining;
    u32 flush_writes;
    int permanent_flush_busy;

    u64 fail_read_address;
    u32 fail_read_skip;
    u32 fail_read_count;
    u64 fail_write_address;
    u32 fail_write_skip;
    u32 fail_write_count;
    u64 corrupt_read_address;
    u32 corrupt_read_skip;
    u32 corrupt_read_count;
    u32 corrupt_read_value;

    struct write_event writes[MAX_WRITES];
    size_t write_count;
    u32 barriers;
    u32 bad_accesses;
};

struct fixture {
    struct mock_hardware hardware;
    struct bcm4388_handoff_io io;
    struct bcm4388_handoff_pages pages;
    struct bcm4388_handoff_result result;
};

static u64 descriptor_page[BCM4388_HANDOFF_TABLE_ENTRIES]
    __attribute__((aligned(BCM4388_HANDOFF_PAGE_SIZE)));
static u64 l1[BCM4388_HANDOFF_TABLE_ENTRIES] __attribute__((aligned(BCM4388_HANDOFF_PAGE_SIZE)));
static u64 client_l2[BCM4388_HANDOFF_TABLE_ENTRIES]
    __attribute__((aligned(BCM4388_HANDOFF_PAGE_SIZE)));
static u64 msi_l2[BCM4388_HANDOFF_TABLE_ENTRIES]
    __attribute__((aligned(BCM4388_HANDOFF_PAGE_SIZE)));

static const u8 golden_descriptor[sizeof(struct bcm4388_handoff_descriptor_v1)] = {
    0x42, 0x43, 0x4d, 0x31, 0x01, 0x00, 0x10, 0x00, 0x80, 0x01, 0x00, 0x00, 0xd8, 0xb7, 0x49, 0x45,
    0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x3f, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x20, 0x60, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x94, 0x05, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x94, 0x05, 0x00, 0x00, 0x00, 0xe4, 0x14, 0x34, 0x44, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe4, 0x14, 0x34, 0x44, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x14, 0x72, 0x5f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xe4, 0x14, 0x72, 0x5f, 0x01, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x21, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0e, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x05, 0x00, 0x10, 0x00, 0x00, 0x01, 0x01, 0x80,
    0x01, 0x01, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xd4, 0x57, 0x64, 0x3c,
    0x86, 0xd2, 0x54, 0xab, 0x8a, 0x96, 0xb4, 0x9b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static void check(int condition, const char *expression, const char *file, int line)
{
    if (condition)
        return;
    fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
    abort();
}

#define CHECK(expression) check((expression), #expression, __FILE__, __LINE__)

static u32 *mock_register(struct mock_hardware *mock, u64 address)
{
    const u64 wifi = BCM4388_HANDOFF_ECAM_BASE + UINT64_C(0x100000);
    const u64 bluetooth = wifi + UINT64_C(0x1000);
    const u64 rid_base = BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;

    if (address == wifi)
        return &mock->wifi_identity;
    if (address == wifi + UINT64_C(4))
        return &mock->wifi_command;
    if (address == bluetooth)
        return &mock->bluetooth_identity;
    if (address == bluetooth + UINT64_C(4))
        return &mock->bluetooth_command;
    if (address == BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET)
        return &mock->msi_config;
    if (address >= rid_base &&
        address < rid_base + (u64)PCIE_T602X_PORT_RID2SID_ENTRY_COUNT * UINT64_C(4) &&
        ((address - rid_base) & UINT64_C(3)) == 0)
        return &mock->rid2sid[(address - rid_base) / UINT64_C(4)];
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS1)
        return &mock->params1;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS2)
        return &mock->params2;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS3)
        return &mock->params3;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PARAMS4)
        return &mock->params4;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TLB_COMMAND)
        return &mock->tlb_command;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_FAULT0)
        return &mock->fault0;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_FAULT1)
        return &mock->fault1;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_FAULT_ADDR_LO)
        return &mock->fault_address_lo;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_FAULT_ADDR_HI)
        return &mock->fault_address_hi;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_FAULT_STATUS)
        return &mock->fault_status;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_PROTECT)
        return &mock->protect;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_STREAM_ENABLE)
        return &mock->stream_enable;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_STREAM_DISABLE)
        return &mock->stream_disable;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1)
        return &mock->tcr_sid1;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TTBR_SID1)
        return &mock->ttbr_sid1;

    mock->bad_accesses++;
    return NULL;
}

static int mock_read32(void *context, u64 address, u32 *value)
{
    struct mock_hardware *mock = context;
    u32 *reg = mock_register(mock, address);

    if (!reg)
        return -1;
    if (address == mock->fail_read_address) {
        if (mock->fail_read_skip) {
            mock->fail_read_skip--;
        } else if (mock->fail_read_count) {
            mock->fail_read_count--;
            return -1;
        }
    }

    if (address == BCM4388_HANDOFF_ECAM_BASE + UINT64_C(0x100004)) {
        mock->wifi_command_reads++;
        if (mock->wifi_command_second_after != 0 &&
            mock->wifi_command_reads > mock->wifi_command_second_after)
            *value = mock->wifi_command_second;
        else
            *value = mock->wifi_command;
    } else if (address == BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET) {
        mock->msi_reads++;
        if (mock->msi_flip_read != 0 && mock->msi_reads == mock->msi_flip_read)
            *value = 1;
        else
            *value = mock->msi_config;
    } else if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TLB_COMMAND &&
               mock->flush_writes != 0) {
        if (mock->permanent_flush_busy) {
            *value = mock->tlb_command | BCM4388_HANDOFF_DART_TLB_BUSY;
        } else if (mock->flush_busy_remaining != 0) {
            mock->flush_busy_remaining--;
            *value = mock->tlb_command | BCM4388_HANDOFF_DART_TLB_BUSY;
        } else {
            *value = mock->tlb_command & ~BCM4388_HANDOFF_DART_TLB_BUSY;
        }
    } else {
        *value = *reg;
    }

    if (address == mock->corrupt_read_address) {
        if (mock->corrupt_read_skip) {
            mock->corrupt_read_skip--;
        } else if (mock->corrupt_read_count) {
            mock->corrupt_read_count--;
            *value = mock->corrupt_read_value;
        }
    }
    return 0;
}

static int mock_write32(void *context, u64 address, u32 value)
{
    struct mock_hardware *mock = context;
    struct write_event *event;
    u32 *reg = mock_register(mock, address);

    if (!reg)
        return -1;
    CHECK(mock->write_count < MAX_WRITES);
    event = &mock->writes[mock->write_count++];
    event->address = address;
    event->value = value;
    event->completed = 1;

    if (address == mock->fail_write_address) {
        if (mock->fail_write_skip) {
            mock->fail_write_skip--;
        } else if (mock->fail_write_count) {
            mock->fail_write_count--;
            event->completed = 0;
            return -1;
        }
    }

    *reg = value;
    if (address == BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TLB_COMMAND) {
        mock->flush_writes++;
        mock->flush_busy_remaining =
            mock->flush_writes == 1 ? mock->first_flush_busy_reads : mock->later_flush_busy_reads;
    }
    return 0;
}

static int mock_barrier(void *context)
{
    struct mock_hardware *mock = context;
    mock->barriers++;
    return 0;
}

static void init_fixture(struct fixture *fixture)
{
    memset(fixture, 0, sizeof(*fixture));
    memset(descriptor_page, 0, sizeof(descriptor_page));
    memset(l1, 0, sizeof(l1));
    memset(client_l2, 0, sizeof(client_l2));
    memset(msi_l2, 0, sizeof(msi_l2));

    fixture->hardware.wifi_identity =
        (BCM4388_HANDOFF_WIFI_DEVICE_ID << 16) | BCM4388_HANDOFF_WIFI_VENDOR;
    fixture->hardware.bluetooth_identity =
        (BCM4388_HANDOFF_BT_DEVICE_ID << 16) | BCM4388_HANDOFF_BT_VENDOR;
    fixture->hardware.params1 = BCM4388_HANDOFF_PAGE_SHIFT
                                << BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_SHIFT;
    fixture->hardware.params2 = UINT32_C(0x12345678);
    fixture->hardware.params3 = UINT32_C(42) << BCM4388_HANDOFF_DART_PARAMS3_PA_WIDTH_SHIFT;
    fixture->hardware.params4 = UINT32_C(256);
    fixture->hardware.first_flush_busy_reads = 1;

    fixture->io.context = &fixture->hardware;
    fixture->io.read32 = mock_read32;
    fixture->io.write32 = mock_write32;
    fixture->io.barrier = mock_barrier;
    fixture->pages.descriptor_page = descriptor_page;
    fixture->pages.l1 = l1;
    fixture->pages.client_l2 = client_l2;
    fixture->pages.msi_l2 = msi_l2;
    fixture->pages.descriptor_physical = UINT64_C(0x100000000);
    fixture->pages.l1_physical = UINT64_C(0x100004000);
    fixture->pages.client_l2_physical = UINT64_C(0x100008000);
    fixture->pages.msi_l2_physical = UINT64_C(0x10000c000);
}

static int install(struct fixture *fixture, u32 poll_attempts)
{
    return bcm4388_legacy_dormant_handoff_install(
        &fixture->result, &fixture->io, &fixture->pages, UINT64_C(0x1122334455667788),
        poll_attempts);
}

static void check_pages_zero(void)
{
    for (u32 index = 0; index < BCM4388_HANDOFF_TABLE_ENTRIES; index++) {
        CHECK(descriptor_page[index] == 0);
        CHECK(l1[index] == 0);
        CHECK(client_l2[index] == 0);
        CHECK(msi_l2[index] == 0);
    }
}

static void check_preflight_failure(struct fixture *fixture, int expected)
{
    CHECK(install(fixture, 8) == expected);
    CHECK(fixture->result.primary_status == expected);
    CHECK(fixture->hardware.write_count == 0);
    CHECK(fixture->result.rollback_attempted == 0);
    check_pages_zero();
}

static void test_success_and_default_deny(void)
{
    struct fixture fixture;
    const struct bcm4388_handoff_descriptor_v1 *descriptor;
    u64 expected_client;
    u64 expected_msi_l2;
    u64 expected_msi_leaf =
        (((BCM4388_HANDOFF_MSI_PAGE_PHYSICAL >> BCM4388_HANDOFF_PAGE_SHIFT) << 10) &
         UINT64_C(0x000000fffffc00)) |
        UINT64_C(0x000fff0000000001);

    init_fixture(&fixture);
    expected_client = (((fixture.pages.client_l2_physical >> BCM4388_HANDOFF_PAGE_SHIFT) << 10) &
                       UINT64_C(0x000000fffffc00)) |
                      UINT64_C(1);
    expected_msi_l2 = (((fixture.pages.msi_l2_physical >> BCM4388_HANDOFF_PAGE_SHIFT) << 10) &
                       UINT64_C(0x000000fffffc00)) |
                      UINT64_C(1);

    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_OK);
    CHECK(fixture.result.installed == 1);
    CHECK(fixture.result.phase == BCM4388_HANDOFF_PHASE_COMPLETE);
    CHECK(fixture.result.poll_reads == 2);
    CHECK(fixture.hardware.rid2sid[0] == PCIE_T602X_BCM4388_WIFI_RID2SID);
    CHECK(fixture.hardware.rid2sid[1] == PCIE_T602X_BCM4388_BLUETOOTH_RID2SID);
    CHECK(fixture.hardware.msi_config == 0);
    CHECK(fixture.hardware.bad_accesses == 0);
    CHECK(l1[BCM4388_HANDOFF_CLIENT_L1_INDEX] == expected_client);
    CHECK(l1[BCM4388_HANDOFF_MSI_L1_INDEX] == expected_msi_l2);
    CHECK(msi_l2[BCM4388_HANDOFF_MSI_L2_INDEX] == expected_msi_leaf);

    for (u32 index = 0; index < BCM4388_HANDOFF_TABLE_ENTRIES; index++) {
        CHECK(client_l2[index] == 0);
        if (index != BCM4388_HANDOFF_CLIENT_L1_INDEX && index != BCM4388_HANDOFF_MSI_L1_INDEX)
            CHECK(l1[index] == 0);
        if (index != BCM4388_HANDOFF_MSI_L2_INDEX)
            CHECK(msi_l2[index] == 0);
    }
    for (size_t index = 0; index < fixture.hardware.write_count; index++) {
        u64 address = fixture.hardware.writes[index].address;
        if (address == BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET)
            CHECK(fixture.hardware.writes[index].value == 0);
        CHECK(address != BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_ADDRESS_LO);
        CHECK(address != BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSI_ADDRESS_HI);
        CHECK(address < BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSIMAP_OFFSET ||
              address >= BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_MSIMAP_OFFSET +
                             UINT64_C(4) * PCIE_T602X_PORT_MSI_VECTOR_COUNT);
    }

    descriptor = (const struct bcm4388_handoff_descriptor_v1 *)descriptor_page;
    CHECK(memcmp(descriptor, golden_descriptor, sizeof(golden_descriptor)) == 0);
    CHECK(bcm4388_handoff_descriptor_validate(descriptor) == BCM4388_HANDOFF_OK);
    CHECK(descriptor->l1_crc32 == bcm4388_handoff_crc32(l1, (u32)sizeof(l1)));
    CHECK(descriptor->client_l2_crc32 == bcm4388_handoff_crc32(client_l2, (u32)sizeof(client_l2)));
    CHECK(descriptor->msi_l2_crc32 == bcm4388_handoff_crc32(msi_l2, (u32)sizeof(msi_l2)));
}

static void test_identity_and_bme_rejections(void)
{
    struct fixture fixture;

    init_fixture(&fixture);
    fixture.hardware.wifi_identity = UINT32_C(0xffffffff);
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_IDENTITY);

    init_fixture(&fixture);
    fixture.hardware.bluetooth_command = BCM4388_HANDOFF_PCI_COMMAND_BME;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_BME_ACTIVE);

    init_fixture(&fixture);
    fixture.hardware.wifi_command_second_after = 1;
    fixture.hardware.wifi_command_second = UINT32_C(0x00000002);
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_BME_UNSTABLE);
}

static void test_msi_duplicate_busy_protect_and_fault_rejections(void)
{
    struct fixture fixture;

    init_fixture(&fixture);
    fixture.hardware.msi_config = 1;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_MSI_ENABLED);

    init_fixture(&fixture);
    fixture.hardware.rid2sid[7] = PCIE_T602X_BCM4388_WIFI_RID2SID;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_RID_DUPLICATE);

    init_fixture(&fixture);
    fixture.hardware.tlb_command = BCM4388_HANDOFF_DART_TLB_BUSY;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_DART_BUSY);

    init_fixture(&fixture);
    fixture.hardware.protect = BCM4388_HANDOFF_DART_PROTECT_TCR_TTBR;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_DART_PROTECTED);

    init_fixture(&fixture);
    fixture.hardware.fault_status = UINT32_C(0x1);
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_DART_FAULT);

    init_fixture(&fixture);
    fixture.hardware.params1 = UINT32_C(12) << BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_SHIFT;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_DART_PARAMS);
}

static void test_alignment_range_and_ownership_rejections(void)
{
    struct fixture fixture;

    init_fixture(&fixture);
    fixture.pages.l1_physical++;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_ALIGNMENT);

    init_fixture(&fixture);
    fixture.pages.msi_l2_physical = fixture.pages.client_l2_physical;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_RANGE);

    init_fixture(&fixture);
    l1[0] = 1;
    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_ERR_TABLE_NOT_EMPTY);
    CHECK(fixture.hardware.write_count == 0);
    CHECK(fixture.result.rollback_attempted == 0);
    CHECK(l1[0] == 1);

    init_fixture(&fixture);
    fixture.pages.l1_physical = UINT64_C(1) << 43;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_RANGE);

    init_fixture(&fixture);
    fixture.hardware.tcr_sid1 = 1;
    check_preflight_failure(&fixture, BCM4388_HANDOFF_ERR_DART_OWNED);
}

static void test_timeout_rolls_back_cleanly(void)
{
    struct fixture fixture;

    init_fixture(&fixture);
    fixture.hardware.first_flush_busy_reads = 3;
    fixture.hardware.later_flush_busy_reads = 0;
    CHECK(install(&fixture, 2) == BCM4388_HANDOFF_ERR_TLB_TIMEOUT);
    CHECK(fixture.result.primary_status == BCM4388_HANDOFF_ERR_TLB_TIMEOUT);
    CHECK(fixture.result.rollback_attempted == 1);
    CHECK(fixture.result.rollback_succeeded == 1);
    CHECK(fixture.result.rollback_status == BCM4388_HANDOFF_OK);
    CHECK(fixture.hardware.tcr_sid1 == 0);
    CHECK(fixture.hardware.ttbr_sid1 == 0);
    CHECK(fixture.hardware.msi_config == 0);
    CHECK(fixture.hardware.rid2sid[0] == 0);
    CHECK(fixture.hardware.rid2sid[1] == 0);
    check_pages_zero();
}

static void test_readback_failure_rolls_back(void)
{
    struct fixture fixture;

    init_fixture(&fixture);
    fixture.hardware.corrupt_read_address =
        BCM4388_HANDOFF_DART_BASE + BCM4388_HANDOFF_DART_TCR_SID1;
    fixture.hardware.corrupt_read_skip = 2;
    fixture.hardware.corrupt_read_count = 1;
    fixture.hardware.corrupt_read_value = 0;
    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_ERR_READBACK);
    CHECK(fixture.result.rollback_succeeded == 1);
    CHECK(fixture.hardware.tcr_sid1 == 0);
    CHECK(fixture.hardware.ttbr_sid1 == 0);
    check_pages_zero();
}

static void test_postinstall_msi_change_rolls_back(void)
{
    struct fixture fixture;

    init_fixture(&fixture);
    fixture.hardware.msi_flip_read = 4;
    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_ERR_MSI_ENABLED);
    CHECK(fixture.result.rollback_succeeded == 1);
    CHECK(fixture.hardware.msi_config == 0);
    CHECK(fixture.hardware.rid2sid[0] == 0);
    CHECK(fixture.hardware.rid2sid[1] == 0);
    check_pages_zero();
}

static void test_postinstall_bme_and_duplicate_rid_roll_back(void)
{
    struct fixture fixture;
    const u64 duplicate_address =
        BCM4388_HANDOFF_PORT_BASE + PCIE_T602X_PORT_RID2SID_OFFSET + UINT64_C(7) * UINT64_C(4);

    init_fixture(&fixture);
    fixture.hardware.wifi_command_second_after = 2;
    fixture.hardware.wifi_command_second = BCM4388_HANDOFF_PCI_COMMAND_BME;
    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_ERR_BME_ACTIVE);
    CHECK(fixture.result.rollback_succeeded == 1);
    CHECK(fixture.hardware.rid2sid[0] == 0);
    CHECK(fixture.hardware.rid2sid[1] == 0);
    check_pages_zero();

    init_fixture(&fixture);
    fixture.hardware.corrupt_read_address = duplicate_address;
    fixture.hardware.corrupt_read_skip = 1;
    fixture.hardware.corrupt_read_count = 1;
    fixture.hardware.corrupt_read_value = PCIE_T602X_BCM4388_WIFI_RID2SID;
    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_ERR_RID_DUPLICATE);
    CHECK(fixture.result.rollback_succeeded == 1);
    CHECK(fixture.hardware.rid2sid[0] == 0);
    CHECK(fixture.hardware.rid2sid[1] == 0);
    check_pages_zero();
}

static void test_rollback_failure_is_reported(void)
{
    struct fixture fixture;

    init_fixture(&fixture);
    fixture.hardware.permanent_flush_busy = 1;
    CHECK(install(&fixture, 2) == BCM4388_HANDOFF_ERR_ROLLBACK);
    CHECK(fixture.result.primary_status == BCM4388_HANDOFF_ERR_TLB_TIMEOUT);
    CHECK(fixture.result.rollback_status == BCM4388_HANDOFF_ERR_ROLLBACK);
    CHECK(fixture.result.rollback_attempted == 1);
    CHECK(fixture.result.rollback_succeeded == 0);
    CHECK(fixture.hardware.tcr_sid1 == 0);
    CHECK(fixture.hardware.ttbr_sid1 == 0);
    check_pages_zero();
}

static void test_descriptor_checksum_rejection(void)
{
    struct fixture fixture;
    struct bcm4388_handoff_descriptor_v1 *descriptor;

    init_fixture(&fixture);
    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_OK);
    descriptor = (struct bcm4388_handoff_descriptor_v1 *)descriptor_page;
    descriptor->generation ^= UINT64_C(1);
    CHECK(bcm4388_handoff_descriptor_validate(descriptor) == BCM4388_HANDOFF_ERR_DESCRIPTOR);
}

static void dump_golden_descriptor(void)
{
    struct fixture fixture;
    const u8 *bytes;

    init_fixture(&fixture);
    CHECK(install(&fixture, 8) == BCM4388_HANDOFF_OK);
    bytes = (const u8 *)descriptor_page;
    for (u32 index = 0; index < sizeof(struct bcm4388_handoff_descriptor_v1); index++) {
        if ((index % 12) == 0)
            fputs("    ", stdout);
        printf("0x%02x%s", bytes[index],
               index + 1 == sizeof(struct bcm4388_handoff_descriptor_v1) ? "" : ", ");
        if ((index % 12) == 11 || index + 1 == sizeof(struct bcm4388_handoff_descriptor_v1))
            fputc('\n', stdout);
    }
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--dump-golden") == 0) {
        dump_golden_descriptor();
        return 0;
    }

    test_success_and_default_deny();
    test_identity_and_bme_rejections();
    test_msi_duplicate_busy_protect_and_fault_rejections();
    test_alignment_range_and_ownership_rejections();
    test_timeout_rolls_back_cleanly();
    test_readback_failure_rolls_back();
    test_postinstall_msi_change_rolls_back();
    test_postinstall_bme_and_duplicate_rid_roll_back();
    test_rollback_failure_is_reported();
    test_descriptor_checksum_rejection();

    puts("BCM4388 SID1 handoff host tests: PASS");
    return 0;
}
