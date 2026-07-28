/* SPDX-License-Identifier: MIT */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pcie.h"

_Static_assert(PCIE_T602X_BCM4388_PORT0_BASE == UINT64_C(0x594008000), "port base");
_Static_assert(PCIE_T602X_PORT_MSI_CONFIG_OFFSET == UINT64_C(0x124), "MSICFG");
_Static_assert(PCIE_T602X_PORT_MSI_ADDRESS_LO == UINT64_C(0x16c), "MSI address low");
_Static_assert(PCIE_T602X_PORT_MSI_ADDRESS_HI == UINT64_C(0x170), "MSI address high");
_Static_assert(PCIE_T602X_PORT_RID2SID_OFFSET == UINT64_C(0x3000), "RID2SID");
_Static_assert(PCIE_T602X_PORT_MSIMAP_OFFSET == UINT64_C(0x3800), "MSIMAP");
_Static_assert(PCIE_T602X_PORT_MSI_VECTOR_COUNT == 32, "MSI vector count");
_Static_assert(PCIE_T602X_BCM4388_MSI_ADDRESS == UINT32_C(0xfffff000), "MSI address");
_Static_assert(PCIE_T602X_BCM4388_WIFI_RID2SID == UINT32_C(0x80010100), "Wi-Fi RID2SID");
_Static_assert(PCIE_T602X_BCM4388_BLUETOOTH_RID2SID == UINT32_C(0x80010101), "Bluetooth RID2SID");

#define MAX_WRITES 128

struct write_event {
    u64 address;
    u32 value;
    bool completed;
};

struct mock_mmio {
    u32 msi_config;
    u32 msi_address_lo;
    u32 msi_address_hi;
    u32 rid2sid[2];
    u32 msimap[PCIE_T602X_PORT_MSI_VECTOR_COUNT];

    struct write_event writes[MAX_WRITES];
    size_t write_count;
    unsigned int bad_accesses;

    u64 fail_write_address;
    unsigned int fail_write_count;
    u64 fail_read_address;
    unsigned int fail_read_count;
    u64 corrupt_read_address;
    unsigned int corrupt_read_skip;
    unsigned int corrupt_read_count;
    u32 corrupt_read_value;
};

static void check(bool condition, const char *expression, const char *file, int line)
{
    if (condition)
        return;

    fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
    exit(1);
}

#define CHECK(expression) check((expression), #expression, __FILE__, __LINE__)

static u32 *mock_register(struct mock_mmio *mock, u64 address)
{
    const u64 base = PCIE_T602X_BCM4388_PORT0_BASE;
    const u64 map_base = base + PCIE_T602X_PORT_MSIMAP_OFFSET;

    if (address == base + PCIE_T602X_PORT_MSI_CONFIG_OFFSET)
        return &mock->msi_config;
    if (address == base + PCIE_T602X_PORT_MSI_ADDRESS_LO)
        return &mock->msi_address_lo;
    if (address == base + PCIE_T602X_PORT_MSI_ADDRESS_HI)
        return &mock->msi_address_hi;
    if (address == base + PCIE_T602X_PORT_RID2SID_OFFSET)
        return &mock->rid2sid[0];
    if (address == base + PCIE_T602X_PORT_RID2SID_OFFSET + 4)
        return &mock->rid2sid[1];
    if (address >= map_base && address < map_base + 4 * PCIE_T602X_PORT_MSI_VECTOR_COUNT &&
        ((address - map_base) & 3) == 0)
        return &mock->msimap[(address - map_base) / 4];

    mock->bad_accesses++;
    return NULL;
}

static int mock_read32(void *context, u64 address, u32 *value)
{
    struct mock_mmio *mock = context;
    u32 *reg = mock_register(mock, address);

    if (!reg)
        return -1;

    if (address == mock->fail_read_address && mock->fail_read_count) {
        mock->fail_read_count--;
        return -1;
    }

    if (address == mock->corrupt_read_address) {
        if (mock->corrupt_read_skip) {
            mock->corrupt_read_skip--;
        } else if (mock->corrupt_read_count) {
            mock->corrupt_read_count--;
            *value = mock->corrupt_read_value;
            return 0;
        }
    }

    *value = *reg;
    return 0;
}

static int mock_write32(void *context, u64 address, u32 value)
{
    struct mock_mmio *mock = context;
    struct write_event *event;
    u32 *reg = mock_register(mock, address);

    if (!reg)
        return -1;
    CHECK(mock->write_count < MAX_WRITES);

    event = &mock->writes[mock->write_count++];
    event->address = address;
    event->value = value;
    event->completed = true;

    if (address == mock->fail_write_address && mock->fail_write_count) {
        mock->fail_write_count--;
        event->completed = false;
        return -1;
    }

    *reg = value;
    return 0;
}

static const struct pcie_t602x_mmio_ops mock_ops = {
    .read32 = mock_read32,
    .write32 = mock_write32,
};

static void check_write(const struct mock_mmio *mock, size_t index, u64 address, u32 value,
                        bool completed)
{
    CHECK(index < mock->write_count);
    CHECK(mock->writes[index].address == address);
    CHECK(mock->writes[index].value == value);
    CHECK(mock->writes[index].completed == completed);
}

static void test_success_write_order(void)
{
    struct mock_mmio mock = {0};
    const u64 base = PCIE_T602X_BCM4388_PORT0_BASE;
    size_t index = 0;

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &mock) == PCIE_T602X_BCM4388_OK);

    check_write(&mock, index++, base + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, 0, true);
    check_write(&mock, index++, base + PCIE_T602X_PORT_RID2SID_OFFSET,
                PCIE_T602X_BCM4388_WIFI_RID2SID, true);
    check_write(&mock, index++, base + PCIE_T602X_PORT_RID2SID_OFFSET + 4,
                PCIE_T602X_BCM4388_BLUETOOTH_RID2SID, true);
    check_write(&mock, index++, base + PCIE_T602X_PORT_MSI_ADDRESS_LO,
                PCIE_T602X_BCM4388_MSI_ADDRESS, true);
    check_write(&mock, index++, base + PCIE_T602X_PORT_MSI_ADDRESS_HI, 0, true);

    for (u32 vector = 0; vector < PCIE_T602X_PORT_MSI_VECTOR_COUNT; vector++)
        check_write(&mock, index++, base + PCIE_T602X_PORT_MSIMAP_OFFSET + 4 * vector,
                    PCIE_T602X_MSIMAP_VALID | vector, true);

    check_write(&mock, index++, base + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, 1, true);
    CHECK(index == mock.write_count);
    CHECK(mock.bad_accesses == 0);
    CHECK(mock.rid2sid[0] == PCIE_T602X_BCM4388_WIFI_RID2SID);
    CHECK(mock.rid2sid[1] == PCIE_T602X_BCM4388_BLUETOOTH_RID2SID);
    CHECK(mock.msi_address_lo == PCIE_T602X_BCM4388_MSI_ADDRESS);
    CHECK(mock.msi_address_hi == 0);
    CHECK(mock.msi_config == 1);

    for (u32 vector = 0; vector < PCIE_T602X_PORT_MSI_VECTOR_COUNT; vector++)
        CHECK(mock.msimap[vector] == (PCIE_T602X_MSIMAP_VALID | vector));
}

static void test_idempotent_preinstalled_rids(void)
{
    struct mock_mmio mock = {
        .rid2sid = {PCIE_T602X_BCM4388_WIFI_RID2SID, PCIE_T602X_BCM4388_BLUETOOTH_RID2SID},
    };

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &mock) == PCIE_T602X_BCM4388_OK);
    CHECK(mock.write_count == 36);
    check_write(&mock, 0, PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, 0,
                true);
}

static void test_occupied_rid_rejection_has_zero_writes(void)
{
    struct mock_mmio rid0_occupied = {.rid2sid = {UINT32_C(0x80020200), 0}};
    struct mock_mmio rid1_occupied = {.rid2sid = {0, UINT32_C(0x80020201)}};

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &rid0_occupied) ==
          PCIE_T602X_BCM4388_ERR_RID0_OCCUPIED);
    CHECK(rid0_occupied.write_count == 0);

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &rid1_occupied) ==
          PCIE_T602X_BCM4388_ERR_RID1_OCCUPIED);
    CHECK(rid1_occupied.write_count == 0);
}

static void test_active_msi_decoder_rejection_has_zero_writes(void)
{
    struct mock_mmio mock = {.msi_config = 1};

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &mock) ==
          PCIE_T602X_BCM4388_ERR_MSI_NOT_QUIESCED);
    CHECK(mock.write_count == 0);
    CHECK(mock.msi_config == 1);
    CHECK(mock.rid2sid[0] == 0);
    CHECK(mock.rid2sid[1] == 0);
}

static void test_slot1_failure_rolls_back_slot0(void)
{
    const u64 rid_base = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;
    struct mock_mmio mock = {
        .fail_write_address = rid_base + 4,
        .fail_write_count = 1,
    };

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &mock) == PCIE_T602X_BCM4388_ERR_RID1_WRITE);
    CHECK(mock.write_count == 6);
    check_write(&mock, 0, PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, 0,
                true);
    check_write(&mock, 1, rid_base, PCIE_T602X_BCM4388_WIFI_RID2SID, true);
    check_write(&mock, 2, rid_base + 4, PCIE_T602X_BCM4388_BLUETOOTH_RID2SID, false);
    check_write(&mock, 3, rid_base + 4, 0, true);
    check_write(&mock, 4, rid_base, 0, true);
    check_write(&mock, 5, PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, 0,
                true);
    CHECK(mock.rid2sid[0] == 0);
    CHECK(mock.rid2sid[1] == 0);
    CHECK(mock.msi_config == 0);
}

static void test_slot1_readback_mismatch_rolls_back_both_slots(void)
{
    const u64 rid_base = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;
    struct mock_mmio mock = {
        .corrupt_read_address = rid_base + 4,
        .corrupt_read_skip = 1,
        .corrupt_read_count = 1,
        .corrupt_read_value = UINT32_C(0xdeadbeef),
    };

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &mock) ==
          PCIE_T602X_BCM4388_ERR_RID1_READBACK_MISMATCH);
    CHECK(mock.write_count == 6);
    check_write(&mock, 0, PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, 0,
                true);
    check_write(&mock, 1, rid_base, PCIE_T602X_BCM4388_WIFI_RID2SID, true);
    check_write(&mock, 2, rid_base + 4, PCIE_T602X_BCM4388_BLUETOOTH_RID2SID, true);
    check_write(&mock, 3, rid_base + 4, 0, true);
    check_write(&mock, 4, rid_base, 0, true);
    check_write(&mock, 5, PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET, 0,
                true);
    CHECK(mock.rid2sid[0] == 0);
    CHECK(mock.rid2sid[1] == 0);
    CHECK(mock.msi_config == 0);
}

static void test_msi_read_failure_leaves_decoder_disabled(void)
{
    const u32 failed_vector = 7;
    const u64 failed_address =
        PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSIMAP_OFFSET + 4 * failed_vector;
    struct mock_mmio mock = {
        .rid2sid = {PCIE_T602X_BCM4388_WIFI_RID2SID, PCIE_T602X_BCM4388_BLUETOOTH_RID2SID},
        .fail_read_address = failed_address,
        .fail_read_count = 1,
    };

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &mock) ==
          PCIE_T602X_BCM4388_ERR_MSI_MAP_READ(failed_vector));
    CHECK(mock.msi_config == 0);
    CHECK(mock.writes[mock.write_count - 1].address ==
          PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET);
    CHECK(mock.writes[mock.write_count - 1].value == 0);

    for (size_t index = 0; index < mock.write_count; index++) {
        if (mock.writes[index].address ==
            PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET)
            CHECK(mock.writes[index].value == 0);
    }
}

static void test_msi_failure_rolls_back_new_rids(void)
{
    const u32 failed_vector = 7;
    const u64 failed_address =
        PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSIMAP_OFFSET + 4 * failed_vector;
    struct mock_mmio mock = {
        .fail_read_address = failed_address,
        .fail_read_count = 1,
    };

    CHECK(pcie_t602x_bcm4388_setup_port0(&mock_ops, &mock) ==
          PCIE_T602X_BCM4388_ERR_MSI_MAP_READ(failed_vector));
    CHECK(mock.msi_config == 0);
    CHECK(mock.rid2sid[0] == 0);
    CHECK(mock.rid2sid[1] == 0);
    CHECK(mock.writes[mock.write_count - 2].address ==
          PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET + 4);
    CHECK(mock.writes[mock.write_count - 2].value == 0);
    CHECK(mock.writes[mock.write_count - 1].address ==
          PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET);
    CHECK(mock.writes[mock.write_count - 1].value == 0);
}

static void test_invalid_ops(void)
{
    struct mock_mmio mock = {0};
    struct pcie_t602x_mmio_ops missing_read = {.write32 = mock_write32};
    struct pcie_t602x_mmio_ops missing_write = {.read32 = mock_read32};

    CHECK(pcie_t602x_bcm4388_setup_port0(NULL, &mock) == PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT);
    CHECK(pcie_t602x_bcm4388_setup_port0(&missing_read, &mock) ==
          PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT);
    CHECK(pcie_t602x_bcm4388_setup_port0(&missing_write, &mock) ==
          PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT);
    CHECK(mock.write_count == 0);
}

static void test_split_rid_route_never_touches_msi(void)
{
    struct mock_mmio mock = {0};
    struct pcie_t602x_bcm4388_rid_transaction transaction;
    const u64 rid_base = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;

    CHECK(pcie_t602x_bcm4388_route_port0_rids(&mock_ops, &mock, &transaction) ==
          PCIE_T602X_BCM4388_OK);
    CHECK(mock.write_count == 2);
    check_write(&mock, 0, rid_base, PCIE_T602X_BCM4388_WIFI_RID2SID, true);
    check_write(&mock, 1, rid_base + 4, PCIE_T602X_BCM4388_BLUETOOTH_RID2SID, true);
    CHECK(mock.msi_config == 0);

    CHECK(pcie_t602x_bcm4388_rollback_port0_rids(&mock_ops, &mock, &transaction) ==
          PCIE_T602X_BCM4388_OK);
    CHECK(mock.rid2sid[0] == 0);
    CHECK(mock.rid2sid[1] == 0);
    CHECK(mock.msi_config == 0);
}

static void test_split_msi_enable_never_touches_rids(void)
{
    struct mock_mmio mock = {0};

    CHECK(pcie_t602x_bcm4388_enable_port0_msi(&mock_ops, &mock) == PCIE_T602X_BCM4388_OK);
    CHECK(mock.rid2sid[0] == 0);
    CHECK(mock.rid2sid[1] == 0);
    CHECK(mock.msi_config == 1);
    for (size_t index = 0; index < mock.write_count; index++) {
        CHECK(mock.writes[index].address !=
              PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET);
        CHECK(mock.writes[index].address !=
              PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET + 4);
    }
}

int main(void)
{
    test_success_write_order();
    test_idempotent_preinstalled_rids();
    test_occupied_rid_rejection_has_zero_writes();
    test_active_msi_decoder_rejection_has_zero_writes();
    test_slot1_failure_rolls_back_slot0();
    test_slot1_readback_mismatch_rolls_back_both_slots();
    test_msi_read_failure_leaves_decoder_disabled();
    test_msi_failure_rolls_back_new_rids();
    test_invalid_ops();
    test_split_rid_route_never_touches_msi();
    test_split_msi_enable_never_touches_rids();

    puts("T602X BCM4388 PCIe host tests: PASS");
    return 0;
}
