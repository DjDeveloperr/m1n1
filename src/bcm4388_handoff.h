/* SPDX-License-Identifier: MIT */

#ifndef BCM4388_HANDOFF_H
#define BCM4388_HANDOFF_H

#include "pcie.h"

#define BCM4388_HANDOFF_SIGNATURE UINT32_C(0x314d4342) /* "BCM1" */
#define BCM4388_HANDOFF_VERSION   UINT16_C(1)

#define BCM4388_HANDOFF_CHIP_ID UINT32_C(0x6020)
#define BCM4388_HANDOFF_SID     UINT32_C(1)

#define BCM4388_HANDOFF_ECAM_BASE UINT64_C(0x580000000)
#define BCM4388_HANDOFF_DART_BASE UINT64_C(0x594000000)
#define BCM4388_HANDOFF_PORT_BASE PCIE_T602X_BCM4388_PORT0_BASE

#define BCM4388_HANDOFF_WIFI_BUS       UINT32_C(1)
#define BCM4388_HANDOFF_WIFI_DEVICE    UINT32_C(0)
#define BCM4388_HANDOFF_WIFI_FUNCTION  UINT32_C(0)
#define BCM4388_HANDOFF_WIFI_VENDOR    UINT32_C(0x14e4)
#define BCM4388_HANDOFF_WIFI_DEVICE_ID UINT32_C(0x4434)
#define BCM4388_HANDOFF_WIFI_RID       UINT32_C(0x100)

#define BCM4388_HANDOFF_BT_BUS       UINT32_C(1)
#define BCM4388_HANDOFF_BT_DEVICE    UINT32_C(0)
#define BCM4388_HANDOFF_BT_FUNCTION  UINT32_C(1)
#define BCM4388_HANDOFF_BT_VENDOR    UINT32_C(0x14e4)
#define BCM4388_HANDOFF_BT_DEVICE_ID UINT32_C(0x5f72)
#define BCM4388_HANDOFF_BT_RID       UINT32_C(0x101)

#define BCM4388_HANDOFF_PCI_COMMAND_BME UINT32_C(0x00000004)

#define BCM4388_HANDOFF_PAGE_SHIFT     UINT32_C(14)
#define BCM4388_HANDOFF_PAGE_SIZE      (UINT64_C(1) << BCM4388_HANDOFF_PAGE_SHIFT)
#define BCM4388_HANDOFF_TABLE_ENTRIES  UINT32_C(2048)
#define BCM4388_HANDOFF_L2_WINDOW_SIZE +(BCM4388_HANDOFF_PAGE_SIZE * BCM4388_HANDOFF_TABLE_ENTRIES)

#define BCM4388_HANDOFF_CLIENT_IOVA_BASE  UINT64_C(0x20000000)
#define BCM4388_HANDOFF_CLIENT_IOVA_LIMIT UINT64_C(0x22000000)
#define BCM4388_HANDOFF_WIFI_IOVA_BASE    UINT64_C(0x20000000)
#define BCM4388_HANDOFF_WIFI_IOVA_LIMIT   UINT64_C(0x21800000)
#define BCM4388_HANDOFF_BT_IOVA_BASE      UINT64_C(0x21800000)
#define BCM4388_HANDOFF_BT_IOVA_LIMIT     UINT64_C(0x22000000)

#define BCM4388_HANDOFF_MSI_DOORBELL_IOVA UINT64_C(0xfffff000)
#define BCM4388_HANDOFF_MSI_PAGE_IOVA     UINT64_C(0xffffc000)
#define BCM4388_HANDOFF_MSI_PAGE_PHYSICAL UINT64_C(0xffffc000)
#define BCM4388_HANDOFF_CLIENT_L1_INDEX   UINT32_C(16)
#define BCM4388_HANDOFF_MSI_L1_INDEX      UINT32_C(127)
#define BCM4388_HANDOFF_MSI_L2_INDEX      UINT32_C(2047)

#define BCM4388_HANDOFF_DART_PARAMS1        UINT64_C(0x0000)
#define BCM4388_HANDOFF_DART_PARAMS2        UINT64_C(0x0004)
#define BCM4388_HANDOFF_DART_PARAMS3        UINT64_C(0x0008)
#define BCM4388_HANDOFF_DART_PARAMS4        UINT64_C(0x000c)
#define BCM4388_HANDOFF_DART_TLB_COMMAND    UINT64_C(0x0080)
#define BCM4388_HANDOFF_DART_FAULT0         UINT64_C(0x0100)
#define BCM4388_HANDOFF_DART_FAULT1         UINT64_C(0x0104)
#define BCM4388_HANDOFF_DART_FAULT_ADDR_LO  UINT64_C(0x0170)
#define BCM4388_HANDOFF_DART_FAULT_ADDR_HI  UINT64_C(0x0174)
#define BCM4388_HANDOFF_DART_FAULT_STATUS   UINT64_C(0x01c0)
#define BCM4388_HANDOFF_DART_PROTECT        UINT64_C(0x0200)
#define BCM4388_HANDOFF_DART_STREAM_ENABLE  UINT64_C(0x0c00)
#define BCM4388_HANDOFF_DART_STREAM_DISABLE UINT64_C(0x0c20)
#define BCM4388_HANDOFF_DART_TCR_SID1       UINT64_C(0x1004)
#define BCM4388_HANDOFF_DART_TTBR_SID1      UINT64_C(0x1404)

#define BCM4388_HANDOFF_DART_TLB_BUSY                 UINT32_C(0x80000000)
#define BCM4388_HANDOFF_DART_TLB_FLUSH_SID1           UINT32_C(0x00000101)
#define BCM4388_HANDOFF_DART_STREAM_SID1              UINT32_C(0x00000002)
#define BCM4388_HANDOFF_DART_TCR_TRANSLATE            UINT32_C(0x00000001)
#define BCM4388_HANDOFF_DART_TTBR_VALID               UINT32_C(0x00000001)
#define BCM4388_HANDOFF_DART_TTBR_ADDRESS             UINT32_C(0x3ffffffc)
#define BCM4388_HANDOFF_DART_PROTECT_TCR_TTBR         UINT32_C(0x00000001)
#define BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_MASK  UINT32_C(0x0f000000)
#define BCM4388_HANDOFF_DART_PARAMS1_PAGE_SHIFT_SHIFT UINT32_C(24)
#define BCM4388_HANDOFF_DART_PARAMS3_PA_WIDTH_MASK    UINT32_C(0x3f000000)
#define BCM4388_HANDOFF_DART_PARAMS3_PA_WIDTH_SHIFT   UINT32_C(24)
#define BCM4388_HANDOFF_DART_PARAMS4_SID_COUNT_MASK   UINT32_C(0x000001ff)
#define BCM4388_HANDOFF_MAX_POLL_ATTEMPTS             UINT32_C(4096)

#define BCM4388_HANDOFF_FLAG_COMPLETE          UINT32_C(0x00000001)
#define BCM4388_HANDOFF_FLAG_DEFAULT_DENY      UINT32_C(0x00000002)
#define BCM4388_HANDOFF_FLAG_MSI_PAGE_ONLY     UINT32_C(0x00000004)
#define BCM4388_HANDOFF_FLAG_BME_CLEAR         UINT32_C(0x00000008)
#define BCM4388_HANDOFF_FLAG_PORT_MSI_DISABLED UINT32_C(0x00000010)
#define BCM4388_HANDOFF_FLAG_RIDS_UNIQUE       UINT32_C(0x00000020)
#define BCM4388_HANDOFF_REQUIRED_FLAGS                                                             \
    +(BCM4388_HANDOFF_FLAG_COMPLETE | BCM4388_HANDOFF_FLAG_DEFAULT_DENY |                          \
      +BCM4388_HANDOFF_FLAG_MSI_PAGE_ONLY | BCM4388_HANDOFF_FLAG_BME_CLEAR |                       \
      +BCM4388_HANDOFF_FLAG_PORT_MSI_DISABLED | BCM4388_HANDOFF_FLAG_RIDS_UNIQUE)

struct bcm4388_handoff_endpoint_v1 {
    u32 identity;
    u32 command_before;
    u32 command_after;
    u16 segment;
    u16 rid;
    u16 vendor_id;
    u16 device_id;
    u8 bus;
    u8 device;
    u8 function;
    u8 reserved0;
    u32 reserved1;
    u32 reserved2;
};

/*
 * Little-endian, fixed-size handoff wire format.  The checksum is IEEE CRC-32
 * over total_size bytes with checksum set to zero.  Every physical range is
 * owned by the producer and must be reserved by Mu before Windows adoption.
 */
struct bcm4388_handoff_descriptor_v1 {
    u32 signature;
    u16 version;
    u16 header_size;
    u32 total_size;
    u32 checksum;
    u64 generation;
    u32 flags;
    u32 sid;

    u32 chip_id;
    u32 page_shift;
    u32 sid_count;
    u32 pa_width;
    u64 ecam_base;
    u64 port_base;
    u64 dart_base;

    struct bcm4388_handoff_endpoint_v1 wifi;
    struct bcm4388_handoff_endpoint_v1 bluetooth;

    u64 descriptor_physical;
    u64 descriptor_length;
    u64 l1_physical;
    u64 l1_length;
    u64 client_l2_physical;
    u64 client_l2_length;
    u64 msi_l2_physical;
    u64 msi_l2_length;

    u64 client_iova_base;
    u64 client_iova_limit;
    u64 wifi_iova_base;
    u64 wifi_iova_limit;
    u64 bluetooth_iova_base;
    u64 bluetooth_iova_limit;
    u64 msi_doorbell_iova;
    u64 msi_page_iova;
    u64 msi_page_physical;

    u32 params1;
    u32 params2;
    u32 params3;
    u32 params4;
    u32 protect_before;
    u32 tcr_before;
    u32 ttbr_before;
    u32 tlb_before;
    u32 faults_before[5];

    u32 tcr_after;
    u32 ttbr_after;
    u32 rid0_after;
    u32 rid1_after;
    u32 msi_config_after;
    u32 poll_reads;
    u32 l1_crc32;
    u32 client_l2_crc32;
    u32 msi_l2_crc32;
    u32 faults_after[5];
    u32 reserved;
};

struct bcm4388_handoff_pages {
    u64 *l1;
    u64 *client_l2;
    u64 *msi_l2;
    void *descriptor_page;
    u64 l1_physical;
    u64 client_l2_physical;
    u64 msi_l2_physical;
    u64 descriptor_physical;
};

struct bcm4388_handoff_io {
    void *context;
    int (*read32)(void *context, u64 address, u32 *value);
    int (*write32)(void *context, u64 address, u32 value);
    int (*barrier)(void *context);
};

enum bcm4388_handoff_status {
    BCM4388_HANDOFF_OK = 0,
    BCM4388_HANDOFF_ERR_ARGUMENT = -1000,
    BCM4388_HANDOFF_ERR_ALIGNMENT = -1001,
    BCM4388_HANDOFF_ERR_RANGE = -1002,
    BCM4388_HANDOFF_ERR_TABLE_NOT_EMPTY = -1003,
    BCM4388_HANDOFF_ERR_IDENTITY = -1004,
    BCM4388_HANDOFF_ERR_BME_ACTIVE = -1005,
    BCM4388_HANDOFF_ERR_BME_UNSTABLE = -1006,
    BCM4388_HANDOFF_ERR_MSI_ENABLED = -1007,
    BCM4388_HANDOFF_ERR_DART_PARAMS = -1008,
    BCM4388_HANDOFF_ERR_DART_PROTECTED = -1009,
    BCM4388_HANDOFF_ERR_DART_OWNED = -1010,
    BCM4388_HANDOFF_ERR_DART_BUSY = -1011,
    BCM4388_HANDOFF_ERR_DART_FAULT = -1012,
    BCM4388_HANDOFF_ERR_RID_OCCUPIED = -1013,
    BCM4388_HANDOFF_ERR_RID_DUPLICATE = -1014,
    BCM4388_HANDOFF_ERR_IO = -1015,
    BCM4388_HANDOFF_ERR_READBACK = -1016,
    BCM4388_HANDOFF_ERR_TLB_TIMEOUT = -1017,
    BCM4388_HANDOFF_ERR_DESCRIPTOR = -1018,
    BCM4388_HANDOFF_ERR_ROLLBACK = -1019,
};

enum bcm4388_handoff_phase {
    BCM4388_HANDOFF_PHASE_NONE = 0,
    BCM4388_HANDOFF_PHASE_PREFLIGHT,
    BCM4388_HANDOFF_PHASE_TABLES,
    BCM4388_HANDOFF_PHASE_DART,
    BCM4388_HANDOFF_PHASE_RIDS,
    BCM4388_HANDOFF_PHASE_DESCRIPTOR,
    BCM4388_HANDOFF_PHASE_COMPLETE,
    BCM4388_HANDOFF_PHASE_ROLLBACK,
};

struct bcm4388_handoff_result {
    int primary_status;
    int rollback_status;
    u32 phase;
    u32 poll_reads;
    int writes_started;
    int rollback_attempted;
    int rollback_succeeded;
    int installed;
};

u32 bcm4388_handoff_crc32(const void *data, u32 length);
int bcm4388_handoff_descriptor_validate(const struct bcm4388_handoff_descriptor_v1 *descriptor);

/*
 * Dormant transaction core.  There is intentionally no runtime call site.
 * The caller must exclusively own all four zeroed 16-KiB pages and serialize
 * every endpoint/port/DART access for the duration of the call.
 */
int bcm4388_handoff_install(struct bcm4388_handoff_result *result,
                            const struct bcm4388_handoff_io *io,
                            const struct bcm4388_handoff_pages *pages, u64 generation,
                            u32 poll_attempts);

#endif
