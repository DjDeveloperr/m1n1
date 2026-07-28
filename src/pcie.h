/* SPDX-License-Identifier: MIT */

#ifndef PCIE_H
#define PCIE_H

#ifdef PCIE_T602X_WIRELESS_HOST_TEST
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
#include "types.h"
#endif

/*
 * T602x APCIE port-0 resources used by the BCM4388 Wi-Fi (RID 0x100) and
 * Bluetooth (RID 0x101) functions.  The setup entry point below is deliberately
 * callback-backed and has no runtime call site: callers must first initialize
 * APCIE port 0 and establish that the link is up.
 */
#define PCIE_T602X_BCM4388_PORT0_BASE        UINT64_C(0x594008000)
#define PCIE_T602X_PORT_MSI_CONFIG_OFFSET    UINT64_C(0x124)
#define PCIE_T602X_PORT_MSI_ADDRESS_LO       UINT64_C(0x16c)
#define PCIE_T602X_PORT_MSI_ADDRESS_HI       UINT64_C(0x170)
#define PCIE_T602X_PORT_RID2SID_OFFSET       UINT64_C(0x3000)
#define PCIE_T602X_PORT_MSIMAP_OFFSET        UINT64_C(0x3800)
#define PCIE_T602X_PORT_MSI_VECTOR_COUNT     32
#define PCIE_T602X_BCM4388_MSI_ADDRESS       UINT32_C(0xfffff000)
#define PCIE_T602X_BCM4388_WIFI_RID2SID      UINT32_C(0x80010100)
#define PCIE_T602X_BCM4388_BLUETOOTH_RID2SID UINT32_C(0x80010101)
#define PCIE_T602X_MSIMAP_VALID              UINT32_C(0x80000000)
#define PCIE_T602X_PORT_RID2SID_ENTRY_COUNT  32

struct pcie_t602x_mmio_ops {
    int (*read32)(void *context, u64 address, u32 *value);
    int (*write32)(void *context, u64 address, u32 value);
};

/*
 * Rollback token for the two fixed BCM4388 RID2SID slots.  The fields are
 * public so a dormant pre-Mu handoff transaction can retain ownership across
 * its DART programming and descriptor-publication phases.  Callers must treat
 * the contents as opaque.
 */
struct pcie_t602x_bcm4388_rid_transaction {
    u32 prior_rid0;
    u32 prior_rid1;
    u32 changed_mask;
    bool active;
};

/*
 * Fixed failures use the negative values below.  Per-vector failures encode
 * the zero-based vector in the low range:
 *
 *   PCIE_T602X_BCM4388_ERR_MSI_MAP_WRITE(vector)
 *   PCIE_T602X_BCM4388_ERR_MSI_MAP_READ(vector)
 *   PCIE_T602X_BCM4388_ERR_MSI_MAP_MISMATCH(vector)
 */
enum pcie_t602x_bcm4388_error {
    PCIE_T602X_BCM4388_OK = 0,
    PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT = -1,

    PCIE_T602X_BCM4388_ERR_RID0_READ = -10,
    PCIE_T602X_BCM4388_ERR_RID1_READ = -11,
    PCIE_T602X_BCM4388_ERR_RID0_OCCUPIED = -12,
    PCIE_T602X_BCM4388_ERR_RID1_OCCUPIED = -13,
    PCIE_T602X_BCM4388_ERR_RID0_WRITE = -14,
    PCIE_T602X_BCM4388_ERR_RID0_READBACK_READ = -15,
    PCIE_T602X_BCM4388_ERR_RID0_READBACK_MISMATCH = -16,
    PCIE_T602X_BCM4388_ERR_RID1_WRITE = -17,
    PCIE_T602X_BCM4388_ERR_RID1_READBACK_READ = -18,
    PCIE_T602X_BCM4388_ERR_RID1_READBACK_MISMATCH = -19,
    PCIE_T602X_BCM4388_ERR_RID0_ROLLBACK = -20,
    PCIE_T602X_BCM4388_ERR_RID1_ROLLBACK = -21,

    PCIE_T602X_BCM4388_ERR_MSI_DISABLE_WRITE = -30,
    PCIE_T602X_BCM4388_ERR_MSI_DISABLE_READ = -31,
    PCIE_T602X_BCM4388_ERR_MSI_DISABLE_MISMATCH = -32,
    PCIE_T602X_BCM4388_ERR_MSI_PREFLIGHT_READ = -33,
    PCIE_T602X_BCM4388_ERR_MSI_NOT_QUIESCED = -34,
    PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_LO_WRITE = -40,
    PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_HI_WRITE = -41,
    PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_LO_READ = -42,
    PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_HI_READ = -43,
    PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_LO_MISMATCH = -44,
    PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_HI_MISMATCH = -45,

    PCIE_T602X_BCM4388_ERR_MSI_MAP_WRITE_BASE = -100,
    PCIE_T602X_BCM4388_ERR_MSI_MAP_READ_BASE = -200,
    PCIE_T602X_BCM4388_ERR_MSI_MAP_MISMATCH_BASE = -300,

    PCIE_T602X_BCM4388_ERR_MSI_ENABLE_WRITE = -400,
    PCIE_T602X_BCM4388_ERR_MSI_ENABLE_READ = -401,
    PCIE_T602X_BCM4388_ERR_MSI_ENABLE_MISMATCH = -402,
    PCIE_T602X_BCM4388_ERR_MSI_DISABLE_RECOVERY = -500,
};

#define PCIE_T602X_BCM4388_ERR_MSI_MAP_WRITE(vector)                                               \
    (PCIE_T602X_BCM4388_ERR_MSI_MAP_WRITE_BASE - (int)(vector))
#define PCIE_T602X_BCM4388_ERR_MSI_MAP_READ(vector)                                                \
    (PCIE_T602X_BCM4388_ERR_MSI_MAP_READ_BASE - (int)(vector))
#define PCIE_T602X_BCM4388_ERR_MSI_MAP_MISMATCH(vector)                                            \
    (PCIE_T602X_BCM4388_ERR_MSI_MAP_MISMATCH_BASE - (int)(vector))

/*
 * Independently callable ownership operations.  None train the link, touch
 * endpoint config space, or enable bus mastering.
 *
 * route_port0_rids() only installs the two fixed RID -> SID1 entries and
 * returns a token that can unwind exactly the slots it acquired.
 * enable_port0_msi() only programs/enables the port decoder and requires it to
 * be disabled on entry.  The dormant default-deny handoff intentionally never
 * calls this function.
 */
int pcie_t602x_bcm4388_require_port0_msi_disabled(const struct pcie_t602x_mmio_ops *ops,
                                                  void *context);
int pcie_t602x_bcm4388_disable_port0_msi(const struct pcie_t602x_mmio_ops *ops, void *context);
int pcie_t602x_bcm4388_route_port0_rids(const struct pcie_t602x_mmio_ops *ops, void *context,
                                        struct pcie_t602x_bcm4388_rid_transaction *transaction);
int pcie_t602x_bcm4388_rollback_port0_rids(const struct pcie_t602x_mmio_ops *ops, void *context,
                                           struct pcie_t602x_bcm4388_rid_transaction *transaction);
int pcie_t602x_bcm4388_enable_port0_msi(const struct pcie_t602x_mmio_ops *ops, void *context);

/*
 * Install the two RID-to-SID entries and then program the 32-vector MSI
 * decoder.  This function does not train the link, touch ECAM or endpoint PCI
 * configuration, enable bus mastering, configure DART, or alter endpoint MSI
 * capabilities.  The caller must serialize access to the two RID2SID slots and
 * the MSI registers for the full duration of the call.  The port MSI decoder
 * must read as zero, and both endpoint functions must have MSI and bus mastering
 * disabled before entry; otherwise this one-way bring-up helper must not run.
 */
int pcie_t602x_bcm4388_setup_port0(const struct pcie_t602x_mmio_ops *ops, void *context);

int pcie_init(void);
int pcie_shutdown(void);

#endif
