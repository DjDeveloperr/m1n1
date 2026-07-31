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
#define PCIE_T602X_PORT_MSI_ENABLE           UINT32_C(0x1)
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

/*
 * The 0xfffff000 doorbell is a property of the APCIE controller, not of the
 * BCM4388: /arm-io/apcie carries `msi-address = 0xfffff000`, `#msi-vectors =
 * 32` and `msi-vector-offset = 1672` for every port.  The BCM4388-flavoured
 * spelling above predates the generic per-port path, so alias it rather than
 * duplicating the literal.
 */
#define PCIE_T602X_MSI_DOORBELL_ADDRESS      PCIE_T602X_BCM4388_MSI_ADDRESS

/*
 * Port interrupt status, Linux pcie-apple.c PORT_INTSTAT (write-1-to-clear).
 * Two of its bits are a free MSI-path diagnostic that costs nothing to arm:
 * MSI_ERR is raised when the port cannot deliver an inbound MSI write, and
 * MSI_BAD_DATA when the write's data payload falls outside the vector map
 * programmed at PORT_MSIMAP.  Absolute addresses on J414s, derived from the
 * per-port block stride (reg[8 + 5N]): port 0 = 0x594008100, port 1 =
 * 0x595008100.  Both are readable from the m1n1 hypervisor while the guest
 * runs, so a "Windows never gets an interrupt" report can be split into
 * "the device never sent one" and "the port refused it" without a rebuild.
 */
#define PCIE_T602X_PORT_INTSTAT              UINT64_C(0x100)
#define PCIE_T602X_PORT_INT_MSI_ERR          (UINT32_C(1) << 18)
#define PCIE_T602X_PORT_INT_MSI_BAD_DATA     (UINT32_C(1) << 19)

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
 * the MSI registers for the full duration of the call.  The port MSI decoder's
 * enable bit must be clear, and both endpoint functions must have MSI and bus
 * mastering disabled before entry; otherwise this one-way bring-up helper must
 * not run. Other MSICFG fields established by pcie_init() are preserved.
 */
int pcie_t602x_bcm4388_setup_port0(const struct pcie_t602x_mmio_ops *ops, void *context);

/*
 * Apple/Asahi PCIe port bring-up: PERST# ordering, refclk handshake and link
 * training.
 *
 * The reference implementation is Linux drivers/pci/controller/pcie-apple.c
 * (AsahiLinux/linux, asahi branch), functions apple_pcie_setup_link(),
 * apple_pcie_setup_refclk() and apple_pcie_setup_port().  The whole sequence is
 * expressed over the callback table below so the ordering, the delays and the
 * graceful no-GPIO path are covered by tests/pcie/test_port_bringup.c without
 * hardware.
 */

/* Port registers used by the bring-up (offsets from the port config block). */
#define PCIE_PORT_LTSSMCTL       UINT64_C(0x080)
#define PCIE_PORT_LTSSMCTL_START UINT32_C(0x1)
#define PCIE_PORT_LINKSTS        UINT64_C(0x208)
#define PCIE_PORT_LINKSTS_UP     UINT32_C(0x1)
#define PCIE_PORT_APPCLK         UINT64_C(0x800)
#define PCIE_PORT_APPCLK_EN      UINT32_C(0x1)
#define PCIE_PORT_PERST_OFF      UINT32_C(0x1)

/*
 * Per-port PHY_LANE_CFG, at offset 0 of the port PHY block.  Names and bit
 * positions from pcie-apple.c.
 */
#define PCIE_PHY_LANE_CFG            UINT64_C(0x000)
#define PCIE_PHY_LANE_CFG_REFCLK0REQ UINT32_C(0x1)
#define PCIE_PHY_LANE_CFG_REFCLK1REQ UINT32_C(0x2)
#define PCIE_PHY_LANE_CFG_REFCLK0ACK UINT32_C(0x4)
#define PCIE_PHY_LANE_CFG_REFCLK1ACK UINT32_C(0x8)
/* REFCLKEN is BIT(9) | BIT(10); m1n1 has always set the two bits separately. */
#define PCIE_PHY_LANE_CFG_REFCLKEN0 UINT32_C(0x200)
#define PCIE_PHY_LANE_CFG_REFCLKEN1 UINT32_C(0x400)
/* Cleared by m1n1 before enabling the refclk; undocumented upstream. */
#define PCIE_PHY_LANE_CFG_UNK14 UINT32_C(0x4000)
/* BIT(30) | BIT(31): refclk clock-gating enable, set once the port is ready. */
#define PCIE_PHY_LANE_CFG_REFCLKCGEN UINT32_C(0xc0000000)

/*
 * PERST# timing taken from the ADT rather than hardcoded.
 *
 * Units, and how they were determined.  Both J414s values are 100, so the ADT
 * alone cannot disambiguate; these conclusions come from the consumers.
 *
 *   t-refclk-to-perst  MICROseconds.  This is Tperst-clk, the minimum time
 *                      from a stable reference clock to PERST# deassertion,
 *                      specified as 100 us by PCIe CEM r5.0 section 2.9.2.
 *                      Three independent lines of evidence:
 *                      (1) pcie-apple.c implements exactly this step with
 *                          usleep_range(100, 200) and cites that clause;
 *                          U-Boot's pcie_apple.c uses udelay(100).
 *                      (2) In Apple's AppleEmbeddedPCIE.kext, the value is
 *                          consumed by APCIECoreRCGen4Port::
 *                          initializeRefclkBuffer() as IODelay(
 *                          getTRefclkToPerst()); IODelay() takes microseconds.
 *                      (3) iMac21,1 pci-bridge1 (an ASMedia ASM3142 xHCI with
 *                          a power-enable GPIO) declares 200000, which is a
 *                          sensible 200 ms but an absurd 200 s.  That machine
 *                          is why the cap below is well above 100 ms: a
 *                          smaller cap would silently truncate a real,
 *                          published value.
 *
 *   perst-to-config    MILLIseconds.  This is the wait between PERST#
 *                      deassertion and the first configuration request,
 *                      specified as 100 ms by PCIe Base r5.0 section 6.6.1.
 *                      pcie-apple.c implements exactly this step with
 *                      msleep(100) and cites that clause; U-Boot uses
 *                      udelay(100 * 1000).  In AppleEmbeddedPCIE.kext,
 *                      AppleEmbeddedPCIEPort::handleLinkUp() converts the time
 *                      since PERST# deassert to milliseconds (ns / 1000000),
 *                      compares it against getPerstToConfig() and passes the
 *                      remainder to IOSleep(), which takes milliseconds.
 *                      Every published Apple Silicon ADT declares 100.
 *
 * Because a misread unit is only dangerous in the "too fast" direction, the
 * converted values are floored at the spec minimum -- so the sequence can only
 * ever be at least as conservative as Asahi's hardcoded delays.  They are also
 * capped, so a corrupt ADT cannot stall boot indefinitely.
 */
#define PCIE_PERST_REFCLK_TO_PERST_MIN_US UINT32_C(100)
/* Above the largest published value (iMac21,1 declares 200000 us). */
#define PCIE_PERST_REFCLK_TO_PERST_MAX_US UINT32_C(500000)
#define PCIE_PERST_TO_CONFIG_MIN_US       UINT32_C(100000)
#define PCIE_PERST_TO_CONFIG_MAX_US       UINT32_C(1000000)

/* Asahi's module default for link training (link_up_timeout = 500 ms). */
#define PCIE_PERST_LINK_UP_TIMEOUT_US UINT32_C(500000)

/*
 * Tpvperl: power valid -> PERST# inactive.  PCIe CEM requires 100 ms, and
 * pcie-apple.c spends exactly that (msleep(100)) after raising a port's pwren
 * and before releasing PERST#, only for ports that actually have a rail.
 */
#define PCIE_PWREN_TO_PERST_US UINT32_C(100000)

struct pcie_perst_delays {
    u32 refclk_to_perst_us;
    u32 perst_to_config_us;
};

/*
 * Convert the ADT's `t-refclk-to-perst` and `perst-to-config` into
 * microseconds.  Absent properties fall back to the spec minimum, which is
 * what pcie-apple.c hardcodes.
 */
void pcie_perst_delays_from_adt(struct pcie_perst_delays *out, bool have_refclk_to_perst,
                                u32 refclk_to_perst, bool have_perst_to_config,
                                u32 perst_to_config);

struct pcie_port_bringup_ops {
    int (*read32)(void *context, u64 address, u32 *value);
    int (*write32)(void *context, u64 address, u32 value);
    int (*set32)(void *context, u64 address, u32 set);
    int (*clear32)(void *context, u64 address, u32 clear);
    int (*poll32)(void *context, u64 address, u32 mask, u32 target, u32 timeout_us);
    void (*delay_us)(void *context, u32 us);
    /*
     * Drive PERST#.  `asserted` is the logical PERST# level, so asserted ==
     * true means the endpoint is held in reset.  The pad is active low
     * (reset-gpios = <&pinctrl_ap N GPIO_ACTIVE_LOW> in Asahi's device trees),
     * so implementations drive the pad to 0 when asserted is true.  NULL when
     * the port has no PERST# GPIO, in which case PERST# is left alone.
     */
    int (*perst_set)(void *context, bool asserted);
};

struct pcie_port_bringup {
    u64 port_base;
    /* 0 when the port has no per-port PHY block (pre-T602x). */
    u64 port_phy_base;
    /* Offset of the internal PERST register: 0x82c on T602x, 0x814 before. */
    u64 perst_reg;
    u32 phy_ack_timeout_us;
    u32 link_up_timeout_us;
    struct pcie_perst_delays delays;
    /* False when function-perst did not resolve; PERST# is then not driven. */
    bool have_perst_gpio;
    /*
     * True when LINKSTS already reported UP before bring-up started.  The
     * link is then never reset out from under whoever owns it: PERST# is not
     * toggled and LTSSM is not restarted.
     */
    bool link_was_up;
};

enum pcie_port_bringup_error {
    PCIE_PORT_BRINGUP_OK = 0,
    PCIE_PORT_BRINGUP_ERR_INVALID_ARGUMENT = -1,
    PCIE_PORT_BRINGUP_ERR_APPCLK = -2,
    PCIE_PORT_BRINGUP_ERR_PERST_ASSERT = -3,
    PCIE_PORT_BRINGUP_ERR_PHY_REQ = -4,
    PCIE_PORT_BRINGUP_ERR_PHY_CLK0_ACK = -5,
    PCIE_PORT_BRINGUP_ERR_PHY_CLK1_ACK = -6,
    PCIE_PORT_BRINGUP_ERR_REFCLK_EN = -7,
    PCIE_PORT_BRINGUP_ERR_PERST_RELEASE_REG = -8,
    PCIE_PORT_BRINGUP_ERR_PERST_RELEASE_GPIO = -9,
    PCIE_PORT_BRINGUP_ERR_REFCLK_CGEN = -10,
    PCIE_PORT_BRINGUP_ERR_LTSSM_START = -11,
    PCIE_PORT_BRINGUP_ERR_LINK_DOWN = -12,
};

/*
 * Phase 1, mirroring apple_pcie_setup_link() + apple_pcie_setup_refclk():
 * enable APPCLK, assert PERST# before the clocks come up, run the refclk
 * request/ack handshake and enable the refclk, honour Tperst-clk, then release
 * the internal PERST register followed by the PERST# pad.
 *
 * The caller must then wait delays.perst_to_config_us after the last PERST#
 * transition before any configuration-space access.
 */
int pcie_port_release_perst(const struct pcie_port_bringup_ops *ops, void *context,
                            const struct pcie_port_bringup *cfg);

/*
 * Phase 2, mirroring the tail of apple_pcie_setup_port(): enable refclk clock
 * gating, start LTSSM and poll LINKSTS for UP with a bounded timeout.  Returns
 * PCIE_PORT_BRINGUP_ERR_LINK_DOWN if the link never trains; callers that do not
 * require a link may treat that as non-fatal.
 */
int pcie_port_start_link(const struct pcie_port_bringup_ops *ops, void *context,
                         const struct pcie_port_bringup *cfg);

enum pcie_port_msi_error {
    PCIE_PORT_MSI_OK = 0,
    PCIE_PORT_MSI_ERR_INVALID_ARGUMENT = -1,
    PCIE_PORT_MSI_ERR_ADDRESS_WRITE = -2,
    PCIE_PORT_MSI_ERR_MAP_WRITE = -3,
    PCIE_PORT_MSI_ERR_ENABLE_WRITE = -4,
    PCIE_PORT_MSI_ERR_ADDRESS_READBACK = -5,
    PCIE_PORT_MSI_ERR_MAP_READBACK = -6,
    PCIE_PORT_MSI_ERR_ENABLE_READBACK = -7,
};

/*
 * Phase 3, mirroring the t602x branch of Linux's apple_pcie_port_setup_irq():
 * point the port's MSI decoder at the 0xfffff000 doorbell, fill PORT_MSIMAP
 * with the identity vector map, set PORT_MSICFG_EN, then clear the two MSI
 * error bits in PORT_INTSTAT so anything latched afterwards belongs to the OS.
 *
 * Every write is read back.  PORT_MSICFG is read-modify-write so the L2MSINUM
 * field the upstream T602x bring-up block leaves behind is preserved; that
 * field is unused by the msimap-style decoder, and the existing BCM4388
 * transaction has always written `prior | EN`.
 */
int pcie_port_program_msi(const struct pcie_port_bringup_ops *ops, void *context, u64 port_base);

int pcie_init(void);
/* Exact-J414s opt-in: initialize only APCIE port 0 and require link-up. */
int pcie_init_wireless(void);
int pcie_shutdown(void);

#endif
