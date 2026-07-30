/* SPDX-License-Identifier: MIT */

#include "pcie.h"

#ifndef PCIE_T602X_WIRELESS_HOST_TEST
#include "adt.h"
#include "gpio.h"
#include "platform_identity.h"
#include "pmgr.h"
#include "smc.h"
#include "string.h"
#include "tunables.h"
#include "utils.h"
#endif

/*
 * PCI-to-PCI bridge (type 1) header, bus number register at 0x18:
 * primary | secondary << 8 | subordinate << 16 | secondary latency << 24.
 */
#define PCI_BRIDGE_BUS_NUMBER      0x18
#define PCI_BRIDGE_PRIMARY_BUS     GENMASK(7, 0)
#define PCI_BRIDGE_SECONDARY_BUS   GENMASK(15, 8)
#define PCI_BRIDGE_SUBORDINATE_BUS GENMASK(23, 16)
#define PCI_BRIDGE_BUS_RANGE       GENMASK(23, 0)

static int pcie_t602x_bcm4388_restore_rid(const struct pcie_t602x_mmio_ops *ops, void *context,
                                          u64 address, u32 prior)
{
    u32 value;

    if (ops->write32(context, address, prior))
        return -1;
    if (ops->read32(context, address, &value))
        return -1;

    return value == prior ? 0 : -1;
}

static int pcie_t602x_bcm4388_rollback_rids(const struct pcie_t602x_mmio_ops *ops, void *context,
                                            bool restore_rid0, u32 prior_rid0, bool restore_rid1,
                                            u32 prior_rid1)
{
    const u64 base = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;
    bool rid0_failed = false;
    bool rid1_failed = false;

    /* Restore slot 1 first, but always attempt slot 0 as required by the contract. */
    if (restore_rid1)
        rid1_failed = pcie_t602x_bcm4388_restore_rid(ops, context, base + 4, prior_rid1) != 0;
    if (restore_rid0)
        rid0_failed = pcie_t602x_bcm4388_restore_rid(ops, context, base, prior_rid0) != 0;

    if (rid0_failed)
        return PCIE_T602X_BCM4388_ERR_RID0_ROLLBACK;
    if (rid1_failed)
        return PCIE_T602X_BCM4388_ERR_RID1_ROLLBACK;

    return PCIE_T602X_BCM4388_OK;
}

static int pcie_t602x_bcm4388_preflight_rids(const struct pcie_t602x_mmio_ops *ops, void *context,
                                             struct pcie_t602x_bcm4388_rid_transaction *state)
{
    const u64 rid0_address = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;
    const u64 rid1_address = rid0_address + 4;

    /* Read both slots before writing anything so occupied-slot rejection is atomic. */
    if (ops->read32(context, rid0_address, &state->prior_rid0))
        return PCIE_T602X_BCM4388_ERR_RID0_READ;
    if (ops->read32(context, rid1_address, &state->prior_rid1))
        return PCIE_T602X_BCM4388_ERR_RID1_READ;

    if (state->prior_rid0 != 0 && state->prior_rid0 != PCIE_T602X_BCM4388_WIFI_RID2SID)
        return PCIE_T602X_BCM4388_ERR_RID0_OCCUPIED;
    if (state->prior_rid1 != 0 && state->prior_rid1 != PCIE_T602X_BCM4388_BLUETOOTH_RID2SID)
        return PCIE_T602X_BCM4388_ERR_RID1_OCCUPIED;

    return PCIE_T602X_BCM4388_OK;
}

static int pcie_t602x_bcm4388_install_rids(const struct pcie_t602x_mmio_ops *ops, void *context,
                                           struct pcie_t602x_bcm4388_rid_transaction *state)
{
    const u64 rid0_address = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_RID2SID_OFFSET;
    const u64 rid1_address = rid0_address + 4;
    u32 value;
    bool changed_rid0 = false;
    bool attempted_rid1 = false;
    int error;

    if (state->prior_rid0 != PCIE_T602X_BCM4388_WIFI_RID2SID) {
        changed_rid0 = true;
        state->changed_mask |= UINT32_C(1) << 0;
        if (ops->write32(context, rid0_address, PCIE_T602X_BCM4388_WIFI_RID2SID)) {
            error = PCIE_T602X_BCM4388_ERR_RID0_WRITE;
            goto rollback_rid0;
        }
        if (ops->read32(context, rid0_address, &value)) {
            error = PCIE_T602X_BCM4388_ERR_RID0_READBACK_READ;
            goto rollback_rid0;
        }
        if (value != PCIE_T602X_BCM4388_WIFI_RID2SID) {
            error = PCIE_T602X_BCM4388_ERR_RID0_READBACK_MISMATCH;
            goto rollback_rid0;
        }
    }

    if (state->prior_rid1 != PCIE_T602X_BCM4388_BLUETOOTH_RID2SID) {
        attempted_rid1 = true;
        state->changed_mask |= UINT32_C(1) << 1;
        if (ops->write32(context, rid1_address, PCIE_T602X_BCM4388_BLUETOOTH_RID2SID)) {
            error = PCIE_T602X_BCM4388_ERR_RID1_WRITE;
            goto rollback_both;
        }
        if (ops->read32(context, rid1_address, &value)) {
            error = PCIE_T602X_BCM4388_ERR_RID1_READBACK_READ;
            goto rollback_both;
        }
        if (value != PCIE_T602X_BCM4388_BLUETOOTH_RID2SID) {
            error = PCIE_T602X_BCM4388_ERR_RID1_READBACK_MISMATCH;
            goto rollback_both;
        }
    }

    state->active = true;
    return PCIE_T602X_BCM4388_OK;

rollback_both: {
    int rollback_error = pcie_t602x_bcm4388_rollback_rids(
        ops, context, changed_rid0, state->prior_rid0, attempted_rid1, state->prior_rid1);
    return rollback_error ? rollback_error : error;
}

rollback_rid0: {
    int rollback_error =
        pcie_t602x_bcm4388_rollback_rids(ops, context, changed_rid0, state->prior_rid0, false, 0);
    return rollback_error ? rollback_error : error;
}
}

static int pcie_t602x_bcm4388_force_msi_disabled(const struct pcie_t602x_mmio_ops *ops,
                                                 void *context, u32 prior_config)
{
    const u64 address = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET;
    const u32 disabled_config = prior_config & ~PCIE_T602X_PORT_MSI_ENABLE;
    u32 value;

    if (ops->write32(context, address, disabled_config))
        return -1;
    if (ops->read32(context, address, &value))
        return -1;

    return value == disabled_config ? 0 : -1;
}

static int pcie_t602x_bcm4388_require_msi_quiesced(const struct pcie_t602x_mmio_ops *ops,
                                                   void *context, u32 *prior_config)
{
    const u64 address = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET;

    if (ops->read32(context, address, prior_config))
        return PCIE_T602X_BCM4388_ERR_MSI_PREFLIGHT_READ;
    if ((*prior_config & PCIE_T602X_PORT_MSI_ENABLE) != 0)
        return PCIE_T602X_BCM4388_ERR_MSI_NOT_QUIESCED;

    return PCIE_T602X_BCM4388_OK;
}

static int pcie_t602x_bcm4388_disable_msi(const struct pcie_t602x_mmio_ops *ops, void *context,
                                         u32 prior_config)
{
    const u64 address = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET;
    const u32 disabled_config = prior_config & ~PCIE_T602X_PORT_MSI_ENABLE;
    u32 value;

    if (ops->write32(context, address, disabled_config))
        return PCIE_T602X_BCM4388_ERR_MSI_DISABLE_WRITE;
    if (ops->read32(context, address, &value))
        return PCIE_T602X_BCM4388_ERR_MSI_DISABLE_READ;
    if (value != disabled_config)
        return PCIE_T602X_BCM4388_ERR_MSI_DISABLE_MISMATCH;

    return PCIE_T602X_BCM4388_OK;
}

int pcie_t602x_bcm4388_require_port0_msi_disabled(const struct pcie_t602x_mmio_ops *ops,
                                                  void *context)
{
    const u64 address = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET;
    u32 value;

    if (!ops || !ops->read32)
        return PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT;
    if (ops->read32(context, address, &value))
        return PCIE_T602X_BCM4388_ERR_MSI_PREFLIGHT_READ;
    return value == 0 ? PCIE_T602X_BCM4388_OK : PCIE_T602X_BCM4388_ERR_MSI_NOT_QUIESCED;
}

int pcie_t602x_bcm4388_disable_port0_msi(const struct pcie_t602x_mmio_ops *ops, void *context)
{
    const u64 address = PCIE_T602X_BCM4388_PORT0_BASE + PCIE_T602X_PORT_MSI_CONFIG_OFFSET;
    u32 value;

    if (!ops || !ops->read32 || !ops->write32)
        return PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT;
    if (ops->write32(context, address, 0))
        return PCIE_T602X_BCM4388_ERR_MSI_DISABLE_WRITE;
    if (ops->read32(context, address, &value))
        return PCIE_T602X_BCM4388_ERR_MSI_DISABLE_READ;
    return value == 0 ? PCIE_T602X_BCM4388_OK : PCIE_T602X_BCM4388_ERR_MSI_DISABLE_MISMATCH;
}

static int pcie_t602x_bcm4388_fail_msi(const struct pcie_t602x_mmio_ops *ops, void *context,
                                       u32 prior_config, int primary_error)
{
    if (pcie_t602x_bcm4388_force_msi_disabled(ops, context, prior_config))
        return PCIE_T602X_BCM4388_ERR_MSI_DISABLE_RECOVERY;

    return primary_error;
}

static int pcie_t602x_bcm4388_program_msi(const struct pcie_t602x_mmio_ops *ops, void *context,
                                         u32 prior_config)
{
    const u64 base = PCIE_T602X_BCM4388_PORT0_BASE;
    const u64 config_address = base + PCIE_T602X_PORT_MSI_CONFIG_OFFSET;
    const u64 address_lo = base + PCIE_T602X_PORT_MSI_ADDRESS_LO;
    const u64 address_hi = base + PCIE_T602X_PORT_MSI_ADDRESS_HI;
    u32 value;

    if (ops->write32(context, address_lo, PCIE_T602X_BCM4388_MSI_ADDRESS))
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_LO_WRITE);
    if (ops->write32(context, address_hi, 0))
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_HI_WRITE);

    for (u32 vector = 0; vector < PCIE_T602X_PORT_MSI_VECTOR_COUNT; vector++) {
        u64 map_address = base + PCIE_T602X_PORT_MSIMAP_OFFSET + 4 * vector;

        if (ops->write32(context, map_address, PCIE_T602X_MSIMAP_VALID | vector))
            return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                               PCIE_T602X_BCM4388_ERR_MSI_MAP_WRITE(vector));
    }

    if (ops->read32(context, address_lo, &value))
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_LO_READ);
    if (value != PCIE_T602X_BCM4388_MSI_ADDRESS)
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_LO_MISMATCH);
    if (ops->read32(context, address_hi, &value))
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_HI_READ);
    if (value != 0)
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ADDRESS_HI_MISMATCH);

    for (u32 vector = 0; vector < PCIE_T602X_PORT_MSI_VECTOR_COUNT; vector++) {
        u64 map_address = base + PCIE_T602X_PORT_MSIMAP_OFFSET + 4 * vector;

        if (ops->read32(context, map_address, &value))
            return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                               PCIE_T602X_BCM4388_ERR_MSI_MAP_READ(vector));
        if (value != (PCIE_T602X_MSIMAP_VALID | vector))
            return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                               PCIE_T602X_BCM4388_ERR_MSI_MAP_MISMATCH(vector));
    }

    value = prior_config | PCIE_T602X_PORT_MSI_ENABLE;
    if (ops->write32(context, config_address, value))
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ENABLE_WRITE);
    if (ops->read32(context, config_address, &value))
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ENABLE_READ);
    if (value != (prior_config | PCIE_T602X_PORT_MSI_ENABLE))
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_config,
                                           PCIE_T602X_BCM4388_ERR_MSI_ENABLE_MISMATCH);

    return PCIE_T602X_BCM4388_OK;
}

int pcie_t602x_bcm4388_route_port0_rids(const struct pcie_t602x_mmio_ops *ops, void *context,
                                        struct pcie_t602x_bcm4388_rid_transaction *transaction)
{
    int ret;

    if (!ops || !ops->read32 || !ops->write32 || !transaction)
        return PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT;

    *transaction = (struct pcie_t602x_bcm4388_rid_transaction){0};
    ret = pcie_t602x_bcm4388_preflight_rids(ops, context, transaction);
    if (ret)
        return ret;

    /*
     * Retain the token even when the first rollback attempt reports failure.
     * A larger fail-closed transaction may then retry restoration while also
     * disabling its DART stream.
     */
    transaction->active = true;
    ret = pcie_t602x_bcm4388_install_rids(ops, context, transaction);
    return ret;
}

int pcie_t602x_bcm4388_rollback_port0_rids(const struct pcie_t602x_mmio_ops *ops, void *context,
                                           struct pcie_t602x_bcm4388_rid_transaction *transaction)
{
    int ret;

    if (!ops || !ops->read32 || !ops->write32 || !transaction)
        return PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT;
    if (!transaction->active)
        return PCIE_T602X_BCM4388_OK;

    ret = pcie_t602x_bcm4388_rollback_rids(
        ops, context, (transaction->changed_mask & (UINT32_C(1) << 0)) != 0,
        transaction->prior_rid0, (transaction->changed_mask & (UINT32_C(1) << 1)) != 0,
        transaction->prior_rid1);
    if (!ret)
        *transaction = (struct pcie_t602x_bcm4388_rid_transaction){0};
    return ret;
}

int pcie_t602x_bcm4388_enable_port0_msi(const struct pcie_t602x_mmio_ops *ops, void *context)
{
    int ret;

    if (!ops || !ops->read32 || !ops->write32)
        return PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT;
    ret = pcie_t602x_bcm4388_require_port0_msi_disabled(ops, context);
    if (ret)
        return ret;
    return pcie_t602x_bcm4388_program_msi(ops, context, 0);
}

int pcie_t602x_bcm4388_setup_port0(const struct pcie_t602x_mmio_ops *ops, void *context)
{
    struct pcie_t602x_bcm4388_rid_transaction rid_state;
    u32 prior_msi_config;
    int rollback_error;
    int ret;

    if (!ops || !ops->read32 || !ops->write32)
        return PCIE_T602X_BCM4388_ERR_INVALID_ARGUMENT;

    /* Never destroy a decoder that another owner may already be using. */
    ret = pcie_t602x_bcm4388_require_msi_quiesced(ops, context, &prior_msi_config);
    if (ret)
        return ret;

    rid_state = (struct pcie_t602x_bcm4388_rid_transaction){0};
    ret = pcie_t602x_bcm4388_preflight_rids(ops, context, &rid_state);
    if (ret)
        return ret;

    /* No RID or MSI state is changed before this fail-closed disable succeeds. */
    ret = pcie_t602x_bcm4388_disable_msi(ops, context, prior_msi_config);
    if (ret)
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_msi_config, ret);

    ret = pcie_t602x_bcm4388_install_rids(ops, context, &rid_state);
    if (ret)
        return pcie_t602x_bcm4388_fail_msi(ops, context, prior_msi_config, ret);

    ret = pcie_t602x_bcm4388_program_msi(ops, context, prior_msi_config);
    if (!ret)
        return PCIE_T602X_BCM4388_OK;

    /* RID2SID controls DMA independently of MSI; unwind newly acquired slots. */
    rollback_error = pcie_t602x_bcm4388_rollback_port0_rids(ops, context, &rid_state);
    return rollback_error ? rollback_error : ret;
}

static u32 pcie_clamp_u32(u32 value, u32 min, u32 max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void pcie_perst_delays_from_adt(struct pcie_perst_delays *out, bool have_refclk_to_perst,
                                u32 refclk_to_perst, bool have_perst_to_config, u32 perst_to_config)
{
    u32 refclk_us = PCIE_PERST_REFCLK_TO_PERST_MIN_US;
    u32 config_us = PCIE_PERST_TO_CONFIG_MIN_US;

    if (!out)
        return;

    /* t-refclk-to-perst is microseconds; see the unit derivation in pcie.h. */
    if (have_refclk_to_perst)
        refclk_us = refclk_to_perst;

    /* perst-to-config is milliseconds; guard the scale against overflow. */
    if (have_perst_to_config) {
        if (perst_to_config > PCIE_PERST_TO_CONFIG_MAX_US / 1000)
            config_us = PCIE_PERST_TO_CONFIG_MAX_US;
        else
            config_us = perst_to_config * 1000;
    }

    out->refclk_to_perst_us = pcie_clamp_u32(refclk_us, PCIE_PERST_REFCLK_TO_PERST_MIN_US,
                                             PCIE_PERST_REFCLK_TO_PERST_MAX_US);
    out->perst_to_config_us =
        pcie_clamp_u32(config_us, PCIE_PERST_TO_CONFIG_MIN_US, PCIE_PERST_TO_CONFIG_MAX_US);
}

/*
 * Power-rail ownership.
 *
 * A port's rail is never declared on the bridge itself.  It lives on whatever
 * node claims that bridge via a `function-pcie_port_control*` property whose
 * first argument is the bridge's own AAPL,phandle.  On J414s:
 *
 *   /arm-io/apcie/pci-bridge1/pcie-sdreader
 *       function-pcie_port_control_sd -> args[0] = 101 = pci-bridge1
 *       function-sd_pwr_en            -> SMC key "gP16", mode word 0
 *   /amfm                                (top level, NOT under apcie)
 *       function-pcie_port_control    -> args[0] = 98  = pci-bridge0
 *       function-reg_on               -> SMC key "gP0d", mode word 0x800000
 *
 * Both verified live against the J414s ADT.  Note the WiFi/BT owner is not a
 * descendant of the bridge at all, which is why a child-only search finds the
 * SD rail and silently misses the wireless one.
 */
/*
 * Everything from here to the matching #endif reads the ADT and talks to the
 * SMC directly, and is reachable only from pcie_init_controller(), which is
 * itself hardware-only.  It has to stay inside this guard: tests/pcie builds
 * src/pcie.c with PCIE_T602X_WIRELESS_HOST_TEST to cover the callback-driven
 * BCM4388 RID/MSI and port bring-up helpers, and an unguarded adt/smc
 * reference here breaks that whole suite -- which is exactly what happened
 * between 3b24e2cb and this commit.
 */
#ifndef PCIE_T602X_WIRELESS_HOST_TEST
static const char *const pcie_port_control_props[] = {
    "function-pcie_port_control",
    "function-pcie_port_control_sd",
};

/* Rail property names, tried in order on a resolved owner node. */
static const char *const pcie_port_rail_names[] = {
    "reg_on",
    "sd_pwr_en",
    "pwr_en",
};

static bool pcie_node_controls_bridge(int node, u32 bridge_phandle)
{
    for (size_t i = 0; i < ARRAY_SIZE(pcie_port_control_props); i++) {
        struct apple_gpio_function fn;
        const void *value;
        u32 len = 0;

        value = adt_getprop(adt, node, pcie_port_control_props[i], &len);
        if (!value || !len)
            continue;
        if (apple_gpio_parse_function(value, len, &fn) < 0)
            continue;
        /* args[0] lands in fn.pin for this record shape. */
        if (fn.pin == bridge_phandle)
            return true;
    }

    return false;
}

/*
 * Deliberately NOT a recursive whole-ADT search.  An earlier version walked
 * the entire tree from the root looking for the owning node; on hardware that
 * hard-reset the machine mid-`pcie_init` (serial dropped outright), and this
 * codebase already has a documented history of unreliable generic tree
 * traversal.  There are only two places an owner is ever found, so look in
 * exactly those two and keep the traversal bounded and deterministic:
 *
 *   1. the bridge's own children  -- /arm-io/apcie/pci-bridgeN/<endpoint>
 *   2. the fixed top-level node   -- /amfm
 *
 * Both are confirmed against the live J414s ADT, and each candidate still has
 * to prove ownership via function-pcie_port_control* naming this bridge, so a
 * machine that wires them differently gets nothing rather than the wrong rail.
 */
static int pcie_find_rail_owner(int bridge_offset, u32 bridge_phandle)
{
    static const char *const owner_paths[] = {"/amfm"};
    int child_count, child;

    if (bridge_phandle == 0)
        return -1;

    child_count = adt_get_child_count(adt, bridge_offset);
    child = adt_first_child_offset(adt, bridge_offset);
    while (child_count-- > 0 && child > 0) {
        if (pcie_node_controls_bridge(child, bridge_phandle))
            return child;
        child = adt_next_sibling_offset(adt, child);
    }

    for (size_t i = 0; i < ARRAY_SIZE(owner_paths); i++) {
        int node = adt_path_offset(adt, owner_paths[i]);

        if (node < 0)
            continue;
        if (pcie_node_controls_bridge(node, bridge_phandle))
            return node;
    }

    return -1;
}

/*
 * Raise a port's power rail, if it has one.  Returns true when a rail was
 * actually driven, so the caller knows whether Tpvperl applies.
 *
 * Best-effort throughout: no owner, no rail, an unresolvable key or a dead SMC
 * all log and continue.  A wrong or missing rail must degrade a device, never
 * refuse the boot.
 */
/*
 * Two things are OPT-IN and deliberately off for the generic pcie_init() path:
 * the port power rails, and the root ports' bridge bus numbers.  They are
 * co-requisite -- routing without power reaches a dead link, power without
 * routing reaches a device nothing can address -- so one flag gates both and
 * the invariant cannot be half-applied.
 *
 * Doing either is correct firmware behaviour, but it is premature for a
 * Windows boot: it hands Windows three PCIe devices (BCM4388 Wi-Fi, its
 * Bluetooth function, and the GL9755 SD reader) whose MSI delivery is not yet
 * activated on this platform.  Measured consequence -- with rails on, Windows
 * bugchecked BUGCODE_USB3_DRIVER (0x144) on a gpu-only profile, and
 * INACCESSIBLE_BOOT_DEVICE (0x7B) on ans-gpu; xHCI shares the interrupt setup
 * these undeliverable devices disturb.  Both boots showed "rail" asserted
 * twice with no link failures, so the rails were the common factor.
 *
 * pcie_init_wireless() opts in, because wireless/SD bring-up cannot proceed
 * without power and the SID-1 handoff cannot read endpoint config space
 * without routing.  Flip the default only once MSI delivery works.
 */
static bool pcie_wireless_profile = false;

/*
 * Give a root port a bus number range so config space behind it is routable.
 *
 * m1n1 has never done this: it trains the links and stops.  Both J414s root
 * ports therefore come out of bring-up with primary = secondary = subordinate
 * = 0, and a bridge whose secondary bus is 0 forwards no configuration request
 * at all -- every config read of the endpoint returns 0xffffffff even though
 * LINKSTS reports the link up.  Measured on hardware: after this controller
 * reported "Port 0 link up (status 0xd9000001)", config 0x18 on both root
 * ports read zero and neither endpoint was visible; writing the bus numbers by
 * hand made 14e4:4434 (Wi-Fi, bus 1 dev 0 fn 0), 14e4:5f72 (Bluetooth, fn 1)
 * and 17a0:9755 (GL9755 SD reader, bus 2 dev 0 fn 0) appear immediately.
 *
 * Numbering: each Apple root port owns exactly one downstream bus, and the
 * root ports themselves are bus 0 devices 0..N-1 (which is precisely why
 * config_base is controller_config_base + (port << 15) -- ECAM device number
 * == port index).  So port N gets secondary = subordinate = N + 1, which is
 * also what a normal OS enumerator ends up assigning.  Primary stays 0.
 *
 * This is not a takeover: any OS PCI enumerator reprograms these registers
 * from scratch during its own scan, and on profiles where the rails stay down
 * there is no powered device behind the bridge for the routing to expose.
 * Best-effort -- a read-back mismatch logs and continues, because a bridge
 * that will not accept a bus number is a diagnosis for the caller to fail on,
 * not a reason to abort the whole controller.
 */
static bool pcie_program_bridge_bus_numbers(u64 config_base, u32 port)
{
    u32 secondary = port + 1;
    u32 want = FIELD_PREP(PCI_BRIDGE_PRIMARY_BUS, 0) |
               FIELD_PREP(PCI_BRIDGE_SECONDARY_BUS, secondary) |
               FIELD_PREP(PCI_BRIDGE_SUBORDINATE_BUS, secondary);
    u32 got;

    if (!pcie_wireless_profile)
        return false;

    /* Preserve the secondary latency timer in the top byte. */
    mask32(config_base + PCI_BRIDGE_BUS_NUMBER, PCI_BRIDGE_BUS_RANGE, want);
    got = read32(config_base + PCI_BRIDGE_BUS_NUMBER) & PCI_BRIDGE_BUS_RANGE;
    if (got != want) {
        printf("pcie: Port %d bus-number write did not stick (%#x, wanted %#x)\n", port, got,
               want);
        return false;
    }

    printf("pcie: Port %d routed as primary 0, secondary %u, subordinate %u\n", port, secondary,
           secondary);
    return true;
}

static bool pcie_enable_port_rail(int bridge_offset, int port)
{
    u32 bridge_phandle = 0;
    int owner;

    if (!pcie_wireless_profile)
        return false;

    if (ADT_GETPROP(adt, bridge_offset, "AAPL,phandle", &bridge_phandle) < 0)
        return false;

    owner = pcie_find_rail_owner(bridge_offset, bridge_phandle);
    if (owner < 0)
        return false;

    for (size_t i = 0; i < ARRAY_SIZE(pcie_port_rail_names); i++) {
        struct apple_smc_rail rail = {0};
        smc_dev_t *smc;
        u32 value;

        if (apple_smc_resolve_function(owner, pcie_port_rail_names[i], &rail) != 0 || !rail.valid)
            continue;

        smc = smc_init();
        if (!smc) {
            printf("pcie: Port %d rail %s: SMC unavailable\n", port, pcie_port_rail_names[i]);
            return false;
        }

        /*
         * CMD_OUTPUT | level, never the ADT's own mode word ORed with the
         * level.  src/dcp.c does the latter and it happens to work for the
         * HDMI rails because their mode word is 0x800000, but the SD rail's is
         * 0 -- `mode | 1` would write a bare 0x1 (CMD_ACTION) and drive
         * nothing.  CMD_OUTPUT is verified against SMCG.asl's published
         * ntasp,smc-gpio-cmd-output and proven on hardware for "gP16".
         */
        value = APPLE_SMC_GPIO_CMD_OUTPUT | 1;
        if (smc_write_u32(smc, rail.key, value) < 0) {
            printf("pcie: Port %d rail %s write failed (key %#x)\n", port, pcie_port_rail_names[i],
                   rail.key);
            smc_shutdown(smc);
            return false;
        }

        printf("pcie: Port %d rail %s enabled (SMC key %#x <- %#x)\n", port,
               pcie_port_rail_names[i], rail.key, value);
        smc_shutdown(smc);
        return true;
    }

    return false;
}
#endif /* !PCIE_T602X_WIRELESS_HOST_TEST */

static bool pcie_bringup_ops_valid(const struct pcie_port_bringup_ops *ops,
                                   const struct pcie_port_bringup *cfg)
{
    if (!ops || !cfg)
        return false;
    if (!ops->read32 || !ops->write32 || !ops->set32 || !ops->clear32 || !ops->poll32 ||
        !ops->delay_us)
        return false;
    if (cfg->have_perst_gpio && !ops->perst_set)
        return false;
    return true;
}

int pcie_port_release_perst(const struct pcie_port_bringup_ops *ops, void *context,
                            const struct pcie_port_bringup *cfg)
{
    bool drive_perst;

    if (!pcie_bringup_ops_valid(ops, cfg))
        return PCIE_PORT_BRINGUP_ERR_INVALID_ARGUMENT;

    /*
     * Never reset a port whose link someone else already brought up, and never
     * touch a pad we could not resolve.
     */
    drive_perst = cfg->have_perst_gpio && !cfg->link_was_up;

    if (ops->set32(context, cfg->port_base + PCIE_PORT_APPCLK, PCIE_PORT_APPCLK_EN))
        return PCIE_PORT_BRINGUP_ERR_APPCLK;

    /*
     * Assert PERST# before the clocks come up.  apple_pcie_setup_link() takes
     * the pad as GPIOD_OUT_HIGH (logical asserted) for exactly this reason:
     * "The Aquantia AQC113 10GB nic used desktop macs is sensitive to
     * deasserting it without prior clock setup."
     */
    if (drive_perst && ops->perst_set(context, true))
        return PCIE_PORT_BRINGUP_ERR_PERST_ASSERT;

    if (cfg->port_phy_base) {
        u64 lane_cfg = cfg->port_phy_base + PCIE_PHY_LANE_CFG;

        if (ops->clear32(context, lane_cfg,
                         PCIE_PHY_LANE_CFG_REFCLK0REQ | PCIE_PHY_LANE_CFG_REFCLK1REQ))
            return PCIE_PORT_BRINGUP_ERR_PHY_REQ;

        if (ops->set32(context, lane_cfg, PCIE_PHY_LANE_CFG_REFCLK0REQ))
            return PCIE_PORT_BRINGUP_ERR_PHY_REQ;
        if (ops->poll32(context, lane_cfg, PCIE_PHY_LANE_CFG_REFCLK0ACK,
                        PCIE_PHY_LANE_CFG_REFCLK0ACK, cfg->phy_ack_timeout_us))
            return PCIE_PORT_BRINGUP_ERR_PHY_CLK0_ACK;

        if (ops->set32(context, lane_cfg, PCIE_PHY_LANE_CFG_REFCLK1REQ))
            return PCIE_PORT_BRINGUP_ERR_PHY_REQ;
        if (ops->poll32(context, lane_cfg, PCIE_PHY_LANE_CFG_REFCLK1ACK,
                        PCIE_PHY_LANE_CFG_REFCLK1ACK, cfg->phy_ack_timeout_us))
            return PCIE_PORT_BRINGUP_ERR_PHY_CLK1_ACK;

        if (ops->clear32(context, lane_cfg, PCIE_PHY_LANE_CFG_UNK14))
            return PCIE_PORT_BRINGUP_ERR_REFCLK_EN;
        if (ops->set32(context, lane_cfg, PCIE_PHY_LANE_CFG_REFCLKEN0))
            return PCIE_PORT_BRINGUP_ERR_REFCLK_EN;
        if (ops->set32(context, lane_cfg, PCIE_PHY_LANE_CFG_REFCLKEN1))
            return PCIE_PORT_BRINGUP_ERR_REFCLK_EN;
    }

    /* Tperst-clk: stable reference clock -> PERST# deassertion. */
    ops->delay_us(context, cfg->delays.refclk_to_perst_us);

    if (ops->set32(context, cfg->port_base + cfg->perst_reg, PCIE_PORT_PERST_OFF))
        return PCIE_PORT_BRINGUP_ERR_PERST_RELEASE_REG;

    if (drive_perst && ops->perst_set(context, false))
        return PCIE_PORT_BRINGUP_ERR_PERST_RELEASE_GPIO;

    return PCIE_PORT_BRINGUP_OK;
}

int pcie_port_start_link(const struct pcie_port_bringup_ops *ops, void *context,
                         const struct pcie_port_bringup *cfg)
{
    u32 link_status = 0;

    if (!pcie_bringup_ops_valid(ops, cfg))
        return PCIE_PORT_BRINGUP_ERR_INVALID_ARGUMENT;

    if (cfg->port_phy_base &&
        ops->set32(context, cfg->port_phy_base + PCIE_PHY_LANE_CFG, PCIE_PHY_LANE_CFG_REFCLKCGEN))
        return PCIE_PORT_BRINGUP_ERR_REFCLK_CGEN;

    if (ops->read32(context, cfg->port_base + PCIE_PORT_LINKSTS, &link_status))
        return PCIE_PORT_BRINGUP_ERR_LINK_DOWN;

    /* Already trained: leave LTSSM alone, exactly as apple_pcie_setup_port() does. */
    if (link_status & PCIE_PORT_LINKSTS_UP)
        return PCIE_PORT_BRINGUP_OK;

    if (ops->write32(context, cfg->port_base + PCIE_PORT_LTSSMCTL, PCIE_PORT_LTSSMCTL_START))
        return PCIE_PORT_BRINGUP_ERR_LTSSM_START;

    if (ops->poll32(context, cfg->port_base + PCIE_PORT_LINKSTS, PCIE_PORT_LINKSTS_UP,
                    PCIE_PORT_LINKSTS_UP, cfg->link_up_timeout_us))
        return PCIE_PORT_BRINGUP_ERR_LINK_DOWN;

    return PCIE_PORT_BRINGUP_OK;
}

#ifndef PCIE_T602X_WIRELESS_HOST_TEST

/*
 * The ADT uses 17 register sets:
 *
 * 0:  90000000 00000006 10000000 00000000  ECAM
 * 1:  80000000 00000006 00040000 00000000  RC
 * 2:  80080000 00000006 00090000 00000000  PHY
 * 3:  800c0000 00000006 00020000 00000000  PHY IP
 * 4:  8c000000 00000006 00004000 00000000  AXI
 * 5:  3d2bc000 00000000 00001000 00000000  fuses
 * 6:  81000000 00000006 00008000 00000000  port 0 config
 * 7:  81010000 00000006 00001000 00000000  port 0 LTSSM debug
 * 8:  80084000 00000006 00004000 00000000  port 0 PHY
 * 9:  800c8000 00000006 00016610 00000000  port 0 PHY IP
   <macOS 12.0 RC and later add a per-port Intr2AXI reg here>
 * 10: 82000000 00000006 00008000 00000000  port 1 config
 * 11: 82010000 00000006 00001000 00000000  port 1 LTSSM debug
 * 12: 80088000 00000006 00004000 00000000  port 1 PHY
 * 13: 800d0000 00000006 00006000 00000000  port 1 PHY IP
   <...>
 * 14: 83000000 00000006 00008000 00000000  port 2 config
 * 15: 83010000 00000006 00001000 00000000  port 2 LTSSM debug
 * 16: 8008c000 00000006 00004000 00000000  port 2 PHY
 * 17: 800d8000 00000006 00006000 00000000  port 2 PHY IP
   <...>
 */

/* PHY registers */

#define APCIE_PHY_CTRL         0x000
#define APCIE_PHY_CTRL_CLK0REQ BIT(0)
#define APCIE_PHY_CTRL_CLK1REQ BIT(1)
#define APCIE_PHY_CTRL_CLK0ACK BIT(2)
#define APCIE_PHY_CTRL_CLK1ACK BIT(3)
#define APCIE_PHY_CTRL_RESET   BIT(7)

#define APCIE_PHYIF_CTRL     0x024
#define APCIE_PHYIF_CTRL_RUN BIT(0)

/* PHY common registers */
#define APCIE_PHYCMN_CLK         0x000
#define APCIE_PHYCMN_CLK_MODE    GENMASK(1, 0) /* Guesswork */
#define APCIE_PHYCMN_CLK_MODE_ON 1
#define APCIE_PHYCMN_CLK_100MHZ  BIT(31)

/* Port registers */

#define APCIE_PORT_LINKSTS      0x208
#define APCIE_PORT_LINKSTS_UP   BIT(0)
#define APCIE_PORT_LINKSTS_BUSY BIT(2)
#define APCIE_PORT_LINKSTS_L2   BIT(6)

#define APCIE_PORT_APPCLK    0x800
#define APCIE_PORT_APPCLK_EN BIT(0)

#define APCIE_PORT_STATUS     0x804
#define APCIE_PORT_STATUS_RUN BIT(0)

#define APCIE_PORT_RESET     0x814
#define APCIE_PORT_RESET_DIS BIT(0)

#define APCIE_T602X_PORT_RESET 0x82c

/* PCIe capability registers */
#define PCIE_CAP_BASE    0x70
#define PCIE_LNKCAP      0x0c
#define PCIE_LNKCAP_SLS  GENMASK(3, 0)
#define PCIE_LNKCAP_MLW  GENMASK(9, 4)
#define PCIE_LNKCAP2     0x2c
#define PCIE_LNKCAP2_SLS GENMASK(6, 1)
#define PCIE_LNKCTL2     0x30
#define PCIE_LNKCTL2_TLS GENMASK(3, 0)

/* DesignWare PCIe Core registers */

#define DWC_DBI_RO_WR    0x8bc
#define DWC_DBI_RO_WR_EN BIT(0)

#define DWC_DBI_PORT_LINK_CONTROL        0x710
#define DWC_DBI_PORT_LINK_DLL_LINK_EN    BIT(5)
#define DWC_DBI_PORT_LINK_FAST_LINK_MODE BIT(7)
#define DWC_DBI_PORT_LINK_MODE           GENMASK(21, 16)
#define DWC_DBI_PORT_LINK_MODE_1_LANE    0x1
#define DWC_DBI_PORT_LINK_MODE_2_LANES   0x3
#define DWC_DBI_PORT_LINK_MODE_4_LANES   0x7
#define DWC_DBI_PORT_LINK_MODE_8_LANES   0xf
#define DWC_DBI_PORT_LINK_MODE_16_LANES  0x1f

#define DWC_DBI_LINK_WIDTH_SPEED_CONTROL 0x80c
#define DWC_DBI_LINK_WIDTH               GENMASK(12, 8)
#define DWC_DBI_SPEED_CHANGE             BIT(17)

#define PHY_STRIDE   0x4000
#define PHYIP_STRIDE 0x40000

struct fuse_bits {
    u16 src_reg;
    u16 tgt_reg;
    u8 src_bit;
    u8 tgt_bit;
    u8 width;
};

const struct fuse_bits pcie_fuse_bits_t8103[] = {
    {0x0084, 0x6238, 4, 0, 6},   {0x0084, 0x6220, 10, 14, 3}, {0x0084, 0x62a4, 13, 17, 2},
    {0x0418, 0x522c, 27, 9, 2},  {0x0418, 0x522c, 13, 12, 3}, {0x0418, 0x5220, 18, 14, 3},
    {0x0418, 0x52a4, 21, 17, 2}, {0x0418, 0x522c, 23, 16, 5}, {0x0418, 0x5278, 23, 20, 3},
    {0x0418, 0x5018, 31, 2, 1},  {0x041c, 0x1204, 0, 2, 5},   {},
};

const struct fuse_bits pcie_fuse_bits_t6000[] = {
    {0x004c, 0x1004, 3, 2, 5},   {0x0048, 0x522c, 26, 16, 5}, {0x0048, 0x522c, 29, 9, 2},
    {0x0048, 0x522c, 26, 12, 3}, {0x0048, 0x522c, 26, 16, 5}, {0x0048, 0x52a4, 24, 17, 2},
    {0x004c, 0x5018, 2, 3, 1},   {0x0048, 0x50a4, 14, 17, 2}, {0x0048, 0x62a4, 14, 17, 2},
    {0x0048, 0x6220, 8, 14, 3},  {0x0048, 0x6238, 2, 0, 6},   {},
};

/* clang-format off */
const struct fuse_bits pcie_fuse_bits_t8112[] = {
    {0x0490, 0x6238, 0, 0, 6},   {0x0490, 0x6220, 6, 14, 3},  {0x0490, 0x62a4, 12, 17, 2},
    {0x0490, 0x5018, 14, 2, 1},  {0x0490, 0x5220, 15, 14, 3}, {0x0490, 0x52a4, 18, 17, 2},
    {0x0490, 0x5278, 20, 20, 3}, {0x0490, 0x522c, 23, 12, 3}, {0x0490, 0x522c, 26, 9, 2},
    {0x0490, 0x522c, 28, 16, 4}, {0x0494, 0x522c, 0, 20, 1},  {0x0494, 0x1204, 5, 2, 5},
    {},
};
/* clang-format on */

enum apcie_type {
    APCIE_T81XX = 0,
    APCIE_T602X = 1,
};

struct reg_info {
    enum apcie_type type;
    int shared_reg_count;
    int config_idx;
    int rc_idx;
    int phy_common_idx;
    int phy_idx;
    int phy_ip_idx;
    int axi_idx;
    int fuse_idx;
    bool alt_phy_start;
};

static const struct reg_info regs_t8xxx_t600x = {
    .type = APCIE_T81XX,
    .shared_reg_count = 6,
    .config_idx = 0,
    .rc_idx = 1,
    .phy_common_idx = -1,
    .phy_idx = 2,
    .phy_ip_idx = 3,
    .axi_idx = 4,
    .fuse_idx = 5,
};

static const struct reg_info regs_t602x = {
    .type = APCIE_T602X,
    .shared_reg_count = 8,
    .config_idx = 0,
    .rc_idx = 1,
    // 2 = phy unknown?
    .phy_common_idx = 3,
    .phy_idx = 4,
    .phy_ip_idx = 5,
    .axi_idx = 6,
    .fuse_idx = 7,
};

static bool pcie_initialized = false;

enum PCIE_CONTROLLERS {
    APCIE,
    APCIE_GE0,
    APCIE_GE1,
    NUM_CONTROLLERS,
};

#define MAX_PHYS 4

struct state {
    int num_phys;
    u64 rc_base;
    u64 phy_common_base;
    u64 phy_base[MAX_PHYS];
    u64 phy_ip_base[MAX_PHYS];
    u64 fuse_base;
    u32 port_count;
    u64 port_base[8];
    u64 port_ltssm_base[8];
    u64 port_phy_base[8];
    u64 port_intr2axi_base[8];
    struct apple_gpio_pin port_perst_gpio[8];
    const struct reg_info *pcie_regs;
    u32 initialized_port_mask;
    bool initialized;
};

static struct state controllers[NUM_CONTROLLERS];

/*
 * Hardware backing for the bring-up sequence.  `context` is the resolved
 * PERST# pad; it is only dereferenced when the sequence was told the pad
 * resolved (have_perst_gpio), and apple_gpio_set_output() rejects an
 * unresolved pin anyway.
 */
static int pcie_hw_read32(void *context, u64 address, u32 *value)
{
    UNUSED(context);
    *value = read32(address);
    return 0;
}

static int pcie_hw_write32(void *context, u64 address, u32 value)
{
    UNUSED(context);
    write32(address, value);
    return 0;
}

static int pcie_hw_set32(void *context, u64 address, u32 set)
{
    UNUSED(context);
    set32(address, set);
    return 0;
}

static int pcie_hw_clear32(void *context, u64 address, u32 clear)
{
    UNUSED(context);
    clear32(address, clear);
    return 0;
}

static int pcie_hw_poll32(void *context, u64 address, u32 mask, u32 target, u32 timeout_us)
{
    UNUSED(context);
    return poll32(address, mask, target, timeout_us);
}

static void pcie_hw_delay_us(void *context, u32 us)
{
    UNUSED(context);
    udelay(us);
}

static int pcie_hw_perst_set(void *context, bool asserted)
{
    /*
     * PERST# is active low (reset-gpios = <&pinctrl_ap N GPIO_ACTIVE_LOW>), so
     * asserting reset drives the pad to 0 and releasing drives it to 1.
     */
    return apple_gpio_set_output((const struct apple_gpio_pin *)context, !asserted);
}

static const struct pcie_port_bringup_ops pcie_hw_bringup_ops = {
    .read32 = pcie_hw_read32,
    .write32 = pcie_hw_write32,
    .set32 = pcie_hw_set32,
    .clear32 = pcie_hw_clear32,
    .poll32 = pcie_hw_poll32,
    .delay_us = pcie_hw_delay_us,
    .perst_set = pcie_hw_perst_set,
};

static int pcie_init_controller(int controller, const char *path, u32 allowed_port_mask,
                                bool require_link_up)
{
    struct state *state = &controllers[controller];
    int adt_path[8];
    int adt_offset;
    u32 lane_mode = DWC_DBI_PORT_LINK_MODE_1_LANE;
    u32 link_width = 1;
    const struct fuse_bits *fuse_bits;

    state->initialized = false;
    state->initialized_port_mask = 0;
    state->num_phys = 1;
    memset(state->port_perst_gpio, 0, sizeof(state->port_perst_gpio));

    adt_offset = adt_path_offset_trace(adt, path, adt_path);
    if (adt_offset < 0) {
        printf("pcie: Error getting node %s\n", path);
        return -1;
    }

    if (adt_is_compatible(adt, adt_offset, "apcie,t8103")) {
        fuse_bits = pcie_fuse_bits_t8103;
        state->pcie_regs = &regs_t8xxx_t600x;
        printf("pcie: Initializing t8103 PCIe controller\n");
    } else if (adt_is_compatible(adt, adt_offset, "apcie,t6000")) {
        fuse_bits = pcie_fuse_bits_t6000;
        state->pcie_regs = &regs_t8xxx_t600x;
        printf("pcie: Initializing t6000 PCIe controller\n");
    } else if (adt_is_compatible(adt, adt_offset, "apcie,t8112")) {
        fuse_bits = pcie_fuse_bits_t8112;
        state->pcie_regs = &regs_t8xxx_t600x;
        printf("pcie: Initializing t8112 PCIe controller\n");
    } else if (adt_is_compatible(adt, adt_offset, "apcie,t6020")) {
        fuse_bits = NULL;
        state->pcie_regs = &regs_t602x;
        printf("pcie: Initializing t6020 PCIe controller\n");
    } else if (adt_is_compatible(adt, adt_offset, "apcie-ge,t6020")) {
        u32 lane_cfg;
        fuse_bits = NULL;
        state->pcie_regs = &regs_t602x;

        printf("pcie: Initializing t6020 PCIe GE controller\n");
        if (ADT_GETPROP(adt, adt_offset, "lane-cfg", &lane_cfg) < 0) {
            printf("pcie: Error getting lane_cfg for %s\n", path);
            return -1;
        }
        switch (lane_cfg) {
            case 0:
                state->num_phys = 4;
                lane_mode = DWC_DBI_PORT_LINK_MODE_16_LANES;
                link_width = 16;
                break;
            case 1:
                state->num_phys = 2;
                lane_mode = DWC_DBI_PORT_LINK_MODE_8_LANES;
                link_width = 8;
                break;
            default:
                printf("pcie: Unknown lane config %d for %s\n", lane_cfg, path);
                return -1;
        }
    } else {
        printf("pcie: Unsupported compatible\n");
        return -1;
    }

    if (ADT_GETPROP(adt, adt_offset, "#ports", &state->port_count) < 0) {
        printf("pcie: Error getting port count for %s\n", path);
        return -1;
    }

    u64 controller_config_base;
    if (adt_get_reg(adt, adt_path, "reg", state->pcie_regs->config_idx,
                    &controller_config_base, NULL)) {
        printf("pcie: Error getting reg with index %d for %s\n", state->pcie_regs->config_idx,
               path);
        return -1;
    }

    if (adt_get_reg(adt, adt_path, "reg", state->pcie_regs->rc_idx, &state->rc_base, NULL)) {
        printf("pcie: Error getting reg with index %d for %s\n", state->pcie_regs->rc_idx, path);
        return -1;
    }

    if (state->pcie_regs->phy_common_idx != -1) {
        if (adt_get_reg(adt, adt_path, "reg", state->pcie_regs->phy_common_idx,
                        &state->phy_common_base, NULL)) {
            printf("pcie: Error getting reg with index %d for %s\n", state->pcie_regs->phy_idx,
                   path);
            return -1;
        }
    } else {
        state->phy_common_base = 0;
    }

    if (adt_get_reg(adt, adt_path, "reg", state->pcie_regs->phy_idx, &state->phy_base[0], NULL)) {
        printf("pcie: Error getting reg with index %d for %s\n", state->pcie_regs->phy_idx, path);
        return -1;
    }

    if (adt_get_reg(adt, adt_path, "reg", state->pcie_regs->phy_ip_idx, &state->phy_ip_base[0],
                    NULL)) {
        printf("pcie: Error getting reg with index %d for %s\n", state->pcie_regs->phy_ip_idx,
               path);
        return -1;
    }

    for (int phy = 1; phy < state->num_phys; phy++) {
        state->phy_base[phy] = state->phy_base[0] + PHY_STRIDE * phy;
        state->phy_ip_base[phy] = state->phy_ip_base[0] + PHYIP_STRIDE * phy;
    }

    if (adt_get_reg(adt, adt_path, "reg", state->pcie_regs->fuse_idx, &state->fuse_base, NULL)) {
        printf("pcie: Error getting reg with index %d for %s\n", state->pcie_regs->fuse_idx, path);
        return -1;
    }

    u32 reg_len;
    if (!adt_getprop(adt, adt_offset, "reg", &reg_len)) {
        printf("pcie: Error getting reg length for %s\n", path);
        return -1;
    }

    int port_regs = (reg_len / 16) - state->pcie_regs->shared_reg_count;

    if (port_regs % state->port_count) {
        printf("pcie: %d port registers do not evenly divide into %d ports\n", port_regs,
               state->port_count);
        return -1;
    }

    int port_reg_cnt = port_regs / state->port_count;
    printf("pcie: ADT uses %d reg entries per port\n", port_reg_cnt);

    if (pmgr_adt_power_enable(path)) {
        printf("pcie: Error enabling power for %s\n", path);
        return -1;
    }

    if (tunables_apply_local(path, "apcie-axi2af-tunables", state->pcie_regs->axi_idx)) {
        printf("pcie: Error applying %s for %s\n", "apcie-axi2af-tunables", path);
        return -1;
    }

    /* ??? */
    if (controller == APCIE)
        write32(state->rc_base + 0x4, 0);

    if (!adt_getprop(adt, adt_offset, "apcie-common-tunables", NULL)) {
        printf("pcie: No common tunables\n");
    } else if (tunables_apply_local(path, "apcie-common-tunables", state->pcie_regs->rc_idx)) {
        printf("pcie: Error applying %s for %s\n", "apcie-common-tunables", path);
        return -1;
    }

    /*
     * Initialize PHY.
     */

    if (!adt_getprop(adt, adt_offset, "apcie-phy-tunables", NULL)) {
        printf("pcie: No PHY tunables\n");
    } else if (tunables_apply_local(path, "apcie-phy-tunables", state->pcie_regs->phy_idx)) {
        printf("pcie: Error applying %s for %s\n", "apcie-phy-tunables", path);
        return -1;
    }

    if (state->pcie_regs->type == APCIE_T602X) {
        if (poll32(state->phy_common_base + APCIE_PHYCMN_CLK, APCIE_PHYCMN_CLK_100MHZ,
                   APCIE_PHYCMN_CLK_100MHZ, 250000)) {
            printf("pcie: Reference clock not available\n");
            return -1;
        }
    }

    for (int phy = 0; phy < state->num_phys; phy++) {
        set32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_CLK0REQ);
        if (poll32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_CLK0ACK,
                   APCIE_PHY_CTRL_CLK0ACK, 50000)) {
            printf("pcie: Timeout enabling PHY CLK0\n");
            return -1;
        }

        set32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_CLK1REQ);
        if (poll32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_CLK1ACK,
                   APCIE_PHY_CTRL_CLK1ACK, 50000)) {
            printf("pcie: Timeout enabling PHY CLK1\n");
            return -1;
        }

        clear32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_RESET);
        udelay(1);

        /* ??? */
        if (state->pcie_regs->type == APCIE_T81XX) {
            set32(state->rc_base + APCIE_PHYIF_CTRL, APCIE_PHYIF_CTRL_RUN);
            udelay(1);
        } else if (state->pcie_regs->type == APCIE_T602X) {
            set32(state->phy_base[phy] + 4, 0x01);
        }

        /* Apply "fuses". */
        for (int i = 0; fuse_bits && fuse_bits[i].width; i++) {
            u32 fuse;
            fuse = (read32(state->fuse_base + fuse_bits[i].src_reg) >> fuse_bits[i].src_bit);
            fuse &= (1 << fuse_bits[i].width) - 1;
            mask32(state->phy_ip_base[phy] + fuse_bits[i].tgt_reg,
                   ((1 << fuse_bits[i].width) - 1) << fuse_bits[i].tgt_bit,
                   fuse << fuse_bits[i].tgt_bit);
        }

        char pll_prop[64];
        char auspma_prop[64];

        if (state->num_phys == 1) {
            strcpy(pll_prop, "apcie-phy-ip-pll-tunables");
            strcpy(auspma_prop, "apcie-phy-ip-auspma-tunables");
        } else {
            snprintf(pll_prop, sizeof(pll_prop), "apcie-phy-%d-ip-pll-tunables", phy);
            snprintf(auspma_prop, sizeof(auspma_prop), "apcie-phy-%d-ip-auspma-tunables", phy);
        }

        if (tunables_apply_local_addr(path, pll_prop, state->phy_ip_base[phy])) {
            printf("pcie: Error applying %s for %s\n", pll_prop, path);
            return -1;
        }
        if (tunables_apply_local_addr(path, auspma_prop, state->phy_ip_base[phy])) {
            printf("pcie: Error applying %s for %s\n", auspma_prop, path);
            return -1;
        }

        if (state->pcie_regs->type == APCIE_T602X) {
            set32(state->phy_base[phy] + 4, 0x10);
        }
    }

    if (state->pcie_regs->type == APCIE_T602X) {
        mask32(state->phy_common_base + APCIE_PHYCMN_CLK, APCIE_PHYCMN_CLK_MODE,
               FIELD_PREP(APCIE_PHYCMN_CLK_MODE, 1));

        // Why always PHY 1 in this case?
        u32 off = state->num_phys > 1 ? PHY_STRIDE : 0;
        if (poll32(state->phy_base[0] + off + 0x8, 1, 1, 250000)) {
            printf("pcie: PHY clock enable timed out\n");
            return -1;
        }
        for (int phy = 0; phy < state->num_phys; phy++) {
            set32(state->phy_base[phy] + APCIE_PHY_CTRL, 0x300);
        }
        write32(state->rc_base + 0x54, 0x140);
        write32(state->rc_base + 0x50, 0x1);
        if (poll32(state->rc_base + 0x58, 1, 1, 250000)) {
            printf("pcie: Failed to initialize RC thing\n");
            return -1;
        }
        if (controller == APCIE)
            clear32(state->rc_base + 0x3c, 0x1);
        pmgr_adt_power_disable_index(path, 1);
    }

    for (u32 port = 0; port < state->port_count; port++) {
        char bridge[64];
        int bridge_offset;
        u64 config_base;

        /*
         * Initialize RC port.
         */

        switch (controller) {
            case APCIE:
                snprintf(bridge, sizeof(bridge), "/arm-io/apcie/pci-bridge%d", port);
                break;
            case APCIE_GE0:
                strcpy(bridge, "/arm-io/apcie-ge0/pci-ge0-bridge");
                break;
            case APCIE_GE1:
                strcpy(bridge, "/arm-io/apcie-ge1/pci-ge1-bridge");
                break;
        }

        if ((bridge_offset = adt_path_offset(adt, bridge)) < 0)
            continue;
        if ((allowed_port_mask & BIT(port)) == 0) {
            printf("pcie: Leaving port %d disabled by profile\n", port);
            continue;
        }

        config_base = controller_config_base + ((u64)port << 15);

        printf("pcie: Initializing port %d\n", port);

        if (adt_get_reg(adt, adt_path, "reg",
                        port * port_reg_cnt + state->pcie_regs->shared_reg_count,
                        &state->port_base[port], NULL)) {
            printf("pcie: Error getting reg with index %d for %s\n",
                   port * port_reg_cnt + state->pcie_regs->shared_reg_count, path);
            return -1;
        }

        if (adt_get_reg(adt, adt_path, "reg",
                        port * port_reg_cnt + state->pcie_regs->shared_reg_count + 1,
                        &state->port_ltssm_base[port], NULL)) {
            printf("pcie: Error getting reg with index %d for %s\n",
                   port * port_reg_cnt + state->pcie_regs->shared_reg_count + 1, path);
            return -1;
        }

        if (adt_get_reg(adt, adt_path, "reg",
                        port * port_reg_cnt + state->pcie_regs->shared_reg_count + 2,
                        &state->port_phy_base[port], NULL)) {
            printf("pcie: Error getting reg with index %d for %s\n",
                   port * port_reg_cnt + state->pcie_regs->shared_reg_count + 2, path);
            return -1;
        }

        if (port_reg_cnt >= 5) {
            if (adt_get_reg(adt, adt_path, "reg",
                            port * port_reg_cnt + state->pcie_regs->shared_reg_count + 4,
                            &state->port_intr2axi_base[port], NULL)) {
                printf("pcie: Error getting reg with index %d for %s\n",
                       port * port_reg_cnt + state->pcie_regs->shared_reg_count + 4, path);
                return -1;
            }
        } else {
            state->port_intr2axi_base[port] = 0;
        }

        /*
         * Everything below reprograms the port, so sample the link state first.
         * A port that is already trained must not be reset out from under
         * whoever owns it (apple_pcie_setup_port() guards its bring-up the same
         * way), so this decides whether PERST# may be toggled at all.
         */
        struct pcie_port_bringup bringup = {
            .port_base = state->port_base[port],
            .port_phy_base = state->pcie_regs->type == APCIE_T602X ? state->port_phy_base[port] : 0,
            .perst_reg =
                state->pcie_regs->type == APCIE_T602X ? APCIE_T602X_PORT_RESET : APCIE_PORT_RESET,
            .phy_ack_timeout_us = 50000,
            .link_up_timeout_us = PCIE_PERST_LINK_UP_TIMEOUT_US,
            .link_was_up =
                (read32(state->port_base[port] + APCIE_PORT_LINKSTS) & APCIE_PORT_LINKSTS_UP) != 0,
        };
        struct apple_gpio_pin perst_gpio = {0};

        /*
         * PERST# is a GPIO on every Apple platform that declares it, and m1n1
         * historically never drove it -- only the internal PERST register.  On
         * J414s both ports leave the pad asserted (DATA=0, active low) out of
         * iBoot, so the endpoint is held in reset and the link can never train.
         *
         * Resolution is entirely ADT-driven (function-perst -> phandle + pin),
         * so nothing here is machine specific.  A port with no function-perst,
         * an SMC-backed rail, or an unresolvable phandle simply keeps the old
         * behaviour: the pad is left exactly as firmware set it.
         */
        if (apple_gpio_resolve_function(bridge_offset, "perst", &perst_gpio) == 0) {
            bringup.have_perst_gpio = true;
            state->port_perst_gpio[port] = perst_gpio;
            printf("pcie: Port %d PERST# is GPIO pin %u at %#lx\n", port, perst_gpio.pin,
                   perst_gpio.base);
        } else {
            printf("pcie: Port %d has no resolvable PERST# GPIO; leaving the pad alone\n", port);
        }

        u32 t_refclk_to_perst = 0, perst_to_config = 0;
        bool have_refclk_to_perst =
            ADT_GETPROP(adt, bridge_offset, "t-refclk-to-perst", &t_refclk_to_perst) >= 0;
        bool have_perst_to_config =
            ADT_GETPROP(adt, bridge_offset, "perst-to-config", &perst_to_config) >= 0;

        pcie_perst_delays_from_adt(&bringup.delays, have_refclk_to_perst, t_refclk_to_perst,
                                   have_perst_to_config, perst_to_config);

        /*
         * `manual-enable` marks a port that is under explicit software
         * enable/disable control rather than being treated as always-on.  It
         * is NOT a "skip this port" marker and it says nothing about whether
         * firmware already brought the port up.
         *
         * Determined from AppleEmbeddedPCIE.kext: AppleEmbeddedPCIEPort::
         * autoEnable() enables the port either way.  Without manual-enable it
         * enables with flags 0x8 and returns true, so AppleEmbeddedPCIE::
         * configure() adds the port to its wait-for-link-up mask; with
         * manual-enable it enables with flags 0x8|0x2 and returns false, so
         * macOS scans the port but does not block on its link.  The only
         * property that actually defers a scan is the separate
         * `manual-enable-defer-scan`, which no published Apple Silicon ADT
         * carries.  Both J414s ports declare manual-enable, and both must be
         * brought up -- skipping either is what would leave WiFi/BT and the SD
         * card reader dead.
         *
         * m1n1's job is to bring hardware up, so the property changes no
         * behaviour here; it is logged so the ownership is explicit, and
         * `manual-enable-s2r` is noted only because it means PERST# for this
         * port is re-driven across suspend/resume (IOPCIFamily
         * IOPCIBridge.cpp), which is not something m1n1 participates in.
         */
        if (adt_getprop(adt, bridge_offset, "manual-enable", NULL))
            printf("pcie: Port %d is manual-enable; m1n1 owns its bring-up\n", port);

        if (bringup.link_was_up)
            printf("pcie: Port %d link is already up; not resetting it\n", port);

        printf("pcie: Port %d PERST# timing: refclk->perst %uus, perst->config %uus\n", port,
               bringup.delays.refclk_to_perst_us, bringup.delays.perst_to_config_us);

        if (state->pcie_regs->type == APCIE_T602X) {
            set32(state->rc_base + 0x3c, 0x1);

            // ??????
            if (controller == APCIE)
                write32(state->port_base[port] + 0x10, 0x2);
            write32(state->port_base[port] + 0x88, 0x110);
            write32(state->port_base[port] + 0x100, 0xffffffff);
            write32(state->port_base[port] + 0x148, 0xffffffff);
            write32(state->port_base[port] + 0x210, 0xffffffff);
            write32(state->port_base[port] + 0x80, 0x0);
            write32(state->port_base[port] + 0x84, 0x0);
            write32(state->port_base[port] + 0x104, 0x7fffffff);
            write32(state->port_base[port] + 0x124, 0x100);
            write32(state->port_base[port] + 0x16c, 0x0);
            write32(state->port_base[port] + 0x13c, 0x10);
            write32(state->port_base[port] + 0x800, 0x100100);
            write32(state->port_base[port] + 0x808, 0x1000ff);
            write32(state->port_base[port] + 0x82c, 0x0);
            for (int i = 0; i < 512; i++)
                write32(state->port_base[port] + PCIE_T602X_PORT_MSIMAP_OFFSET + 4 * i, 0);
            write32(state->port_base[port] + 0x397c, 0x0);
            if (controller == APCIE)
                write32(state->port_base[port] + 0x130, 0x3000000);
            else
                write32(state->port_base[port] + 0x130, 0x3000008);
            write32(state->port_base[port] + 0x140, 0x10);
            write32(state->port_base[port] + 0x144, 0x253770);
            write32(state->port_base[port] + 0x21c, 0x0);
            write32(state->port_base[port] + 0x834, 0x0);
            if (controller != APCIE)
                write32(state->port_base[port] + 0x83c, 0x0);
        }

        if (tunables_apply_local_addr(bridge, "apcie-config-tunables", state->port_base[port])) {
            printf("pcie: Error applying %s for %s\n", "apcie-config-tunables", bridge);
            return -1;
        }

        /*
         * Power-enable rails.  PERST# alone is not enough: on J414s both ports
         * drove PERST# correctly and still reported LINKSTS 0xab000208 forever,
         * because a device with no power rail is not a link partner at all.
         *
         * The rail is declared on the ENDPOINT child, not on the bridge -- on
         * J414s /arm-io/apcie/pci-bridge1/pcie-sdreader carries
         * function-sd_pwr_en (SMC key "gP16" == 0x67503136, verified live
         * against the ADT), while pci-bridge0's wlan/bluetooth-pcie children
         * carry no such property and need nothing here.  So walk the children
         * rather than looking at the bridge.
         *
         * Ordering follows pcie-apple.c: the rail goes high while PERST# is
         * still asserted, then Tpvperl settles before PERST# is released.  We
         * assert here, ahead of pcie_port_release_perst(), so the device is
         * powered and stable for the whole reset window.
         *
         * Best-effort by design: a missing property, an unresolvable key, or a
         * dead SMC logs and continues.  A port whose link is already up is left
         * alone entirely -- never power-cycle a working device.
         */
        /*
         * Raise this port's power rail while PERST# is still asserted, then let
         * Tpvperl settle before it is released.  A port whose link is already
         * up is left alone -- never power-cycle a working device.
         */
        if (!bringup.link_was_up && pcie_enable_port_rail(bridge_offset, port))
            udelay(PCIE_PWREN_TO_PERST_US);

        /*
         * APPCLK on, PERST# asserted before the clocks, refclk request/ack,
         * Tperst-clk, then PERST# released -- apple_pcie_setup_link() order.
         * The register pokes are byte-for-byte the ones m1n1 already issued
         * here; the PERST# pad handling and the ADT-derived delay are new.
         */
        int bringup_ret = pcie_port_release_perst(&pcie_hw_bringup_ops, &perst_gpio, &bringup);
        if (bringup_ret) {
            printf("pcie: Port %d PERST# release failed (%d) on %s\n", port, bringup_ret, bridge);
            return -1;
        }

        if (poll32(state->port_base[port] + APCIE_PORT_STATUS, APCIE_PORT_STATUS_RUN,
                   APCIE_PORT_STATUS_RUN, 250000)) {
            printf("pcie: Port failed to come up on %s\n", bridge);
            return -1;
        }

        if (state->pcie_regs->type == APCIE_T602X && controller != APCIE) {
            write32(state->port_ltssm_base[port] + 0x10, 0x2);
            write32(state->port_ltssm_base[port] + 0x1c, 0x4);
            set32(state->port_ltssm_base[port] + 0x20, 0x2);
            write32(state->port_ltssm_base[port] + 0x14, 0x1);

            clear32(state->port_base[port] + APCIE_PORT_APPCLK, 0x100);
        }

        if (poll32(state->port_base[port] + APCIE_PORT_LINKSTS, APCIE_PORT_LINKSTS_BUSY, 0,
                   250000)) {
            printf("pcie: Port failed to become idle on %s\n", bridge);
            return -1;
        }

        /* Do it again? */
        if (state->pcie_regs->type == APCIE_T602X && controller == APCIE) {
            clear32(state->port_base[port] + APCIE_T602X_PORT_RESET, APCIE_PORT_RESET_DIS);
            set32(state->port_base[port] + APCIE_T602X_PORT_RESET, APCIE_PORT_RESET_DIS);

            if (poll32(state->port_base[port] + APCIE_PORT_LINKSTS, APCIE_PORT_LINKSTS_BUSY, 0,
                       250000)) {
                printf("pcie: Port failed to become idle (2) on %s\n", bridge);
                return -1;
            }

            udelay(1000);

            write32(state->port_ltssm_base[port] + 0x10, 0x2);
            write32(state->port_ltssm_base[port] + 0x1c, 0x4);
            set32(state->port_ltssm_base[port] + 0x20, 0x2);
            write32(state->port_ltssm_base[port] + 0x14, 0x1);
        }

        /*
         * perst-to-config: PCIe Base r5.0 6.6.1 requires 100ms between the last
         * PERST# deassertion and the first configuration request.  The block
         * above re-cycles the internal PERST register (m1n1 has always done
         * that, Asahi does not), so the wait is taken here, after the final
         * PERST transition and before the first config-space access below.  The
         * endpoint's own PERST# pad was released earlier, so it gets at least
         * this much settling time too.
         */
        udelay(bringup.delays.perst_to_config_us);

        /* Make Designware PCIe Core registers writable. */
        set32(config_base + DWC_DBI_RO_WR, DWC_DBI_RO_WR_EN);

        if (tunables_apply_local_addr(bridge, "pcie-rc-tunables", config_base)) {
            printf("pcie: Error applying %s for %s\n", "pcie-rc-tunables", bridge);
            return -1;
        }
        if (tunables_apply_local_addr(bridge, "pcie-rc-gen3-shadow-tunables", config_base)) {
            printf("pcie: Error applying %s for %s\n", "pcie-rc-gen3-shadow-tunables", bridge);
            return -1;
        }
        if (tunables_apply_local_addr(bridge, "pcie-rc-gen4-shadow-tunables", config_base)) {
            printf("pcie: Error applying %s for %s\n", "pcie-rc-gen4-shadow-tunables", bridge);
            return -1;
        }

        u32 max_speed;
        if (ADT_GETPROP(adt, bridge_offset, "maximum-link-speed", &max_speed) >= 0) {
            /* Some devices override "maximum-link-speed" in the device child nodes.
             * The property used for the link speed seems to be ad-hoc made up.
             * The 10 GB ethernet adapter uses "target-link-speed" and the SD card
             * reader uses "expected-link-speed". Assume that PCIe link speed override
             * resides in the first (only?) child node.
             */
            if (max_speed == 1) {
                int np = adt_first_child_offset(adt, bridge_offset);
                if (np >= 0) {
                    int target_speed;
                    if (ADT_GETPROP(adt, np, "target-link-speed", &target_speed) >= 0 &&
                        target_speed > 0) {
                        max_speed = target_speed;
                    } else if (ADT_GETPROP(adt, np, "expected-link-speed", &target_speed) >= 0 &&
                               target_speed > 0) {
                        max_speed = target_speed;
                    }
                }
            }

            printf("pcie: Port %d max speed = %d\n", port, max_speed);

            if (max_speed == 0) {
                printf("pcie: Invalid max-speed\n");
                return -1;
            }

            mask32(config_base + PCIE_CAP_BASE + PCIE_LNKCAP, PCIE_LNKCAP_SLS,
                   FIELD_PREP(PCIE_LNKCAP_SLS, max_speed));

            mask32(config_base + PCIE_CAP_BASE + PCIE_LNKCAP2, PCIE_LNKCAP2_SLS,
                   FIELD_PREP(PCIE_LNKCAP2_SLS, (1 << max_speed) - 1));

            mask16(config_base + PCIE_CAP_BASE + PCIE_LNKCTL2, PCIE_LNKCTL2_TLS,
                   FIELD_PREP(PCIE_LNKCTL2_TLS, max_speed));

            set32(config_base + DWC_DBI_LINK_WIDTH_SPEED_CONTROL, DWC_DBI_SPEED_CHANGE);
        }

        /* Max link width */
        mask32(config_base + DWC_DBI_PORT_LINK_CONTROL, DWC_DBI_PORT_LINK_MODE,
               FIELD_PREP(DWC_DBI_PORT_LINK_MODE, lane_mode));
        mask32(config_base + DWC_DBI_LINK_WIDTH_SPEED_CONTROL, DWC_DBI_LINK_WIDTH,
               FIELD_PREP(DWC_DBI_LINK_WIDTH, link_width));
        mask32(config_base + PCIE_CAP_BASE + PCIE_LNKCAP, PCIE_LNKCAP_MLW,
               FIELD_PREP(PCIE_LNKCAP_MLW, link_width));

        /* Make Designware PCIe Core registers readonly. */
        clear32(config_base + DWC_DBI_RO_WR, DWC_DBI_RO_WR_EN);

        if (state->pcie_regs->type == APCIE_T602X) {
            write32(state->port_base[port] + 0x4020, 0x3);
            if (state->port_intr2axi_base[port])
                write32(state->port_intr2axi_base[port] + 0x80, 0x1);

            clear32(state->rc_base + 0x3c, 0x1);
            for (int i = 0; i < 32; i++)
                write32(state->port_base[port] + PCIE_T602X_PORT_MSIMAP_OFFSET + 4 * i,
                        PCIE_T602X_MSIMAP_VALID | i);
        }

        /*
         * Enable refclk clock gating, start LTSSM and poll for the link with a
         * bounded timeout -- the tail of apple_pcie_setup_port().  m1n1 used to
         * read LINKSTS exactly once here, which can only ever observe a link
         * that some earlier stage had already trained.
         */
        int link_ret = pcie_port_start_link(&pcie_hw_bringup_ops, &perst_gpio, &bringup);
        u32 link_status = read32(state->port_base[port] + APCIE_PORT_LINKSTS);

        if (link_ret) {
            printf("pcie: Port %d link did not come up within %uus (status %#x)\n", port,
                   bringup.link_up_timeout_us, link_status);
            if (require_link_up)
                return -1;
        } else {
            printf("pcie: Port %d link up (status %#x)\n", port, link_status);
            /*
             * Only once the link is actually up: a bus number range on a port
             * with no link partner advertises a hierarchy that does not exist.
             */
            pcie_program_bridge_bus_numbers(config_base, port);
        }

        state->initialized_port_mask |= BIT(port);
    }

    printf("pcie: Initialized controller %d\n", controller);
    state->initialized = true;

    return 0;
}

int pcie_init(void)
{
    bool success = false;

    if (pcie_initialized)
        return 0;

    success |= pcie_init_controller(APCIE, "/arm-io/apcie", UINT32_MAX, false) == 0;
    success |= pcie_init_controller(APCIE_GE0, "/arm-io/apcie-ge0", UINT32_MAX, false) == 0;
    success |= pcie_init_controller(APCIE_GE1, "/arm-io/apcie-ge1", UINT32_MAX, false) == 0;

    if (success)
        pcie_initialized = true;

    return success ? 0 : -1;
}

int pcie_init_wireless(void)
{
    if (!platform_is_j414s()) {
        printf("pcie: wireless profile rejected non-J414s identity\n");
        return -1;
    }
    if (pcie_initialized)
        return -1;

    /*
     * The wireless/SD bring-up path is the one caller that needs the port
     * power rails AND routable bus numbers behind the root ports.  See
     * pcie_wireless_profile: the generic pcie_init() path used by non-wireless
     * Windows boots leaves both alone, because these devices' MSI delivery is
     * not activated yet and powering them destabilises xHCI.
     *
     * Both ports: WiFi/BT is on pci-bridge0 and the SD reader on pci-bridge1,
     * so BIT(0) alone would leave SD dark.  require_link_up stays false --
     * port 1 having no link must not fail the wireless profile, and vice
     * versa.
     */
    pcie_wireless_profile = true;

    if (pcie_init_controller(APCIE, "/arm-io/apcie", BIT(0) | BIT(1), false)) {
        pcie_wireless_profile = false;
        return -1;
    }
    pcie_initialized = true;
    return 0;
}

int pcie_shutdown(void)
{
    if (!pcie_initialized)
        return 0;

    for (u32 controller = 0; controller < NUM_CONTROLLERS; controller++) {
        struct state *state = &controllers[controller];

        if (!state->initialized)
            continue;

        for (u32 port = 0; port < state->port_count; port++) {
            if ((state->initialized_port_mask & BIT(port)) == 0)
                continue;
            /*
             * Put the endpoint back in reset before its clocks go away, so it
             * is never left running unclocked.  Only pads m1n1 itself resolved
             * and drove are touched.  The argument is the pad level, and the
             * pad is active low, so driving it low asserts PERST#.
             */
            if (state->port_perst_gpio[port].valid)
                apple_gpio_set_output(&state->port_perst_gpio[port], false);
            if (state->pcie_regs->type == APCIE_T602X)
                clear32(state->port_base[port] + APCIE_T602X_PORT_RESET, APCIE_PORT_RESET_DIS);
            else
                clear32(state->port_base[port] + APCIE_PORT_RESET, APCIE_PORT_RESET_DIS);
            clear32(state->port_base[port] + APCIE_PORT_APPCLK, APCIE_PORT_APPCLK_EN);
        }

        for (int phy = 0; phy < state->num_phys; phy++) {
            clear32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_RESET);
            clear32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_CLK1REQ);
            clear32(state->phy_base[phy] + APCIE_PHY_CTRL, APCIE_PHY_CTRL_CLK0REQ);
        }

        state->initialized = false;
        state->initialized_port_mask = 0;
    }

    pcie_initialized = false;
    printf("pcie: Shutdown.\n");

    return 0;
}

#endif /* !PCIE_T602X_WIRELESS_HOST_TEST */
