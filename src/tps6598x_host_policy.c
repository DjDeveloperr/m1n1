/* SPDX-License-Identifier: MIT */

#ifdef TPS6598X_HOST_POLICY_HOST_TEST
#include <string.h>
#else
#include "string.h"
#endif
#include "tps6598x_host_policy.h"

#define TPS6598X_PORT_INFO_MASK 0x07

int tps6598x_host_port_resolve(u32 rid, u32 port_number, const char *port_location,
                               u32 port_location_size, u32 controller_count,
                               u32 *controller_index)
{
    if (!port_location || !port_location_size || !controller_index)
        return TPS6598X_HOST_POLICY_ERR_ARGUMENT;
    if (rid >= controller_count || port_number != rid + 1)
        return TPS6598X_HOST_POLICY_ERR_PORT;
    if (port_location[port_location_size - 1] != '\0' || port_location[0] == '\0')
        return TPS6598X_HOST_POLICY_ERR_PORT;

    const char *expected_location;
    switch (rid) {
        case 0:
            expected_location = "left-back";
            break;
        case 1:
            expected_location = "left-front";
            break;
        case 2:
            expected_location = "right";
            break;
        default:
            return TPS6598X_HOST_POLICY_ERR_PORT;
    }
    if (strcmp(port_location, expected_location))
        return TPS6598X_HOST_POLICY_ERR_PORT;

    *controller_index = rid;
    return TPS6598X_HOST_POLICY_READY;
}

/*
 * TPS6598x System Configuration.PortInfo values provide matching dual-role
 * encodings that select the preferred initial role without changing whether
 * PR_Swap or DR_Swap is supported:
 *
 *   010b Sink/UFP, PR_Swap    -> 100b Source/DFP, PR_Swap
 *   011b Sink/UFP, PR+DR Swap -> 101b Source/DFP, PR+DR Swap
 *
 * Sink-only, accessory and disabled configurations are not rewritten: their
 * power-path configuration is not proven capable of safely sourcing VBUS.
 */
int tps6598x_host_policy_prepare(u32 hpm_index, u32 controller_count, s32 preserved_index,
                                 const u8 current[TPS6598X_SYSTEM_CONFIG_LEN],
                                 u8 desired[TPS6598X_SYSTEM_CONFIG_LEN])
{
    if (!current || !desired)
        return TPS6598X_HOST_POLICY_ERR_ARGUMENT;
    if (hpm_index >= controller_count)
        return TPS6598X_HOST_POLICY_ERR_PORT;
    if (preserved_index >= 0 && hpm_index == (u32)preserved_index)
        return TPS6598X_HOST_POLICY_SKIP;

    memcpy(desired, current, TPS6598X_SYSTEM_CONFIG_LEN);

    u8 port_info = current[0] & TPS6598X_PORT_INFO_MASK;
    u8 host_port_info;
    switch (port_info) {
        case 2:
            host_port_info = 4;
            break;
        case 3:
            host_port_info = 5;
            break;
        case 4:
        case 5:
        case 6:
            return TPS6598X_HOST_POLICY_READY;
        default:
            return TPS6598X_HOST_POLICY_ERR_ROLE;
    }

    desired[0] = (current[0] & (u8)~TPS6598X_PORT_INFO_MASK) | host_port_info;
    return TPS6598X_HOST_POLICY_UPDATED;
}

int tps6598x_host_policy_verify(const u8 expected[TPS6598X_SYSTEM_CONFIG_LEN],
                                const u8 readback[TPS6598X_SYSTEM_CONFIG_LEN])
{
    if (!expected || !readback)
        return TPS6598X_HOST_POLICY_ERR_ARGUMENT;
    if (memcmp(expected, readback, TPS6598X_SYSTEM_CONFIG_LEN))
        return TPS6598X_HOST_POLICY_ERR_READBACK;
    return TPS6598X_HOST_POLICY_READY;
}
