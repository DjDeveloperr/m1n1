/* SPDX-License-Identifier: MIT */

#ifndef TPS6598X_HOST_POLICY_H
#define TPS6598X_HOST_POLICY_H

#ifdef TPS6598X_HOST_POLICY_HOST_TEST
#include <stdint.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef int32_t s32;
#else
#include "types.h"
#endif

#define TPS6598X_SYSTEM_CONFIG_LEN 17

enum tps6598x_host_policy_result {
    TPS6598X_HOST_POLICY_SKIP = 0,
    TPS6598X_HOST_POLICY_READY = 1,
    TPS6598X_HOST_POLICY_UPDATED = 2,
    TPS6598X_HOST_POLICY_ERR_ARGUMENT = -1,
    TPS6598X_HOST_POLICY_ERR_PORT = -2,
    TPS6598X_HOST_POLICY_ERR_ROLE = -3,
    TPS6598X_HOST_POLICY_ERR_READBACK = -4,
};

int tps6598x_host_port_resolve(u32 rid, u32 port_number, const char *port_location,
                               u32 port_location_size, u32 controller_count,
                               u32 *controller_index);

int tps6598x_host_policy_prepare(u32 hpm_index, u32 controller_count, s32 preserved_index,
                                 const u8 current[TPS6598X_SYSTEM_CONFIG_LEN],
                                 u8 desired[TPS6598X_SYSTEM_CONFIG_LEN]);
int tps6598x_host_policy_verify(const u8 expected[TPS6598X_SYSTEM_CONFIG_LEN],
                                const u8 readback[TPS6598X_SYSTEM_CONFIG_LEN]);

#endif
