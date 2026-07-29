/* SPDX-License-Identifier: MIT */

#ifndef PLATFORM_IDENTITY_H
#define PLATFORM_IDENTITY_H

#ifdef PLATFORM_IDENTITY_HOST_TEST
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t u32;
#else
#include "types.h"
#endif

/*
 * Raw identity properties used by destructive, board-specific handoffs.
 * Lengths include any trailing NUL bytes present in the ADT property.
 */
struct platform_identity {
    u32 chip_id;
    u32 board_id;
    const void *chosen_target_type;
    u32 chosen_target_type_len;
    const void *root_target_type;
    u32 root_target_type_len;
    const void *model;
    u32 model_len;
    const void *compatible;
    u32 compatible_len;
};

bool platform_identity_matches_j414s(const struct platform_identity *identity);
bool platform_is_j414s(void);

#endif
