/* SPDX-License-Identifier: MIT */

#include "platform_identity.h"
#include "soc.h"
#include "string.h"

#ifndef PLATFORM_IDENTITY_HOST_TEST
#include "adt.h"
#include "utils.h"
#endif

static const char j414s_root_target_type[] = "J414s";
static const char j414s_model[] = "Mac14,9";
static const char j414s_compatible[] = "J414sAP\0Mac14,9\0AppleARM";

static bool property_equals(const void *property, u32 property_len, const void *expected,
                            size_t expected_len)
{
    return property && property_len == expected_len && !memcmp(property, expected, expected_len);
}

bool platform_identity_matches_j414s(const struct platform_identity *identity)
{
    if (!identity)
        return false;

    /* The measured J414s /chosen node has no target-type payload. */
    if (identity->chosen_target_type_len != 0)
        return false;

    return identity->chip_id == T6020 && identity->board_id == 4 &&
           property_equals(identity->root_target_type, identity->root_target_type_len,
                           j414s_root_target_type, sizeof(j414s_root_target_type)) &&
           property_equals(identity->model, identity->model_len, j414s_model,
                           sizeof(j414s_model)) &&
           property_equals(identity->compatible, identity->compatible_len, j414s_compatible,
                           sizeof(j414s_compatible));
}

#ifndef PLATFORM_IDENTITY_HOST_TEST
bool platform_is_j414s(void)
{
    int chosen = adt_path_offset(adt, "/chosen");
    struct platform_identity identity = {
        .chip_id = chip_id,
        .board_id = board_id,
    };

    if (chosen < 0)
        return false;

    identity.chosen_target_type =
        adt_getprop(adt, chosen, "target-type", &identity.chosen_target_type_len);
    identity.root_target_type = adt_getprop(adt, 0, "target-type", &identity.root_target_type_len);
    identity.model = adt_getprop(adt, 0, "model", &identity.model_len);
    identity.compatible = adt_getprop(adt, 0, "compatible", &identity.compatible_len);

    return platform_identity_matches_j414s(&identity);
}
#endif
