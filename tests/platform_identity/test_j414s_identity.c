/* SPDX-License-Identifier: MIT */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform_identity.h"

static const char root_target_type[] = "J414s";
static const char model[] = "Mac14,9";
static const char compatible[] = "J414sAP\0Mac14,9\0AppleARM";

static void check(bool condition, const char *expression, int line)
{
    if (condition)
        return;

    fprintf(stderr, "line %d: check failed: %s\n", line, expression);
    exit(1);
}

#define CHECK(expression) check((expression), #expression, __LINE__)

static struct platform_identity valid_identity(void)
{
    return (struct platform_identity){
        .chip_id = 0x6020,
        .board_id = 4,
        .chosen_target_type = NULL,
        .chosen_target_type_len = 0,
        .root_target_type = root_target_type,
        .root_target_type_len = sizeof(root_target_type),
        .model = model,
        .model_len = sizeof(model),
        .compatible = compatible,
        .compatible_len = sizeof(compatible),
    };
}

static void test_exact_live_identity(void)
{
    struct platform_identity identity = valid_identity();
    static const char unused[] = "not inspected when length is zero";

    CHECK(platform_identity_matches_j414s(&identity));

    /* Both an absent property and a present zero-length property are valid. */
    identity.chosen_target_type = unused;
    CHECK(platform_identity_matches_j414s(&identity));
}

static void test_rejects_chosen_target_type(void)
{
    struct platform_identity identity = valid_identity();
    static const char synthetic_target[] = "J414s";
    static const char empty_string[] = "";

    identity.chosen_target_type = synthetic_target;
    identity.chosen_target_type_len = sizeof(synthetic_target);
    CHECK(!platform_identity_matches_j414s(&identity));

    /* The measured contract is absent/zero-length, not a one-byte C string. */
    identity.chosen_target_type = empty_string;
    identity.chosen_target_type_len = sizeof(empty_string);
    CHECK(!platform_identity_matches_j414s(&identity));
}

static void test_rejects_chip_and_board_mutations(void)
{
    struct platform_identity identity = valid_identity();

    identity.chip_id = 0x6021;
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.board_id = 5;
    CHECK(!platform_identity_matches_j414s(&identity));
}

static void test_rejects_root_target_mutations(void)
{
    struct platform_identity identity = valid_identity();
    static const char wrong_target[] = "J414c";

    identity.root_target_type = NULL;
    identity.root_target_type_len = 0;
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.root_target_type = wrong_target;
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.root_target_type_len--;
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.root_target_type_len++;
    CHECK(!platform_identity_matches_j414s(&identity));
}

static void test_rejects_model_mutations(void)
{
    struct platform_identity identity = valid_identity();
    static const char wrong_model[] = "Mac14,10";

    identity.model = NULL;
    identity.model_len = 0;
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.model = wrong_model;
    identity.model_len = sizeof(wrong_model);
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.model_len--;
    CHECK(!platform_identity_matches_j414s(&identity));
}

static void test_rejects_compatible_mutations(void)
{
    struct platform_identity identity = valid_identity();
    static const char synthetic[] = "apple,j414s\0apple,t6020";
    static const char reordered[] = "J414sAP\0AppleARM\0Mac14,9";
    static const char extended[] = "J414sAP\0Mac14,9\0AppleARM\0extra";
    char changed[sizeof(compatible)];

    identity.compatible = NULL;
    identity.compatible_len = 0;
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.compatible = synthetic;
    identity.compatible_len = sizeof(synthetic);
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.compatible = reordered;
    identity.compatible_len = sizeof(reordered);
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.compatible = extended;
    identity.compatible_len = sizeof(extended);
    CHECK(!platform_identity_matches_j414s(&identity));

    memcpy(changed, compatible, sizeof(changed));
    changed[0] = 'X';
    identity = valid_identity();
    identity.compatible = changed;
    CHECK(!platform_identity_matches_j414s(&identity));

    identity = valid_identity();
    identity.compatible_len--;
    CHECK(!platform_identity_matches_j414s(&identity));
}

int main(void)
{
    CHECK(!platform_identity_matches_j414s(NULL));
    test_exact_live_identity();
    test_rejects_chosen_target_type();
    test_rejects_chip_and_board_mutations();
    test_rejects_root_target_mutations();
    test_rejects_model_mutations();
    test_rejects_compatible_mutations();
    return 0;
}
