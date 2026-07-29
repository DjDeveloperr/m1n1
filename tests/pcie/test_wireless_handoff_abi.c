/* SPDX-License-Identifier: MIT */

#include "wireless_handoff_abi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned char reservation[WIRELESS_HANDOFF_V2_RESERVATION_SIZE]
    __attribute__((aligned(WIRELESS_HANDOFF_V2_PAGE_SIZE)));

static void check(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "wireless-handoff-abi: FAIL: %s\n", message);
        exit(1);
    }
}

static void build(struct wireless_handoff_descriptor_v2 *descriptor, u64 base,
                  u64 guest_top)
{
    memset(reservation, 0, sizeof(reservation));
    reservation[3] = 0x41;
    reservation[WIRELESS_HANDOFF_V2_MSI_L2_OFFSET + 9] = 0x82;
    memset(descriptor, 0, sizeof(*descriptor));
    descriptor->signature = WIRELESS_HANDOFF_V2_SIGNATURE;
    descriptor->version = WIRELESS_HANDOFF_V2_VERSION;
    descriptor->structure_size = sizeof(*descriptor);
    descriptor->flags = WIRELESS_HANDOFF_V2_FLAG_INSTALLED;
    descriptor->sid = WIRELESS_HANDOFF_V2_SID;
    descriptor->page_shift = WIRELESS_HANDOFF_V2_PAGE_SHIFT;
    descriptor->reservation_base = base;
    descriptor->reservation_size = sizeof(reservation);
    descriptor->guest_memory_top = guest_top;
    descriptor->physical_memory_top = base + sizeof(reservation) + 0x4000;
    descriptor->dart_base = WIRELESS_HANDOFF_V2_DART_BASE;
    descriptor->l1_physical = base + WIRELESS_HANDOFF_V2_L1_OFFSET;
    descriptor->msi_l2_physical = base + WIRELESS_HANDOFF_V2_MSI_L2_OFFSET;
    descriptor->descriptor_physical =
        base + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET;
    descriptor->l1_crc32 = wireless_handoff_v2_crc32(
        reservation + WIRELESS_HANDOFF_V2_L1_OFFSET,
        WIRELESS_HANDOFF_V2_PAGE_SIZE);
    descriptor->msi_l2_crc32 = wireless_handoff_v2_crc32(
        reservation + WIRELESS_HANDOFF_V2_MSI_L2_OFFSET,
        WIRELESS_HANDOFF_V2_PAGE_SIZE);
    descriptor->descriptor_crc32 = wireless_handoff_v2_crc32(
        descriptor, sizeof(*descriptor));
    memcpy(reservation + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET,
           descriptor, sizeof(*descriptor));
}

int main(void)
{
    const u64 base = 0x10022040000ULL;
    const u64 guest_top = base - WIRELESS_HANDOFF_V2_PAGE_SIZE;
    struct wireless_handoff_descriptor_v2 descriptor;

    build(&descriptor, base, guest_top);
    check(wireless_handoff_v2_descriptor_validate(
              (const void *)(reservation + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET),
              reservation, base, sizeof(reservation), guest_top) == 0,
          "valid descriptor accepted");

    reservation[0] ^= 1;
    check(wireless_handoff_v2_descriptor_validate(
              (const void *)(reservation + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET),
              reservation, base, sizeof(reservation), guest_top) == -3,
          "table corruption rejected");
    build(&descriptor, base, guest_top);
    ((struct wireless_handoff_descriptor_v2 *)(
        reservation + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET))->version++;
    check(wireless_handoff_v2_descriptor_validate(
              (const void *)(reservation + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET),
              reservation, base, sizeof(reservation), guest_top) == -2,
          "version mismatch rejected");
    check(wireless_handoff_v2_descriptor_validate(
              &descriptor, reservation, base + WIRELESS_HANDOFF_V2_PAGE_SIZE,
              sizeof(reservation), guest_top) == -2,
          "base mismatch rejected");
    puts("wireless handoff ABI v2 tests: PASS");
    return 0;
}
