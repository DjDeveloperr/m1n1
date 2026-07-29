/* SPDX-License-Identifier: MIT */

#ifdef WIRELESS_HANDOFF_ABI_HOST_TEST
#include <string.h>
#else
#include "string.h"
#endif
#include "wireless_handoff_abi.h"

u32 wireless_handoff_v2_crc32(const void *data, u32 length)
{
    const u8 *bytes = data;
    u32 crc = 0xffffffffU;

    for (u32 index = 0; index < length; index++) {
        crc ^= bytes[index];
        for (u32 bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

int wireless_handoff_v2_descriptor_validate(
    const struct wireless_handoff_descriptor_v2 *descriptor,
    const void *reservation, u64 expected_base, u64 expected_size,
    u64 expected_guest_top)
{
    struct wireless_handoff_descriptor_v2 copy;
    u32 descriptor_crc;

    if (!descriptor || !reservation ||
        expected_size != WIRELESS_HANDOFF_V2_RESERVATION_SIZE ||
        (expected_base & (WIRELESS_HANDOFF_V2_PAGE_SIZE - 1)) ||
        expected_base > ~0ULL - expected_size)
        return -1;
    if (descriptor->signature != WIRELESS_HANDOFF_V2_SIGNATURE ||
        descriptor->version != WIRELESS_HANDOFF_V2_VERSION ||
        descriptor->structure_size != sizeof(*descriptor) ||
        descriptor->flags != WIRELESS_HANDOFF_V2_FLAG_INSTALLED ||
        descriptor->sid != WIRELESS_HANDOFF_V2_SID ||
        descriptor->page_shift != WIRELESS_HANDOFF_V2_PAGE_SHIFT ||
        descriptor->reserved != 0 ||
        descriptor->reservation_base != expected_base ||
        descriptor->reservation_size != expected_size ||
        descriptor->guest_memory_top != expected_guest_top ||
        descriptor->physical_memory_top < expected_base + expected_size ||
        descriptor->dart_base != WIRELESS_HANDOFF_V2_DART_BASE ||
        descriptor->l1_physical != expected_base + WIRELESS_HANDOFF_V2_L1_OFFSET ||
        descriptor->msi_l2_physical != expected_base + WIRELESS_HANDOFF_V2_MSI_L2_OFFSET ||
        descriptor->descriptor_physical !=
            expected_base + WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET)
        return -2;

    memcpy(&copy, descriptor, sizeof(copy));
    descriptor_crc = copy.descriptor_crc32;
    copy.descriptor_crc32 = 0;
    if (!descriptor_crc ||
        wireless_handoff_v2_crc32(&copy, sizeof(copy)) != descriptor_crc ||
        wireless_handoff_v2_crc32(
            (const u8 *)reservation + WIRELESS_HANDOFF_V2_L1_OFFSET,
            WIRELESS_HANDOFF_V2_PAGE_SIZE) != descriptor->l1_crc32 ||
        wireless_handoff_v2_crc32(
            (const u8 *)reservation + WIRELESS_HANDOFF_V2_MSI_L2_OFFSET,
            WIRELESS_HANDOFF_V2_PAGE_SIZE) != descriptor->msi_l2_crc32)
        return -3;
    return 0;
}
