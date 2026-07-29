/* SPDX-License-Identifier: MIT */

#ifndef WIRELESS_HANDOFF_ABI_H
#define WIRELESS_HANDOFF_ABI_H

#ifdef WIRELESS_HANDOFF_ABI_HOST_TEST
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#ifndef BIT
#define BIT(_bit) (UINT32_C(1) << (_bit))
#endif
#else
#include "types.h"
#endif

#define WIRELESS_HANDOFF_V2_SIGNATURE       0x3248574eU /* NWH2 */
#define WIRELESS_HANDOFF_V2_VERSION         2U
#define WIRELESS_HANDOFF_V2_FLAG_INSTALLED  BIT(0)
#define WIRELESS_HANDOFF_V2_RESERVATION_SIZE 0x10000ULL
#define WIRELESS_HANDOFF_V2_PAGE_SIZE       0x4000ULL
#define WIRELESS_HANDOFF_V2_L1_OFFSET       0x0000ULL
#define WIRELESS_HANDOFF_V2_MSI_L2_OFFSET   0x4000ULL
#define WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET 0xc000ULL
#define WIRELESS_HANDOFF_V2_DART_BASE       0x594000000ULL
#define WIRELESS_HANDOFF_V2_SID             1U
#define WIRELESS_HANDOFF_V2_PAGE_SHIFT      14U

struct wireless_handoff_descriptor_v2 {
    u32 signature;
    u16 version;
    u16 structure_size;
    u32 flags;
    u16 sid;
    u16 page_shift;
    u64 reservation_base;
    u64 reservation_size;
    u64 guest_memory_top;
    u64 physical_memory_top;
    u64 dart_base;
    u64 l1_physical;
    u64 msi_l2_physical;
    u64 descriptor_physical;
    u32 l1_crc32;
    u32 msi_l2_crc32;
    u32 descriptor_crc32;
    u32 reserved;
} __attribute__((packed));

_Static_assert(sizeof(struct wireless_handoff_descriptor_v2) == 96,
               "wireless handoff ABI v2 size");

u32 wireless_handoff_v2_crc32(const void *data, u32 length);

int wireless_handoff_v2_descriptor_validate(
    const struct wireless_handoff_descriptor_v2 *descriptor,
    const void *reservation, u64 expected_base, u64 expected_size,
    u64 expected_guest_top);

#endif
