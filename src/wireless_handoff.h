/* SPDX-License-Identifier: MIT */

#ifndef WIRELESS_HANDOFF_H
#define WIRELESS_HANDOFF_H

#include "types.h"

/*
 * Install the persistent J414s BCM4388 SID-1 deny-all domain. The caller must
 * have completed pcie_init(), reserved the supplied range with the monotonic
 * top-of-memory allocator, and paired that exact range with Mu's DRT0 profile.
 * Any nonzero result is a hard stop before starting Mu or Windows.
 */
int wireless_handoff_init(u64 reservation_base, u64 reservation_size);

#endif
