/* SPDX-License-Identifier: MIT */

#ifndef WIRELESS_HANDOFF_H
#define WIRELESS_HANDOFF_H

/*
 * Install the persistent J414s BCM4388 SID-1 deny-all domain. The caller must
 * have completed pcie_init(), and must treat any nonzero result as a hard stop
 * before starting Mu or Windows.
 */
int wireless_handoff_init(void);

#endif
