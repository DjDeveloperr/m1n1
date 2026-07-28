/* SPDX-License-Identifier: MIT */

#ifndef MTP_HANDOFF_H
#define MTP_HANDOFF_H

/*
 * Boot the J414s MTP RTKit endpoint for the Windows-native-AIC guest, then
 * leave the DockChannel FIFO and interrupt state entirely to Windows.
 *
 * This is intentionally a no-op unless both the build-time Windows profile
 * and the runtime J414s board check match.  See docs/windows-mtp-handoff.md.
 */
void mtp_handoff_init(void);

#endif
