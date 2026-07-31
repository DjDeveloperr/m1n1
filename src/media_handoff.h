/* SPDX-License-Identifier: MIT */

#ifndef MEDIA_HANDOFF_H
#define MEDIA_HANDOFF_H

#include "types.h"

/*
 * J414s Windows media profile (MCA/ADMAC audio, AOP PDM microphones, ISP
 * camera).  See src/media_handoff.c for the full contract; the short version
 * is that the Windows drivers own every mutation of their own devices, and
 * this helper exists to prove -- at EL2, where the evidence actually lives --
 * that the preconditions those drivers assume are true.
 *
 * flags == 0 performs NO register writes anywhere.
 */

/*
 * Read the DART instances whose ADT node carries a PMGR gate (dart-sio,
 * dart-isp0).  Requires raising that gate, which this code then deliberately
 * does NOT lower again: power-gating a DART discards TTBR/TCR, and AppleIsp
 * adopts dart-isp0's inherited translation rather than installing its own.
 * Without this flag those two DARTs are reported as "not probed".
 */
#define MEDIA_HANDOFF_FLAG_PROBE_GATED_DARTS BIT(0)

/*
 * Program the six MCA clock muxes in /arm-io/mca-switch reg[2], the way
 * clk_set_mca_muxes() does on the kboot (Linux) path which the Windows profile
 * never takes.  This is the only write this file can ever perform.
 */
#define MEDIA_HANDOFF_FLAG_MCA_CLOCK_MUXES BIT(1)

#define MEDIA_HANDOFF_FLAG_ALL                                                                     \
    (MEDIA_HANDOFF_FLAG_PROBE_GATED_DARTS | MEDIA_HANDOFF_FLAG_MCA_CLOCK_MUXES)

int media_handoff_init(u32 flags);

#endif
