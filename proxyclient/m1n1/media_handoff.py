# SPDX-License-Identifier: MIT
"""J414s Windows media profile: host-side contract for ``P_MEDIA_HANDOFF_INIT``.

This module is the host-side counterpart of ``src/media_handoff.c``.  It is
deliberately much smaller than :mod:`m1n1.wireless_handoff`, and for a reason
worth stating rather than leaving implicit.

Why there is no descriptor here
-------------------------------
The wireless handoff needs an ABI: m1n1 installs a DART domain at EL2 before Mu
exists, so Mu and ``AppleDart.sys`` have to find and validate a table neither of
them watched being built.  That is what ``NWH2`` is for.

The media profile has no such gap.  Each Windows driver performs its own device
mutation after it starts -- ``AppleMcaAudio`` raises its own PMGR chain and
installs its own ``dart-sio`` SID-2 domain, ``AppleIsp`` raises ``ps_isp_*`` and
adopts ``dart-isp0``'s inherited page table, ``AppleAopAudio`` writes to no DART
at all -- so there is nothing for firmware to hand over and nothing for a
descriptor to describe.  Inventing one would add a second source of truth for
state the drivers already read directly out of the hardware.

What ``media_handoff_init()`` is therefore for is evidence and one clock write.
The two flags below are the entire mutation surface, and ``flags=0`` writes
nothing anywhere.

Offline use
-----------
:func:`isp_carveout_from_adt` and :func:`aop_dram_segments_from_adt` reproduce
the two DRAM audits the device side performs, so a captured ADT plus a captured
``boot_args`` can be checked without hardware -- see
``tests/python/test_media_handoff_contract.py``.
"""

import struct

#: Read the DARTs whose ADT node carries a PMGR gate (``dart-sio``,
#: ``dart-isp0``).  Raises those gates and deliberately never lowers them.
FLAG_PROBE_GATED_DARTS = 1 << 0

#: Program the six MCA clock muxes, the way ``clk_set_mca_muxes()`` does on the
#: kboot path the Windows profile never takes.  The only write available.
FLAG_MCA_CLOCK_MUXES = 1 << 1

FLAG_ALL = FLAG_PROBE_GATED_DARTS | FLAG_MCA_CLOCK_MUXES

#: Errors ``media_handoff_init()`` can return, mirroring ``enum
#: media_handoff_error`` in ``src/media_handoff.c``.
ERRORS = {
    0: "OK",
    -1: "platform identity is not J414s",
    -2: "unknown flag bits",
    -3: "a required ADT node or reg entry is missing",
    -4: "a reg window is not at its pinned J414s address",
    -5: "segment-ranges could not be parsed",
    -6: "the ISP firmware carveout overlaps allocatable DRAM",
    -7: "a gated DART's PMGR domain could not be raised",
    -8: "an MCA clock mux did not read back",
}

#: ``struct adt_segment_ranges`` (``src/adt.h``): phys, iova, remap, size, unk.
SEGMENT_RANGE_FORMAT = "<QQQII"
SEGMENT_RANGE_SIZE = struct.calcsize(SEGMENT_RANGE_FORMAT)
assert SEGMENT_RANGE_SIZE == 32

FOUR_GIB = 1 << 32

__all__ = [
    "FLAG_PROBE_GATED_DARTS",
    "FLAG_MCA_CLOCK_MUXES",
    "FLAG_ALL",
    "ERRORS",
    "MediaHandoffError",
    "parse_segment_ranges",
    "dram_extent",
    "isp_carveout_from_adt",
    "aop_dram_segments_from_adt",
    "check_isp_carveout",
]


class MediaHandoffError(Exception):
    """A media-profile precondition does not hold."""


def parse_segment_ranges(blob):
    """Decode an ADT ``segment-ranges`` blob into dicts."""
    blob = bytes(blob)
    if not blob or len(blob) % SEGMENT_RANGE_SIZE:
        raise MediaHandoffError(
            f"segment-ranges length {len(blob)} is not a multiple of "
            f"{SEGMENT_RANGE_SIZE}"
        )
    out = []
    for off in range(0, len(blob), SEGMENT_RANGE_SIZE):
        phys, iova, remap, size, unk = struct.unpack(
            SEGMENT_RANGE_FORMAT, blob[off : off + SEGMENT_RANGE_SIZE]
        )
        out.append(
            {"phys": phys, "iova": iova, "remap": remap, "size": size, "unk": unk}
        )
    return out


def dram_extent(segments, dram_base):
    """``[lo, hi)`` over the segments at or above ``dram_base``.

    Segments below ``dram_base`` are on-chip SRAM (the AOP's ``__TEXT``/
    ``__DATA`` at ``0x2a6c00000``) and must not widen the extent -- they are not
    memory any allocator can hand out.  Returns ``None`` when a node has no DRAM
    segment at all.
    """
    lo = None
    hi = None
    for seg in segments:
        if seg["phys"] < dram_base:
            continue
        end = seg["phys"] + seg["size"]
        lo = seg["phys"] if lo is None else min(lo, seg["phys"])
        hi = end if hi is None else max(hi, end)
    if hi is None:
        return None
    return lo, hi


def _segment_ranges_of(adt, path):
    node = adt[path]
    blob = node._properties["segment-ranges"]
    if not isinstance(blob, (bytes, bytearray)):
        raise MediaHandoffError(f"{path}: segment-ranges is not a raw blob")
    return parse_segment_ranges(blob)


def isp_carveout_from_adt(adt, dram_base, path="/arm-io/isp0"):
    """``(lo, hi)`` of the iBoot-placed ISP firmware image, from the ADT."""
    extent = dram_extent(_segment_ranges_of(adt, path), dram_base)
    if extent is None:
        raise MediaHandoffError(f"{path} declares no DRAM firmware segment")
    return extent


def aop_dram_segments_from_adt(adt, dram_base, path="/arm-io/aop"):
    """``(lo, hi)`` of the AOP's DRAM segments, or ``None`` if it has none."""
    return dram_extent(_segment_ranges_of(adt, path), dram_base)


def check_isp_carveout(carveout, phys_base, mem_size):
    """Reproduce ``media_check_isp_carveout()``.

    ``phys_base``/``mem_size`` are m1n1's own ``boot_args`` window, not the
    guest's: the hypervisor gives the guest ``phys_base = heap_top``, so the
    guest window is a strict subset and this is the stronger test.  Returns the
    margin in bytes between the carveout's end and ``phys_base``.
    """
    lo, hi = carveout
    if lo < hi and phys_base < hi and lo < phys_base + mem_size:
        raise MediaHandoffError(
            f"ISP firmware carveout {lo:#x}..{hi:#x} overlaps allocatable DRAM "
            f"{phys_base:#x}..{phys_base + mem_size:#x}; the camera image would "
            "be destroyed"
        )
    if hi > phys_base:
        raise MediaHandoffError(
            f"ISP firmware carveout {lo:#x}..{hi:#x} is above "
            f"boot_args.phys_base {phys_base:#x} but outside mem_size; refusing "
            "to call that safe"
        )
    return phys_base - hi
