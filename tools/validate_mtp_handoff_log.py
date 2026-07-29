#!/usr/bin/env python3
"""Validate the J414s Windows MTP handoff from an m1n1 console log."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


EXPECTED = {
    "channel": 1,
    "irq": 0x2A9B14000,
    "config": 0x2A9B30000,
    "data": 0x2A9B34000,
    "sram": 0x2A9C00000,
    "sram_size": 0x100000,
}

SUCCESS_RE = re.compile(
    r"mtp-handoff: RTKit ready; DockChannel\[(?P<channel>\d+)\] "
    r"RX=(?P<rx>\d+), FIFO preserved "
    r"\(irq=(?P<irq>0x[0-9a-fA-F]+) "
    r"config=(?P<config>0x[0-9a-fA-F]+) "
    r"data=(?P<data>0x[0-9a-fA-F]+) "
    r"sram=(?P<sram>0x[0-9a-fA-F]+)/\+(?P<sram_size>0x[0-9a-fA-F]+)\)"
)

FAILURE_PREFIXES = (
    "mtp-handoff: unexpected ",
    "mtp-handoff: incomplete ",
    "mtp-handoff: could not ",
    "mtp-handoff: DAPF setup failed",
    "mtp-handoff: DART stream ",
    "mtp-handoff: IOVA allocator setup failed",
    "mtp-handoff: MTP ASC setup failed",
    "mtp-handoff: MTP RTKit boot failed",
    "mtp-handoff: no DockChannel INIT data",
    "mtp-handoff: disabled after setup failure",
)


def validate_text(text: str) -> tuple[bool, str]:
    """Return whether one complete, exact J414s handoff is present."""

    lines = [line.strip() for line in text.splitlines()]
    failures = [line for line in lines if line.startswith(FAILURE_PREFIXES)]
    matches = list(SUCCESS_RE.finditer(text))

    if failures:
        return False, "MTP handoff failed: " + " | ".join(failures)
    if not matches:
        if any("mtp-handoff: preparing J414s" in line for line in lines):
            return False, "MTP handoff started but no terminal success line was captured"
        return False, "J414s MTP handoff did not run or its log is absent"
    if len(matches) != 1:
        return False, f"expected one MTP success record, found {len(matches)}"

    values = {
        key: int(value, 0)
        for key, value in matches[0].groupdict().items()
    }
    if values["rx"] == 0:
        return False, "MTP success record has an empty INIT FIFO"

    mismatches = []
    for key, expected in EXPECTED.items():
        if values[key] != expected:
            mismatches.append(f"{key}={values[key]:#x} (expected {expected:#x})")
    if mismatches:
        return False, "unexpected J414s handoff values: " + ", ".join(mismatches)

    return (
        True,
        "J414s MTP handoff valid: "
        f"RX={values['rx']} irq={values['irq']:#x} config={values['config']:#x} "
        f"data={values['data']:#x} sram={values['sram']:#x}/+{values['sram_size']:#x}",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        help="m1n1 console log (reads standard input when omitted)",
    )
    args = parser.parse_args()

    text = args.log.read_text(errors="replace") if args.log else sys.stdin.read()
    valid, message = validate_text(text)
    print(message)
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
