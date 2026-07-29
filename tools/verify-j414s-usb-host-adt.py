#!/usr/bin/env python3
"""Verify the private J414s ADT topology used by the USB host handoff."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import stat

from m1n1.adt import load_adt


ADT_SIZE = 491520
ADT_SHA256 = "93d96b4a3ea736288278606b723f263361c6ae6c3d5c4f24f08f6f7a73f4b66e"
MANAGER = "/arm-io/i2c0/hpmBusManager"
PORTS = {
    "hpm0": {"rid": 0, "port-number": 1, "port-location": "left-back"},
    "hpm1": {"rid": 1, "port-number": 2, "port-location": "left-front"},
    "hpm2": {"rid": 2, "port-number": 3, "port-location": "right"},
}


def verify(path: Path) -> None:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise SystemExit("ADT is not a regular private file")
    if stat.S_IMODE(info.st_mode) != 0o600:
        raise SystemExit("ADT mode is not 0600")
    data = path.read_bytes()
    if len(data) != ADT_SIZE or hashlib.sha256(data).hexdigest() != ADT_SHA256:
        raise SystemExit("ADT size/hash does not match the authoritative J414s capture")

    adt = load_adt(data)
    manager = adt[MANAGER]
    for name, expected in PORTS.items():
        node = manager[name]
        actual = {key: node.getprop(key) for key in expected}
        if actual != expected:
            raise SystemExit(f"{name} USB identity mismatch: {actual!r}")
        if list(node.getprop("compatible")) != ["usbc,cd3217"]:
            raise SystemExit(f"{name} is not a CD3217 HPM")

    hpm5 = manager["hpm5"]
    if hpm5.getprop("rid") != 5 or hpm5.getprop("port-location") is not None:
        raise SystemExit("hpm5 non-port identity changed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adt", type=Path, required=True)
    args = parser.parse_args()
    verify(args.adt)
    print(f"J414s USB host ADT: PASS {ADT_SHA256}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
