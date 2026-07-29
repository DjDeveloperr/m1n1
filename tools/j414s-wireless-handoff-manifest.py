#!/usr/bin/env python3
"""Seal/verify the same-instance J414s wireless handoff ABI v2 capture."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import zlib


SCHEMA = "ntasi.j414s.wireless-handoff.v2"
M1N1_SCHEMA = "ntasi.j414s.m1n1-unified.v1"
SIGNATURE = 0x3248574E
VERSION = 2
FLAGS = 1
SIZE = 0x10000
PAGE = 0x4000
DESCRIPTOR_OFFSET = 0xC000
DESCRIPTOR_FORMAT = "<IHHIHHQQQQQQQQIIII"
DESCRIPTOR_SIZE = struct.calcsize(DESCRIPTOR_FORMAT)
DART_BASE = 0x594000000


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_capture(path: Path, base: int) -> dict[str, int]:
    data = path.read_bytes()
    if len(data) != SIZE or base & (PAGE - 1):
        raise ValueError("reservation capture size/alignment mismatch")
    values = struct.unpack_from(DESCRIPTOR_FORMAT, data, DESCRIPTOR_OFFSET)
    names = (
        "signature", "version", "structure_size", "flags", "sid",
        "page_shift", "reservation_base", "reservation_size",
        "guest_memory_top", "physical_memory_top", "dart_base",
        "l1_physical", "msi_l2_physical", "descriptor_physical",
        "l1_crc32", "msi_l2_crc32", "descriptor_crc32", "reserved",
    )
    descriptor = dict(zip(names, values, strict=True))
    expected = {
        "signature": SIGNATURE,
        "version": VERSION,
        "structure_size": DESCRIPTOR_SIZE,
        "flags": FLAGS,
        "sid": 1,
        "page_shift": 14,
        "reservation_base": base,
        "reservation_size": SIZE,
        "dart_base": DART_BASE,
        "l1_physical": base,
        "msi_l2_physical": base + PAGE,
        "descriptor_physical": base + DESCRIPTOR_OFFSET,
        "reserved": 0,
    }
    if any(descriptor[name] != value for name, value in expected.items()):
        raise ValueError("wireless handoff descriptor identity/layout mismatch")
    if not (descriptor["guest_memory_top"] + PAGE <= base and
            base + SIZE <= descriptor["physical_memory_top"]):
        raise ValueError("wireless handoff memory bounds mismatch")
    raw_descriptor = bytearray(data[DESCRIPTOR_OFFSET:DESCRIPTOR_OFFSET + DESCRIPTOR_SIZE])
    struct.pack_into("<I", raw_descriptor, DESCRIPTOR_SIZE - 8, 0)
    checks = {
        "l1_crc32": zlib.crc32(data[:PAGE]) & 0xFFFFFFFF,
        "msi_l2_crc32": zlib.crc32(data[PAGE:2 * PAGE]) & 0xFFFFFFFF,
        "descriptor_crc32": zlib.crc32(raw_descriptor) & 0xFFFFFFFF,
    }
    if any(not descriptor[name] or descriptor[name] != value
           for name, value in checks.items()):
        raise ValueError("wireless handoff CRC mismatch")
    return descriptor


def verify_m1n1(path: Path) -> dict:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema") != M1N1_SCHEMA:
        raise ValueError("m1n1 manifest schema mismatch")
    profile = manifest.get("profile", {})
    if profile.get("authoritative_wireless_contract") != \
            "dynamic_reserved_wireless_handoff_v2":
        raise ValueError("m1n1 manifest does not authorize dynamic handoff v2")
    if profile.get("bcm4388_descriptor_transaction") != \
            "legacy_reference_fixed_layout_no_current_abi_no_call_site":
        raise ValueError("legacy BCM4388 producer policy mismatch")
    return manifest


def make_manifest(m1n1_path: Path, capture_path: Path, base: int) -> dict:
    source = verify_m1n1(m1n1_path)
    descriptor = parse_capture(capture_path, base)
    return {
        "schema": SCHEMA,
        "artifact_status": "READY_FOR_SAME_INSTANCE_MU_BUILD",
        "hardware_touched": True,
        "contract": {
            "name": "dynamic_reserved_wireless_handoff_v2",
            "descriptor_version": VERSION,
            "descriptor_size": DESCRIPTOR_SIZE,
            "descriptor_offset": DESCRIPTOR_OFFSET,
        },
        "m1n1": {
            "manifest_path": str(m1n1_path.resolve()),
            "manifest_sha256": sha256(m1n1_path),
            "source_commit": source["source"]["commit"],
            "macho_sha256": source["files"]["m1n1.macho"]["sha256"],
        },
        "reservation": {
            "capture_path": str(capture_path.resolve()),
            "capture_size": capture_path.stat().st_size,
            "capture_sha256": sha256(capture_path),
            "base": descriptor["reservation_base"],
            "size": descriptor["reservation_size"],
            "guest_memory_top": descriptor["guest_memory_top"],
            "physical_memory_top": descriptor["physical_memory_top"],
        },
        "descriptor": {name: descriptor[name] for name in descriptor},
    }


def verify_manifest(path: Path) -> dict:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema") != SCHEMA or \
            manifest.get("artifact_status") != "READY_FOR_SAME_INSTANCE_MU_BUILD" or \
            manifest.get("hardware_touched") is not True:
        raise ValueError("wireless handoff manifest state mismatch")
    m1n1_path = Path(manifest["m1n1"]["manifest_path"])
    capture_path = Path(manifest["reservation"]["capture_path"])
    rebuilt = make_manifest(m1n1_path, capture_path, manifest["reservation"]["base"])
    if rebuilt != manifest:
        raise ValueError("wireless handoff manifest evidence mismatch")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    seal = sub.add_parser("seal")
    seal.add_argument("--m1n1-manifest", type=Path, required=True)
    seal.add_argument("--reservation-capture", type=Path, required=True)
    seal.add_argument("--reservation-base", type=lambda value: int(value, 0), required=True)
    seal.add_argument("--output", type=Path, required=True)
    verify = sub.add_parser("verify")
    verify.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "seal":
        manifest = make_manifest(
            args.m1n1_manifest.resolve(), args.reservation_capture.resolve(),
            args.reservation_base)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        verify_manifest(args.output)
        print(f"wireless handoff manifest SHA-256: {sha256(args.output)}")
    else:
        verify_manifest(args.manifest.resolve())
        print("J414s wireless handoff ABI v2 manifest: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
