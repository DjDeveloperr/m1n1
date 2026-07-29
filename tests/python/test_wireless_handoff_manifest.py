import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "wireless_manifest", ROOT / "tools/j414s-wireless-handoff-manifest.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class WirelessHandoffManifestTests(unittest.TestCase):
    def capture(self, path: Path, base: int) -> None:
        data = bytearray(MODULE.SIZE)
        data[5] = 0x40
        data[MODULE.PAGE + 7] = 0x80
        values = [
            MODULE.SIGNATURE, MODULE.VERSION, MODULE.DESCRIPTOR_SIZE,
            MODULE.FLAGS, 1, 14, base, MODULE.SIZE, base - MODULE.PAGE,
            base + MODULE.SIZE, MODULE.DART_BASE, base, base + MODULE.PAGE,
            base + MODULE.DESCRIPTOR_OFFSET,
            zlib.crc32(data[:MODULE.PAGE]) & 0xFFFFFFFF,
            zlib.crc32(data[MODULE.PAGE:2 * MODULE.PAGE]) & 0xFFFFFFFF,
            0, 0,
        ]
        packed = bytearray(struct.pack(MODULE.DESCRIPTOR_FORMAT, *values))
        values[-2] = zlib.crc32(packed) & 0xFFFFFFFF
        struct.pack_into(MODULE.DESCRIPTOR_FORMAT, data,
                         MODULE.DESCRIPTOR_OFFSET, *values)
        path.write_bytes(data)

    def test_capture_is_strictly_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "reservation.bin"
            base = 0x10022040000
            self.capture(capture, base)
            parsed = MODULE.parse_capture(capture, base)
            self.assertEqual(parsed["reservation_base"], base)
            damaged = bytearray(capture.read_bytes())
            damaged[0] ^= 1
            capture.write_bytes(damaged)
            with self.assertRaisesRegex(ValueError, "CRC"):
                MODULE.parse_capture(capture, base)

    def test_legacy_policy_must_be_explicitly_forbidden(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "m1n1.json"
            path.write_text(json.dumps({
                "schema": MODULE.M1N1_SCHEMA,
                "profile": {
                    "authoritative_wireless_contract":
                        "dynamic_reserved_wireless_handoff_v2",
                    "bcm4388_descriptor_transaction": "selectable",
                },
            }))
            with self.assertRaisesRegex(ValueError, "legacy"):
                MODULE.verify_m1n1(path)


if __name__ == "__main__":
    unittest.main()
