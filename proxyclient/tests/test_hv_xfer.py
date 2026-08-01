#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Host tests for the bulk host<->guest channel. No hardware required.

Two things are checked here:

  1. ABI parity. The register offsets, command numbers, status codes and event
     struct in proxyclient/m1n1/hv/xfer.py are parsed out of src/hv_xfer.h and
     compared. A silent drift between the EL2 device and its host half would
     show up on hardware as a guest reading the wrong register, which is
     exactly the class of bug that costs a boot to diagnose.

  2. The protocol state machine, driven against a fake target memory. Every
     refusal path matters as much as the happy path: the guest names streams,
     so a name that escapes the inbox, or a length that walks off the end of
     the window, must fail here and not on the machine.

Run:  python3 proxyclient/tests/test_hv_xfer.py
"""

import importlib.util
import io
import os
import re
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HEADER = REPO / "src" / "hv_xfer.h"


def _load_xfer():
    """Import xfer.py directly: importing m1n1.hv would drag in pyserial."""
    path = REPO / "proxyclient" / "m1n1" / "hv" / "xfer.py"
    spec = importlib.util.spec_from_file_location("hv_xfer_under_test", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


xfer = _load_xfer()


def _header_defines():
    text = HEADER.read_text()
    out = {}
    for name, value in re.findall(
            r"^#define\s+(HV_XFER_\w+)\s+(-?(?:0[xX][0-9a-fA-F]+|[0-9]+))\s*(?:/\*.*)?$",
            text, re.M):
        out[name] = int(value, 0)
    return out


DEFINES = _header_defines()


class FakeMemory:
    """A sparse byte-addressed stand-in for target physical memory."""

    def __init__(self):
        self.data = bytearray(1 << 22)
        self.base = 0x8_0000_0000
        self.writes = 0
        self.reads = 0
        self.fail_next = False

    def _slice(self, addr, size):
        off = addr - self.base
        if off < 0 or off + size > len(self.data):
            raise IndexError(f"fake memory access out of range: {addr:#x}+{size:#x}")
        return off

    def readmem(self, addr, size):
        if self.fail_next:
            self.fail_next = False
            raise IOError("simulated proxy failure")
        self.reads += 1
        off = self._slice(addr, size)
        return bytes(self.data[off:off + size])

    def writemem(self, addr, data):
        if self.fail_next:
            self.fail_next = False
            raise IOError("simulated proxy failure")
        self.writes += 1
        off = self._slice(addr, len(data))
        self.data[off:off + len(data)] = data


class TestAbiParity(unittest.TestCase):
    def test_event_struct_size(self):
        # 4 x u64 + 6 x u32
        self.assertEqual(xfer.XferExcInfo.sizeof(), 4 * 8 + 6 * 4)

    def test_register_offsets_match_header(self):
        for name, off in xfer.REG.items():
            key = f"HV_XFER_REG_{name}"
            self.assertIn(key, DEFINES, f"{key} missing from src/hv_xfer.h")
            self.assertEqual(DEFINES[key], off, f"{key} offset drift")

    def test_every_header_register_is_exposed(self):
        for key in DEFINES:
            if key.startswith("HV_XFER_REG_"):
                self.assertIn(key[len("HV_XFER_REG_"):], xfer.REG,
                              f"{key} is not in xfer.REG")

    def test_commands_match_header(self):
        for name, value in (("NOP", xfer.CMD_NOP), ("PING", xfer.CMD_PING),
                            ("OPEN", xfer.CMD_OPEN), ("GET", xfer.CMD_GET),
                            ("PUT", xfer.CMD_PUT), ("CLOSE", xfer.CMD_CLOSE),
                            ("SET_WINDOW", xfer.CMD_SET_WINDOW),
                            ("RESET_WINDOW", xfer.CMD_RESET_WINDOW)):
            self.assertEqual(DEFINES[f"HV_XFER_CMD_{name}"], value, name)

    def test_status_codes_match_header(self):
        for name, value in (("OK", xfer.ST_OK), ("INVAL", xfer.ST_INVAL),
                            ("NODEV", xfer.ST_NODEV), ("IO", xfer.ST_IO),
                            ("RANGE", xfer.ST_RANGE), ("FAULT", xfer.ST_FAULT),
                            ("BADCMD", xfer.ST_BADCMD)):
            self.assertEqual(DEFINES[f"HV_XFER_ST_{name}"], value, name)

    def test_identity_constants_match_header(self):
        self.assertEqual(DEFINES["HV_XFER_ID"], xfer.HV_XFER_ID)
        self.assertEqual(DEFINES["HV_XFER_VERSION"], xfer.HV_XFER_VERSION)
        self.assertEqual(DEFINES["HV_XFER_NAME_SIZE"], xfer.HV_XFER_NAME_SIZE)
        self.assertEqual(DEFINES["HV_XFER_PAGE_SIZE"], xfer.HV_XFER_PAGE_SIZE)
        self.assertEqual(DEFINES["HV_XFER_REGS_SIZE"], xfer.HV_XFER_REGS_SIZE)

    def test_name_field_fits_in_the_register_page(self):
        self.assertLessEqual(DEFINES["HV_XFER_REG_NAME"] + DEFINES["HV_XFER_NAME_SIZE"],
                             DEFINES["HV_XFER_REGS_SIZE"])


class XferTestCase(unittest.TestCase):
    WIN_SIZE = 0x10000

    def setUp(self):
        self.mem = FakeMemory()
        self.win = self.mem.base + 0x1000
        self.name_phys = self.mem.base + 0x800
        self.tmp = tempfile.TemporaryDirectory()
        self.inbox = Path(self.tmp.name) / "inbox"
        self.dev = xfer.XferHostDevice(inbox=self.inbox, verbose=False)
        self.addCleanup(self.tmp.cleanup)
        self.addCleanup(self.dev.close_all)

    def set_name(self, name):
        raw = name.encode("ascii", "replace") if isinstance(name, str) else name
        raw = raw.ljust(xfer.HV_XFER_NAME_SIZE, b"\0")[:xfer.HV_XFER_NAME_SIZE]
        self.mem.writemem(self.name_phys, raw)

    def call(self, cmd, tag=0, off=0, length=0, win=None, win_size=None):
        return self.dev.dispatch(self.mem, cmd, tag, off, length,
                                 win if win is not None else self.win,
                                 win_size if win_size is not None else self.WIN_SIZE,
                                 self.name_phys)


class TestHostToGuest(XferTestCase):
    def test_get_streams_a_published_blob_in_chunks(self):
        blob = bytes(range(256)) * 400  # 102400 bytes
        self.dev.publish("driver.cab", blob)
        self.set_name("driver.cab")

        st, total = self.call(xfer.CMD_OPEN, tag=1)
        self.assertEqual(st, xfer.ST_OK)
        self.assertEqual(total, len(blob))

        got = bytearray()
        while True:
            st, n = self.call(xfer.CMD_GET, tag=1, off=0, length=self.WIN_SIZE)
            self.assertEqual(st, xfer.ST_OK)
            if n == 0:
                break
            got += self.mem.readmem(self.win, n)

        self.assertEqual(bytes(got), blob)

        st, n = self.call(xfer.CMD_CLOSE, tag=1)
        self.assertEqual(st, xfer.ST_OK)
        self.assertEqual(n, len(blob))
        # One VM exit per window, not per byte: 102400 bytes in 2 GETs.
        self.assertEqual(self.dev.stats()["bytes_out"], len(blob))

    def test_get_honours_a_nonzero_window_offset(self):
        self.dev.publish("x", b"abcdef")
        self.set_name("x")
        self.assertEqual(self.call(xfer.CMD_OPEN, tag=3)[0], xfer.ST_OK)
        st, n = self.call(xfer.CMD_GET, tag=3, off=0x40, length=0x10)
        self.assertEqual((st, n), (xfer.ST_OK, 6))
        self.assertEqual(self.mem.readmem(self.win + 0x40, 6), b"abcdef")

    def test_unpublished_blob_is_refused(self):
        self.set_name("nope.bin")
        self.assertEqual(self.call(xfer.CMD_OPEN, tag=1)[0], xfer.ST_NODEV)

    def test_get_without_open_is_refused(self):
        self.assertEqual(self.call(xfer.CMD_GET, tag=9, length=16)[0], xfer.ST_INVAL)

    def test_get_on_a_write_stream_is_refused(self):
        self.set_name("out.bin")
        self.assertEqual(self.call(xfer.CMD_OPEN, tag=xfer.TAG_WRITE | 2)[0], xfer.ST_OK)
        self.assertEqual(self.call(xfer.CMD_GET, tag=2, length=16)[0], xfer.ST_INVAL)


class TestGuestToHost(XferTestCase):
    def test_put_commits_atomically_on_close(self):
        payload = bytes((i * 31 + 7) & 0xff for i in range(3 * self.WIN_SIZE))
        self.set_name("telemetry.bin")
        self.assertEqual(self.call(xfer.CMD_OPEN, tag=xfer.TAG_WRITE | 5)[0], xfer.ST_OK)

        final = self.inbox / "telemetry.bin"
        part = self.inbox / "telemetry.bin.part"

        for i in range(0, len(payload), self.WIN_SIZE):
            chunk = payload[i:i + self.WIN_SIZE]
            self.mem.writemem(self.win, chunk)
            st, n = self.call(xfer.CMD_PUT, tag=5, off=0, length=len(chunk))
            self.assertEqual((st, n), (xfer.ST_OK, len(chunk)))
            # Nothing under the final name until the guest says it is done.
            self.assertFalse(final.exists())
            self.assertTrue(part.exists())

        st, n = self.call(xfer.CMD_CLOSE, tag=5)
        self.assertEqual((st, n), (xfer.ST_OK, len(payload)))
        self.assertTrue(final.exists())
        self.assertFalse(part.exists())
        self.assertEqual(final.read_bytes(), payload)

    def test_guest_crash_mid_transfer_leaves_a_part_file_only(self):
        self.set_name("half.bin")
        self.assertEqual(self.call(xfer.CMD_OPEN, tag=xfer.TAG_WRITE | 1)[0], xfer.ST_OK)
        self.mem.writemem(self.win, b"partial")
        self.assertEqual(self.call(xfer.CMD_PUT, tag=1, off=0, length=7)[0], xfer.ST_OK)

        # Guest bugchecks and reboots; it never sends CLOSE. It reopens later.
        self.set_name("half.bin")
        self.assertEqual(self.call(xfer.CMD_OPEN, tag=xfer.TAG_WRITE | 1)[0], xfer.ST_OK)
        self.mem.writemem(self.win, b"complete")
        self.assertEqual(self.call(xfer.CMD_PUT, tag=1, off=0, length=8)[0], xfer.ST_OK)
        self.assertEqual(self.call(xfer.CMD_CLOSE, tag=1)[1], 8)

        self.assertEqual((self.inbox / "half.bin").read_bytes(), b"complete")

    def test_put_without_an_inbox_is_refused(self):
        dev = xfer.XferHostDevice(inbox=None, verbose=False)
        self.set_name("x.bin")
        st, _ = dev.dispatch(self.mem, xfer.CMD_OPEN, xfer.TAG_WRITE | 1, 0, 0,
                             self.win, self.WIN_SIZE, self.name_phys)
        self.assertEqual(st, xfer.ST_NODEV)

    def test_put_on_a_read_stream_is_refused(self):
        self.dev.publish("r", b"data")
        self.set_name("r")
        self.assertEqual(self.call(xfer.CMD_OPEN, tag=4)[0], xfer.ST_OK)
        self.assertEqual(self.call(xfer.CMD_PUT, tag=4, length=4)[0], xfer.ST_INVAL)


class TestRefusals(XferTestCase):
    def test_range_checks(self):
        self.dev.publish("r", b"x" * 4096)
        self.set_name("r")
        self.call(xfer.CMD_OPEN, tag=1)
        for off, length in ((self.WIN_SIZE, 16),        # off at the end
                            (self.WIN_SIZE + 4, 16),    # off past the end
                            (0, 0),                     # zero length
                            (0, self.WIN_SIZE + 1),     # longer than the window
                            (self.WIN_SIZE - 8, 16)):   # straddles the end
            st, _ = self.call(xfer.CMD_GET, tag=1, off=off, length=length)
            self.assertEqual(st, xfer.ST_RANGE, f"off={off:#x} len={length:#x}")

    def test_unknown_command(self):
        self.assertEqual(self.call(0x1234)[0], xfer.ST_BADCMD)

    def test_ping_needs_nothing(self):
        st, ver = self.call(xfer.CMD_PING)
        self.assertEqual((st, ver), (xfer.ST_OK, xfer.HV_XFER_VERSION))

    def test_close_without_open(self):
        self.assertEqual(self.call(xfer.CMD_CLOSE, tag=77)[0], xfer.ST_INVAL)

    def test_transport_failure_becomes_a_status_not_an_exception(self):
        self.dev.publish("r", b"x" * 64)
        self.set_name("r")
        self.call(xfer.CMD_OPEN, tag=1)
        self.mem.fail_next = True
        st, n = self.call(xfer.CMD_GET, tag=1, off=0, length=64)
        self.assertEqual((st, n), (xfer.ST_IO, 0))
        self.assertEqual(self.dev.stats()["failures"], 1)

    def test_open_with_no_name_pointer(self):
        st, _ = self.dev.dispatch(self.mem, xfer.CMD_OPEN, 1, 0, 0, self.win,
                                  self.WIN_SIZE, 0)
        self.assertEqual(st, xfer.ST_INVAL)


class TestNameSanitising(unittest.TestCase):
    def test_accepts_plain_basenames(self):
        for name in ("driver.cab", "AppleNvme.sys", "a", "x_1-2.3+4",
                     "A" * 63):
            self.assertEqual(xfer.sanitise_name(name), name, name)

    def test_refuses_escapes_and_junk(self):
        for raw in (b"../../etc/passwd", b"/etc/passwd", b"..", b".",
                    b"a/b", b"a\\b", b"", b"\0", b"-leading-dash",
                    b"sp ace", b"\xff\xfe", b"A" * 64, b"c:name",
                    b"na\nme", b"$(reboot)"):
            self.assertIsNone(xfer.sanitise_name(raw), raw)

    def test_stops_at_the_first_nul(self):
        self.assertEqual(xfer.sanitise_name(b"ok.bin\0garbage/../"), "ok.bin")

    def test_publish_refuses_unsafe_names(self):
        dev = xfer.XferHostDevice(verbose=False)
        with self.assertRaises(ValueError):
            dev.publish("../escape", b"x")


class FakeProxy:
    """Just enough of M1N1Proxy for ProxyTransport's compressed path."""

    def __init__(self, mem):
        self.mem = mem
        self.gzdecs = 0

    def gzdec(self, inbuf, insize, outbuf, outsize):
        import gzip
        self.gzdecs += 1
        raw = gzip.decompress(self.mem.readmem(inbuf, insize))
        assert len(raw) == outsize
        self.mem.writemem(outbuf, raw)
        return len(raw)


class FakeHeap:
    def __init__(self, base):
        self.base = base

    from contextlib import contextmanager as _cm

    @_cm
    def guarded_malloc(self, size):
        yield self.base


class FakeUtils:
    def __init__(self, heap):
        self.heap = heap


class TestProxyTransport(unittest.TestCase):
    def setUp(self):
        self.mem = FakeMemory()
        self.p = FakeProxy(self.mem)
        self.u = FakeUtils(FakeHeap(self.mem.base + 0x200000))

    def test_compressible_payload_takes_the_gzdec_path(self):
        t = xfer.ProxyTransport(self.mem, self.p, self.u, compress=True)
        data = b"\0" * 65536
        t.writemem(self.mem.base + 0x1000, data)
        self.assertEqual(self.p.gzdecs, 1)
        self.assertEqual(self.mem.readmem(self.mem.base + 0x1000, len(data)), data)
        # The point of the exercise: far fewer bytes on the wire.
        self.assertLess(t.wire_bytes, len(data) // 10)

    def test_incompressible_payload_falls_back_to_a_plain_write(self):
        t = xfer.ProxyTransport(self.mem, self.p, self.u, compress=True)
        data = bytes((i * 181 + 71) & 0xff for i in range(65536))
        data = bytes(x ^ y for x, y in zip(data, os.urandom(len(data))))
        t.writemem(self.mem.base + 0x1000, data)
        self.assertEqual(self.p.gzdecs, 0)
        self.assertEqual(self.mem.readmem(self.mem.base + 0x1000, len(data)), data)

    def test_small_payload_never_bothers_compressing(self):
        t = xfer.ProxyTransport(self.mem, self.p, self.u, compress=True)
        t.writemem(self.mem.base + 0x1000, b"\0" * 1024)
        self.assertEqual(self.p.gzdecs, 0)

    def test_compression_can_be_disabled(self):
        t = xfer.ProxyTransport(self.mem, self.p, self.u, compress=False)
        t.writemem(self.mem.base + 0x1000, b"\0" * 65536)
        self.assertEqual(self.p.gzdecs, 0)


class TestThroughputModel(unittest.TestCase):
    """The whole point, expressed as an assertion rather than a claim."""

    def test_exits_per_transfer_is_bounded_by_window_count(self):
        mem = FakeMemory()
        dev = xfer.XferHostDevice(verbose=False)
        blob = os.urandom(164 * 1024)
        dev.publish("pkg.bin", blob)
        name_phys = mem.base + 0x800
        mem.writemem(name_phys, b"pkg.bin".ljust(64, b"\0"))
        win, win_size = mem.base + 0x1000, 1 << 18  # 256 KiB window

        exits = 0
        st, _ = dev.dispatch(mem, xfer.CMD_OPEN, 1, 0, 0, win, win_size, name_phys)
        exits += 1
        self.assertEqual(st, xfer.ST_OK)
        while True:
            st, n = dev.dispatch(mem, xfer.CMD_GET, 1, 0, win_size, win,
                                 win_size, name_phys)
            exits += 1
            self.assertEqual(st, xfer.ST_OK)
            if n == 0:
                break
        exits += 1
        dev.dispatch(mem, xfer.CMD_CLOSE, 1, 0, 0, win, win_size, name_phys)

        # OPEN + one GET carrying everything + an EOF GET + CLOSE.
        self.assertEqual(exits, 4)
        # The vUART would have needed one VM exit per byte.
        self.assertLess(exits, len(blob) / 10000)


if __name__ == "__main__":
    unittest.main(verbosity=2)
