#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for the HV-side glue of the bulk channel. No hardware required.

test_hv_xfer.py covers the protocol engine in isolation. This file covers the
three pieces of m1n1/hv/__init__.py that sit between the engine and the target,
because each of them can only fail on hardware otherwise:

  * xfer_translate(), the Python twin of xfer_validate_window() in
    src/hv_xfer.c. It is what stands between a guest-supplied pointer and a
    bulk write, on the hypercall path where there is no C validation at all.
  * handle_xfer(), which round-trips hv_xfer_exc_info through the real
    construct definition -- a field-order or width mistake there would show up
    on hardware as a command with the wrong length.
  * the BRK #0x4242 hypercall front end: register decode, and the cache
    maintenance invariant (always after the access, over the range actually
    touched).

Needs pyserial and construct, i.e. m1n1's venv. Skips cleanly without them so
it can sit in a runner that also has to work with a bare python3.
"""

import sys
import types
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "proxyclient"))

try:
    from m1n1.hv import HV
    from m1n1.hv.xfer import (XferExcInfo, XferHostDevice, CMD_GET, CMD_PUT,
                              CMD_OPEN, ST_OK, ST_INVAL, TAG_WRITE,
                              HV_XFER_PAGE_SIZE, HV_XFER_NAME_SIZE)
except ImportError as exc:  # pragma: no cover - environment dependent
    print(f"SKIP: m1n1 proxyclient not importable ({exc})")
    sys.exit(0)


RAM_BASE = 0x8_0000_0000
RAM_SIZE = 0x1_0000_0000
PTE_VALID = 1


class FakeBootArgs:
    phys_base = RAM_BASE + 0x1000_0000
    mem_size_actual = RAM_SIZE


class FakeProxy:
    def __init__(self, walk):
        self._walk = walk
        self.civacs = []
        self.exits = []

    def hv_pt_walk(self, addr):
        return self._walk(addr)

    def dc_civac(self, addr, size):
        self.civacs.append((addr, size))

    def exit(self, ret):
        self.exits.append(ret)


class FakeMem:
    """Byte-addressed stand-in for target memory, shared by iface + transport."""

    def __init__(self, base=RAM_BASE, size=1 << 20):
        self.base = base
        self.data = bytearray(size)

    def readmem(self, addr, size):
        off = addr - self.base
        assert 0 <= off and off + size <= len(self.data), f"{addr:#x}+{size:#x}"
        return bytes(self.data[off:off + size])

    def writemem(self, addr, data):
        off = addr - self.base
        assert 0 <= off and off + len(data) <= len(self.data), f"{addr:#x}"
        self.data[off:off + len(data)] = data

    def readstruct(self, addr, cls):
        return cls.parse(self.readmem(addr, cls.sizeof()))


def make_hv(walk=None, mem=None):
    """A duck-typed HV good enough for the methods under test."""
    if walk is None:
        def walk(addr):
            if RAM_BASE <= addr < RAM_BASE + RAM_SIZE:
                return addr | PTE_VALID
            return 0

    hv = types.SimpleNamespace()
    hv.PTE_VALID = HV.PTE_VALID
    hv.p = FakeProxy(walk)
    hv.u = types.SimpleNamespace(ba=FakeBootArgs())
    hv.iface = mem if mem is not None else FakeMem()
    hv.xfer_dev = None
    hv.xfer_transport = None
    hv.xfer_hvcall_id = None
    hv.hvcalls = {}
    hv.add_hvcall = lambda cid, fn: hv.hvcalls.__setitem__(cid, fn)
    for name in ("_xfer_dram_range", "xfer_translate", "handle_xfer",
                 "enable_xfer_hvcall"):
        setattr(hv, name, types.MethodType(getattr(HV, name), hv))
    return hv


class TestXferTranslate(unittest.TestCase):
    def test_accepts_an_identity_mapped_dram_window(self):
        hv = make_hv()
        ipa = RAM_BASE + 0x2000_0000
        self.assertEqual(hv.xfer_translate(ipa, 0x10000), ipa)

    def test_refuses_alignment_and_size_errors(self):
        hv = make_hv()
        ipa = RAM_BASE + 0x2000_0000
        for bad_ipa, bad_size in ((ipa + 4, 0x8000), (ipa, 0x8004), (ipa, 0),
                                  (ipa, -0x4000)):
            self.assertIsNone(hv.xfer_translate(bad_ipa, bad_size),
                              f"{bad_ipa:#x}+{bad_size:#x}")

    def test_refuses_an_unmapped_page_in_the_middle(self):
        hole = RAM_BASE + 0x2000_0000 + HV_XFER_PAGE_SIZE

        def walk(addr):
            if addr == hole:
                return 0
            return addr | PTE_VALID if RAM_BASE <= addr < RAM_BASE + RAM_SIZE else 0

        hv = make_hv(walk)
        self.assertIsNone(hv.xfer_translate(RAM_BASE + 0x2000_0000, 0x10000))

    def test_refuses_a_discontiguous_window(self):
        odd = RAM_BASE + 0x2000_0000 + HV_XFER_PAGE_SIZE

        def walk(addr):
            pa = addr + (HV_XFER_PAGE_SIZE if addr == odd else 0)
            return pa | PTE_VALID

        hv = make_hv(walk)
        self.assertIsNone(hv.xfer_translate(RAM_BASE + 0x2000_0000, 0x10000))

    def test_refuses_mmio_even_though_it_is_mapped_and_contiguous(self):
        # THE case that matters: /arm-io is mapped HW too, so contiguity alone
        # would happily let a guest aim a bulk transfer at a device.
        hv = make_hv(walk=lambda addr: addr | PTE_VALID)
        self.assertIsNone(hv.xfer_translate(0x2_3000_0000, 0x8000))

    def test_refuses_a_window_that_runs_off_the_top_of_dram(self):
        hv = make_hv(walk=lambda addr: addr | PTE_VALID)
        self.assertIsNone(hv.xfer_translate(RAM_BASE + RAM_SIZE - 0x4000, 0x8000))

    def test_refuses_a_pte_without_the_valid_bit(self):
        hv = make_hv(walk=lambda addr: addr)  # mapped SW, not HW
        self.assertIsNone(hv.xfer_translate(RAM_BASE + 0x2000_0000, 0x4000))


class TestHandleXfer(unittest.TestCase):
    """Round-trips the real hv_xfer_exc_info through the real construct."""

    INFO_AT = RAM_BASE + 0x1000
    WIN_AT = RAM_BASE + 0x10000
    NAME_AT = RAM_BASE + 0x800
    WIN_SIZE = 0x8000

    def setUp(self):
        self.mem = FakeMem()
        self.hv = make_hv(mem=self.mem)
        # readstruct() is asked for ExcInfo first; only .data is used.
        real_readstruct = self.mem.readstruct

        def readstruct(addr, cls):
            if cls is not XferExcInfo:
                return types.SimpleNamespace(data=self.INFO_AT)
            return real_readstruct(addr, cls)

        self.mem.readstruct = readstruct
        self.hv.xfer_transport = self.mem

    def put_info(self, cmd, tag=0, off=0, length=0):
        self.mem.writemem(self.INFO_AT, XferExcInfo.build({
            "devbase": 0x2_0000_0000,
            "win_phys": self.WIN_AT,
            "win_size": self.WIN_SIZE,
            "name_phys": self.NAME_AT,
            "cmd": cmd, "tag": tag, "off": off, "length": length,
            "result": 0, "status": -2,
        }))

    def get_info(self):
        return XferExcInfo.parse(self.mem.readmem(self.INFO_AT,
                                                  XferExcInfo.sizeof()))

    def test_no_engine_answers_nodev_and_never_hangs(self):
        self.put_info(CMD_GET, length=16)
        self.hv.handle_xfer(None, None, 0)
        self.assertEqual(self.get_info().status, -2)  # ST_NODEV
        self.assertEqual(len(self.hv.p.exits), 1)

    def test_get_moves_bytes_into_the_window(self):
        dev = XferHostDevice(verbose=False)
        dev.publish("blob", b"m1n1" * 64)
        self.hv.xfer_dev = dev

        self.mem.writemem(self.NAME_AT, b"blob".ljust(HV_XFER_NAME_SIZE, b"\0"))
        self.put_info(CMD_OPEN, tag=1)
        self.hv.handle_xfer(None, None, 0)
        info = self.get_info()
        self.assertEqual((info.status, info.result), (ST_OK, 256))

        self.put_info(CMD_GET, tag=1, off=0x40, length=0x1000)
        self.hv.handle_xfer(None, None, 0)
        info = self.get_info()
        self.assertEqual((info.status, info.result), (ST_OK, 256))
        self.assertEqual(self.mem.readmem(self.WIN_AT + 0x40, 256), b"m1n1" * 64)

        # The struct came back with every other field intact.
        self.assertEqual(info.win_phys, self.WIN_AT)
        self.assertEqual(info.win_size, self.WIN_SIZE)
        self.assertEqual(info.cmd, CMD_GET)
        self.assertEqual(info.length, 0x1000)

    def test_a_negative_status_survives_the_round_trip(self):
        self.hv.xfer_dev = XferHostDevice(verbose=False)
        self.put_info(CMD_GET, tag=9, length=16)  # no such stream
        self.hv.handle_xfer(None, None, 0)
        self.assertEqual(self.get_info().status, ST_INVAL)


class TestHvcallFrontEnd(unittest.TestCase):
    WIN = RAM_BASE + 0x10000
    WIN_SIZE = 0x8000
    NAME_AT = RAM_BASE + 0x800

    def setUp(self):
        self.mem = FakeMem()
        self.hv = make_hv(mem=self.mem)
        self.dev = XferHostDevice(verbose=False)
        self.hv.xfer_dev = self.dev
        self.hv.xfer_transport = self.mem
        self.callid = self.hv.enable_xfer_hvcall()
        self.handler = self.hv.hvcalls[self.callid]

    def ctx(self, cmd, win=None, win_size=None, off=0, length=0, tag=0,
            name=0):
        regs = [0] * 32
        regs[0] = self.callid
        regs[1] = cmd
        regs[2] = self.WIN if win is None else win
        regs[3] = self.WIN_SIZE if win_size is None else win_size
        regs[4] = off
        regs[5] = length
        regs[6] = tag
        regs[7] = name
        return types.SimpleNamespace(regs=regs)

    def test_get_decodes_registers_and_syncs_what_was_written(self):
        self.dev.publish("pkg", b"\xa5" * 1024)
        self.mem.writemem(self.NAME_AT, b"pkg".ljust(HV_XFER_NAME_SIZE, b"\0"))

        c = self.ctx(CMD_OPEN, tag=3, name=self.NAME_AT)
        self.assertTrue(self.handler(c))
        self.assertEqual((c.regs[0], c.regs[1]), (ST_OK, 1024))

        c = self.ctx(CMD_GET, tag=3, off=0x100, length=0x2000)
        self.assertTrue(self.handler(c))
        self.assertEqual((c.regs[0], c.regs[1]), (ST_OK, 1024))
        self.assertEqual(self.mem.readmem(self.WIN + 0x100, 1024), b"\xa5" * 1024)

        # Cache maintenance covers exactly what the host wrote, and only that.
        self.assertEqual(self.hv.p.civacs[-1], (self.WIN + 0x100, 1024))

    def test_put_syncs_the_requested_length(self):
        self.dev.set_inbox(Path(__file__).resolve().parent / "_xfer_tmp")
        self.mem.writemem(self.NAME_AT, b"out".ljust(HV_XFER_NAME_SIZE, b"\0"))
        self.assertTrue(self.handler(self.ctx(CMD_OPEN, tag=TAG_WRITE | 4,
                                              name=self.NAME_AT)))
        self.mem.writemem(self.WIN, b"z" * 512)
        c = self.ctx(CMD_PUT, tag=4, off=0, length=512)
        self.assertTrue(self.handler(c))
        self.assertEqual((c.regs[0], c.regs[1]), (ST_OK, 512))
        self.assertEqual(self.hv.p.civacs[-1], (self.WIN, 512))
        self.dev.close_all()
        import shutil
        shutil.rmtree(self.dev.inbox, ignore_errors=True)

    def test_a_bad_window_is_refused_before_the_engine_sees_it(self):
        for win, size in ((0x2_3000_0000, 0x8000),          # MMIO
                          (self.WIN + 4, 0x8000),           # unaligned
                          (RAM_BASE + RAM_SIZE, 0x8000)):   # off the top
            c = self.ctx(CMD_GET, win=win, win_size=size, length=16)
            self.assertTrue(self.handler(c))
            self.assertEqual(c.regs[0], ST_INVAL & 0xffffffffffffffff)
            self.assertEqual(c.regs[1], 0)
        self.assertEqual(self.hv.p.civacs, [])
        self.assertEqual(self.dev.commands, 0)

    def test_a_bad_name_pointer_is_refused(self):
        c = self.ctx(CMD_OPEN, tag=1, name=0x2_3000_0000)
        self.assertTrue(self.handler(c))
        self.assertEqual(c.regs[0], ST_INVAL & 0xffffffffffffffff)

    def test_a_name_straddling_the_end_of_its_page_is_refused(self):
        near_end = self.WIN + HV_XFER_PAGE_SIZE - 8
        c = self.ctx(CMD_OPEN, tag=1, name=near_end)
        self.assertTrue(self.handler(c))
        self.assertEqual(c.regs[0], ST_INVAL & 0xffffffffffffffff)

    def test_a_failed_command_performs_no_cache_maintenance(self):
        c = self.ctx(CMD_GET, tag=77, length=16)  # no such stream
        self.assertTrue(self.handler(c))
        self.assertEqual(c.regs[0], ST_INVAL & 0xffffffffffffffff)
        self.assertEqual(self.hv.p.civacs, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
