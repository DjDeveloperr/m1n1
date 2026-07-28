#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import unittest

from m1n1.hv import HV


class _Esr:
    ISS = 0x5360


class _Context:
    esr = _Esr()
    elr = 0x1000


class _FakeHv:
    def __init__(self):
        self.brkcall_handlers = {}
        self.hvcall_handlers = {}
        self.lowered = False

    def _lower(self):
        self.lowered = True
        return False


class BrkCallTests(unittest.TestCase):
    def test_brk_immediate_dispatch_preserves_x0_and_advances_once(self):
        fake = _FakeHv()
        context = _Context()
        context.regs = [0xC0000034] + [0] * 30
        seen = []

        HV.add_brkcall(fake, 0x5360, lambda ctx: seen.append(ctx.regs[0]) or True)
        self.assertTrue(HV.handle_brk(fake, context))
        self.assertEqual(seen, [0xC0000034])
        self.assertEqual(context.regs[0], 0xC0000034)
        self.assertEqual(context.elr, 0x1004)
        self.assertFalse(fake.lowered)

    def test_unregistered_brk_keeps_legacy_lowering_behavior(self):
        fake = _FakeHv()
        context = _Context()
        context.esr = type("Esr", (), {"ISS": 0x1234})()
        self.assertFalse(HV.handle_brk(fake, context))
        self.assertTrue(fake.lowered)

    def test_call_id_must_fit_brk_immediate(self):
        fake = _FakeHv()
        with self.assertRaises(ValueError):
            HV.add_brkcall(fake, 0x10000, lambda _ctx: True)


if __name__ == "__main__":
    unittest.main()
