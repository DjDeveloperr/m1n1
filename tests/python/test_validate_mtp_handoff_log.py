"""Tests for the J414s MTP console-log validator."""

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "validate_mtp_handoff_log.py"
SPEC = importlib.util.spec_from_file_location("validate_mtp_handoff_log", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


SUCCESS = (
    "mtp-handoff: preparing J414s MTP for Windows DockChannel ownership\n"
    "rtkit(mtp-handoff): pre-allocated physical buffer "
    "(ep 0x2, phys 0x2a9c00000, size 0x10000)\n"
    "mtp-handoff: RTKit ready; DockChannel[1] RX=312, FIFO preserved "
    "(irq=0x2a9b14000 config=0x2a9b30000 data=0x2a9b34000 "
    "sram=0x2a9c00000/+0x100000)\n"
)


class ValidateMtpHandoffLogTests(unittest.TestCase):
    def test_accepts_complete_live_j414s_contract(self):
        valid, message = MODULE.validate_text(SUCCESS)
        self.assertTrue(valid, message)
        self.assertIn("RX=312", message)

    def test_rejects_stale_dockchannel_map(self):
        valid, message = MODULE.validate_text(
            "mtp-handoff: unexpected J414s DockChannel map "
            "irq=0x2a9b14000/+0x1000 config=0x2a9b28000/+0x1000 "
            "data=0x2a9b2c000\n"
        )
        self.assertFalse(valid)
        self.assertIn("unexpected J414s DockChannel map", message)

    def test_rejects_missing_terminal_record(self):
        valid, message = MODULE.validate_text(
            "mtp-handoff: preparing J414s MTP for Windows DockChannel ownership\n"
        )
        self.assertFalse(valid)
        self.assertIn("started but no terminal success", message)

    def test_rejects_zero_rx_or_wrong_resource(self):
        zero_rx = SUCCESS.replace("RX=312", "RX=0")
        valid, message = MODULE.validate_text(zero_rx)
        self.assertFalse(valid)
        self.assertIn("empty INIT FIFO", message)

        wrong_irq = SUCCESS.replace("irq=0x2a9b14000", "irq=0x2a9b15000")
        valid, message = MODULE.validate_text(wrong_irq)
        self.assertFalse(valid)
        self.assertIn("irq=0x2a9b15000", message)

    def test_rejects_explicit_bounded_failure(self):
        valid, message = MODULE.validate_text(
            "mtp-handoff: no DockChannel INIT data after RTKit AP reached ON\n"
            "mtp-handoff: disabled after setup failure; Windows will not receive partial state\n"
        )
        self.assertFalse(valid)
        self.assertIn("no DockChannel INIT data", message)


if __name__ == "__main__":
    unittest.main()
