"""Static contract checks for the J414s Windows MTP/DockChannel handoff."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "src" / "mtp_handoff.c").read_text()
HV = (ROOT / "src" / "hv.c").read_text()
CONFIG = (ROOT / "config.h").read_text()
DOC = (ROOT / "docs" / "windows-mtp-handoff.md").read_text()


class MtpHandoffContractTests(unittest.TestCase):
    def test_mtp_handoff_is_strictly_windows_native_aic_and_j414s_gated(self):
        self.assertIn("#define ENABLE_J414S_WINDOWS_MTP_HANDOFF", CONFIG)
        self.assertIn(
            "defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && defined(ENABLE_J414S_WINDOWS_MTP_HANDOFF)",
            SOURCE,
        )
        self.assertIn('chip_id != T6020 || !adt_is_compatible(adt, 0, "J414sAP")', SOURCE)
        self.assertIn("mtp_handoff_init();", HV)

    def test_mtp_handoff_uses_the_mtp_dart_and_rtkit_boot_path(self):
        self.assertIn('dapf_init(MTP_DART_PATH, MTP_DOCKCHANNEL_INDEX)', SOURCE)
        self.assertIn('dart_init_adt(MTP_DART_PATH, 0, MTP_DOCKCHANNEL_INDEX, false)', SOURCE)
        self.assertIn('rtkit_init("mtp-handoff"', SOURCE)
        self.assertIn("rtkit_boot(mtp_handoff.rtkit)", SOURCE)

    def test_mtp_handoff_preserves_dockchannel_init_fifo(self):
        self.assertIn("sole DockChannel register read", SOURCE)
        self.assertIn("read32(mtp_handoff.data_base + DOCKCHANNEL_RX_COUNT)", SOURCE)
        self.assertIn("RX_8", SOURCE)
        self.assertIn("RX_32", SOURCE)
        self.assertIn("writes a DockChannel IRQ mask", DOC)
        self.assertIn("never reads `RX_8` or `RX_32`", DOC)

    def test_mtp_handoff_documents_the_acpi_contract(self):
        for address in ("0x2a9b14000", "0x2a9b30000", "0x2a9b34000"):
            self.assertIn(address, DOC)
            self.assertIn(address, SOURCE)
        self.assertNotIn("0x2a9b28000", SOURCE + DOC)
        self.assertNotIn("0x2a9b2c000", SOURCE + DOC)
        self.assertIn("0x1000", DOC)
        self.assertIn("J414S_MTP_APERTURE_SIZE 0x1000", SOURCE)
        self.assertIn("677", DOC)
        self.assertIn("GPIO and interface firmware boundary", DOC)


if __name__ == "__main__":
    unittest.main()
