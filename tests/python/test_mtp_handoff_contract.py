"""Static contract checks for the J414s Windows MTP/DockChannel handoff."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "src" / "mtp_handoff.c").read_text()
IDENTITY = (ROOT / "src" / "platform_identity.c").read_text()
RTKIT_SOURCE = (ROOT / "src" / "rtkit.c").read_text()
RTKIT_HEADER = (ROOT / "src" / "rtkit.h").read_text()
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
        self.assertIn("if (!platform_is_j414s())", SOURCE)
        self.assertIn("bool platform_identity_matches_j414s", IDENTITY)
        self.assertIn("mtp_handoff_init();", HV)

    def test_mtp_handoff_uses_the_mtp_dart_and_rtkit_boot_path(self):
        self.assertIn('dapf_init(MTP_DART_PATH, MTP_DOCKCHANNEL_INDEX)', SOURCE)
        self.assertIn('dart_init_adt(MTP_DART_PATH, 0, MTP_DOCKCHANNEL_INDEX, false)', SOURCE)
        self.assertIn('rtkit_init("mtp-handoff"', SOURCE)
        self.assertIn("rtkit_set_phys_window(mtp_handoff.rtkit", SOURCE)
        self.assertIn("rtkit_boot_timed(mtp_handoff.rtkit, MTP_READY_TIMEOUT)", SOURCE)
        rollback = SOURCE.index("static void mtp_handoff_rollback")
        stop = SOURCE.index("asc_cpu_stop(mtp_handoff.asc)", rollback)
        release = SOURCE.index("rtkit_free(mtp_handoff.rtkit)", rollback)
        self.assertLess(stop, release)
        self.assertNotIn("rtkit_quiesce(mtp_handoff.rtkit)", SOURCE)

    def test_mtp_handoff_waits_for_ap_on_and_nonempty_init_fifo(self):
        self.assertIn("bool rtkit_boot_timed", RTKIT_HEADER)
        self.assertIn("rtkit_wait_for_power", RTKIT_SOURCE)
        self.assertIn('&rtk->ap_power, RTKIT_POWER_ON, timeout_usec, "AP"', RTKIT_SOURCE)
        self.assertIn("MTP_READY_TIMEOUT    (3 * USEC_PER_SEC)", SOURCE)
        self.assertIn("while (!timeout_expired(timeout))", SOURCE)
        self.assertIn("if (!mtp_handoff.initial_rx_count)", SOURCE)
        self.assertIn("no DockChannel INIT data after RTKit AP reached ON", SOURCE)

    def test_rtkit_physical_buffers_are_bounded_to_mtp_sram(self):
        self.assertIn("bool rtkit_set_phys_window", RTKIT_HEADER)
        self.assertIn("base > UINT64_MAX - size", RTKIT_SOURCE)
        self.assertIn("addr >= rtk->phys_window_base", RTKIT_SOURCE)
        self.assertIn(
            "sz <= rtk->phys_window_size - (addr - rtk->phys_window_base)",
            RTKIT_SOURCE,
        )
        self.assertIn("J414S_MTP_FIXED_BUFFER_BASE 0x2a9c00000ULL", SOURCE)
        self.assertIn("J414S_MTP_FIXED_BUFFER_SIZE 0x100000ULL", SOURCE)
        self.assertIn(
            "mtp_handoff.sram_base = J414S_MTP_FIXED_BUFFER_BASE", SOURCE
        )
        self.assertIn("rtkit_set_phys_window(mtp_handoff.rtkit", SOURCE)

    def test_mtp_handoff_preserves_dockchannel_init_fifo(self):
        self.assertIn("RX_COUNT is the sole DockChannel register polled", SOURCE)
        self.assertIn("read32(mtp_handoff.data_base + DOCKCHANNEL_RX_COUNT)", SOURCE)
        self.assertIn("RX_8", SOURCE)
        self.assertIn("RX_32", SOURCE)
        self.assertIn("writes a DockChannel IRQ mask", DOC)
        self.assertIn("never reads `RX_8` or `RX_32`", DOC)

    def test_mtp_handoff_documents_the_acpi_contract(self):
        for address in ("0x2a9b14000", "0x2a9b30000", "0x2a9b34000", "0x2a9c00000"):
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
