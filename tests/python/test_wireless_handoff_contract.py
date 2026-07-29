# SPDX-License-Identifier: MIT

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "src" / "wireless_handoff.c").read_text(encoding="utf-8")
IDENTITY = (ROOT / "src" / "platform_identity.c").read_text(encoding="utf-8")
HEADER = (ROOT / "src" / "wireless_handoff.h").read_text(encoding="utf-8")
PROXY_C = (ROOT / "src" / "proxy.c").read_text(encoding="utf-8")
PROXY_H = (ROOT / "src" / "proxy.h").read_text(encoding="utf-8")
PROXY_PY = (ROOT / "proxyclient" / "m1n1" / "proxy.py").read_text(
    encoding="utf-8"
)


class WirelessHandoffContractTests(unittest.TestCase):
    def test_is_opt_in_and_runtime_board_gated(self) -> None:
        config = (ROOT / "config.h").read_text(encoding="utf-8")
        self.assertIn("ENABLE_J414S_WINDOWS_WIRELESS_HANDOFF", config)
        self.assertIn("platform_is_j414s()", SOURCE)
        self.assertIn("identity->chip_id == T6020", IDENTITY)
        self.assertIn("identity->board_id == 4", IDENTITY)
        self.assertIn('"J414s"', IDENTITY)
        self.assertIn('"Mac14,9"', IDENTITY)
        self.assertIn('"J414sAP\\0Mac14,9\\0AppleARM"', IDENTITY)
        self.assertIn("identity->chosen_target_type_len != 0", IDENTITY)
        self.assertIn("int wireless_handoff_init(void);", HEADER)

    def test_explicit_identity_mismatch_is_an_error(self) -> None:
        self.assertIn("WLAN_ERR_IDENTITY = -12", SOURCE)
        self.assertIn("return WLAN_ERR_IDENTITY;", SOURCE)
        self.assertNotIn("if (!wlan_is_j414s())", SOURCE)

    def test_proxy_operation_is_explicit_and_signed(self) -> None:
        self.assertIn("P_WIRELESS_HANDOFF_INIT", PROXY_H)
        self.assertIn("case P_WIRELESS_HANDOFF_INIT:", PROXY_C)
        self.assertIn("P_WIRELESS_HANDOFF_INIT = 0xe02", PROXY_PY)
        self.assertIn(
            "return self.request(self.P_WIRELESS_HANDOFF_INIT, signed=True)",
            PROXY_PY,
        )
        self.assertIn("P_PCIE_WIRELESS_INIT = 0xe03", PROXY_PY)
        self.assertIn(
            "return self.request(self.P_PCIE_WIRELESS_INIT, signed=True)",
            PROXY_PY,
        )

    def test_exact_endpoints_are_required_before_writes(self) -> None:
        preflight = SOURCE.index("static int wlan_check_endpoints_quiescent")
        tables = SOURCE.index("wlan_build_tables();")
        section = SOURCE[preflight:tables]
        self.assertIn("WLAN_WIFI_ID", section)
        self.assertIn("WLAN_BT_ID", section)
        self.assertIn("WLAN_ERR_ENDPOINT_ID", section)

    def test_domain_precedes_requester_routing(self) -> None:
        table = SOURCE.index("wlan_build_tables();")
        translation = SOURCE.index(
            "write32(wlan_dart_regs + WLAN_DART_TCR(WLAN_SID), "
            "WLAN_DART_TCR_TRANSLATE_ENABLE);"
        )
        flush = SOURCE.index("status = wlan_flush_sid1();")
        port = SOURCE.index("pcie_t602x_bcm4388_setup_port0")
        self.assertLess(table, translation)
        self.assertLess(translation, flush)
        self.assertLess(flush, port)

    def test_exact_persistent_geometry(self) -> None:
        for literal in (
            "WLAN_DART0_BASE 0x594000000ULL",
            "WLAN_MSI_DOORBELL_IOVA 0xfffff000ULL",
            "WLAN_MSI_DOORBELL_PAGE 0xffffc000ULL",
            "WLAN_MSI_L1_INDEX      127",
            "WLAN_MSI_L2_INDEX      2047",
            "WLAN_PT_CARVEOUT_PHYS 0x10022000000ULL",
            "WLAN_PT_CARVEOUT_SIZE 0x10000ULL",
            "WLAN_DART_TLB_CMD              0x080",
            "WLAN_DART_TLB_CMD_FLUSH_SID1   0x101",
        ):
            self.assertIn(literal, SOURCE)

    def test_preflight_rejects_live_or_faulted_hardware(self) -> None:
        preflight = SOURCE.index("static int wlan_check_dart_quiescent")
        tables = SOURCE.index("wlan_build_tables();")
        section = SOURCE[preflight:tables]
        self.assertIn("WLAN_DART_PROTECT_TTBR_TCR", section)
        self.assertIn("WLAN_DART_TLB_CMD_BUSY", section)
        self.assertIn("WLAN_DART_TCR(WLAN_SID)", section)
        self.assertIn("WLAN_DART_TTBR(WLAN_SID)", section)
        self.assertIn("WLAN_DART_ERROR_STREAMS", section)
        self.assertIn("WLAN_ERR_PREEXISTING_FAULT", section)

    def test_rollback_never_enables_bypass(self) -> None:
        rollback = SOURCE[
            SOURCE.index("static void wlan_block_sid1") :
            SOURCE.index("static int wlan_mmio_read32")
        ]
        self.assertIn("WLAN_DART_DISABLE_STREAMS", rollback)
        self.assertIn("WLAN_DART_TCR(WLAN_SID), 0", rollback)
        self.assertIn("WLAN_DART_TTBR(WLAN_SID), 0", rollback)
        self.assertNotIn("BYPASS", rollback)


if __name__ == "__main__":
    unittest.main()
