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
        self.assertIn(
            "int wireless_handoff_init(u64 reservation_base, u64 reservation_size);",
            HEADER,
        )

    def test_explicit_identity_mismatch_is_an_error(self) -> None:
        self.assertIn("WLAN_ERR_IDENTITY = -12", SOURCE)
        self.assertIn("return WLAN_ERR_IDENTITY;", SOURCE)
        self.assertNotIn("if (!wlan_is_j414s())", SOURCE)

    def test_proxy_operation_is_explicit_and_signed(self) -> None:
        self.assertIn("P_WIRELESS_HANDOFF_INIT", PROXY_H)
        self.assertIn("case P_WIRELESS_HANDOFF_INIT:", PROXY_C)
        self.assertIn("P_WIRELESS_HANDOFF_INIT = 0xe02", PROXY_PY)
        self.assertIn("reservation_base=None, reservation_size=None", PROXY_PY)
        self.assertIn("reservation_base,", PROXY_PY)
        self.assertIn("reservation_size,", PROXY_PY)
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

    def test_endpoint_identities_are_the_live_enumerated_ids(self) -> None:
        # Live bus enumeration on J414s: bus 1 device 0 function 0 is
        # 14e4:4434 (Wi-Fi) and function 1 is 14e4:5f72 (Bluetooth) -- one
        # device, two functions, both behind APCIE port 0.
        self.assertIn("#define WLAN_WIFI_ID    0x443414e4U", SOURCE)
        self.assertIn("#define WLAN_BT_ID      0x5f7214e4U", SOURCE)
        self.assertIn(
            "static const u32 expected_identity[2] = {WLAN_WIFI_ID, WLAN_BT_ID};",
            SOURCE,
        )
        self.assertIn("#define WLAN_ENDPOINT_BUS          1U", SOURCE)

    def test_bus_routing_is_proven_before_the_identity_is_believed(self) -> None:
        # A root port left at secondary=subordinate=0 forwards no config
        # request, so every endpoint read is 0xffffffff.  That must report as
        # its own error, never as a bogus identity mismatch.
        self.assertIn("WLAN_ERR_BUS_ROUTING = -18", SOURCE)
        self.assertIn("static int wlan_check_bus_routing", SOURCE)
        quiescent = SOURCE[
            SOURCE.index("static int wlan_check_endpoints_quiescent") :
            SOURCE.index("static int wlan_check_dart_quiescent")
        ]
        routing = quiescent.index("wlan_check_bus_routing()")
        identity = quiescent.index("wlan_pci_identity(function)")
        self.assertLess(routing, identity)

    def test_quiescence_rule_is_bus_master_only(self) -> None:
        # Mu and Windows both enumerate and assign BARs before this runs, so
        # requiring command == 0 would make the handoff unreachable.  Only Bus
        # Master Enable (bit 2) may block it.
        quiescent = SOURCE[
            SOURCE.index("static int wlan_check_endpoints_quiescent") :
            SOURCE.index("static int wlan_check_dart_quiescent")
        ]
        self.assertIn("if (command & BIT(2)) {", quiescent)
        self.assertNotIn("command != 0", quiescent)

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
            "WLAN_PT_CARVEOUT_SIZE 0x10000ULL",
            "WLAN_PT_ALIGNMENT     0x4000ULL",
            "WLAN_DART_TLB_CMD              0x080",
            "WLAN_DART_TLB_CMD_FLUSH_SID1   0x101",
        ):
            self.assertIn(literal, SOURCE)
        abi = (ROOT / "src/wireless_handoff_abi.h").read_text()
        self.assertIn("WIRELESS_HANDOFF_V2_DESCRIPTOR_OFFSET 0xc000ULL", abi)
        self.assertIn("struct wireless_handoff_descriptor_v2", abi)
        self.assertIn("wireless_handoff_v2_descriptor_validate", SOURCE)

    def test_reservation_is_dynamic_and_above_guest_system_memory(self) -> None:
        self.assertNotIn("0x10022000000ULL", SOURCE)
        self.assertIn("wlan_validate_reservation", SOURCE)
        self.assertIn("cur_boot_args.phys_base + cur_boot_args.mem_size", SOURCE)
        # The physical-memory-top bound now lives in wlan_physical_memory_top();
        # tests/python/test_wireless_handoff_derivation.py pins the formula and
        # its agreement with Mu's own derivation.
        self.assertIn("wlan_physical_memory_top", SOURCE)
        self.assertIn(
            "ALIGN_DOWN(cur_boot_args.phys_base, BIT(32)) + mem_size_actual",
            SOURCE,
        )
        self.assertIn("base < guest_top + SZ_16K", SOURCE)
        self.assertIn("P_TOP_OF_MEMORY_ALLOC", PROXY_C)
        self.assertIn("top_of_memory_alloc(request->args[0])", PROXY_C)
        self.assertIn("memcpy((void *)boot_args_addr, &cur_boot_args", PROXY_C)

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


class PcieBusRoutingContractTests(unittest.TestCase):
    """Root-port bus numbers: the wireless profile's other hard requirement."""

    PCIE = (ROOT / "src" / "pcie.c").read_text(encoding="utf-8")

    def test_bridge_bus_numbers_are_programmed_after_link_up(self) -> None:
        self.assertIn("#define PCI_BRIDGE_BUS_NUMBER      0x18", self.PCIE)
        self.assertIn("static bool pcie_program_bridge_bus_numbers", self.PCIE)
        # Port N owns bus N+1, matching the ECAM device == port mapping that
        # config_base already relies on.
        self.assertIn("u32 secondary = port + 1;", self.PCIE)
        self.assertIn(
            "FIELD_PREP(PCI_BRIDGE_SECONDARY_BUS, secondary)", self.PCIE)
        self.assertIn(
            "FIELD_PREP(PCI_BRIDGE_SUBORDINATE_BUS, secondary)", self.PCIE)
        # Only on a port whose link actually came up.
        link_up = self.PCIE.index('printf("pcie: Port %d link up (status %#x)')
        call = self.PCIE.index("pcie_program_bridge_bus_numbers(config_base, port)")
        mask_set = self.PCIE.index("state->initialized_port_mask |= BIT(port);")
        self.assertLess(link_up, call)
        self.assertLess(call, mask_set)

    def test_routing_and_rails_share_one_opt_in(self) -> None:
        # Co-requisite: routing without power reaches a dead link, power
        # without routing reaches a device nothing can address.
        self.assertIn("static bool pcie_wireless_profile = false;", self.PCIE)
        self.assertNotIn("pcie_rails_enabled", self.PCIE)
        self.assertEqual(self.PCIE.count("pcie_wireless_profile = true;"), 1)
        wireless = self.PCIE[self.PCIE.index("int pcie_init_wireless(void)"):]
        self.assertIn("pcie_wireless_profile = true;", wireless)
        # The generic path must not opt in.
        generic = self.PCIE[
            self.PCIE.index("int pcie_init(void)"):
            self.PCIE.index("int pcie_init_wireless(void)")
        ]
        self.assertNotIn("pcie_wireless_profile", generic)
        for guarded in ("pcie_program_bridge_bus_numbers",
                        "pcie_enable_port_rail"):
            body = self.PCIE[self.PCIE.index(f"static bool {guarded}("):]
            self.assertIn("if (!pcie_wireless_profile)", body[:600], guarded)

    def test_adt_and_smc_helpers_stay_out_of_the_host_test_build(self) -> None:
        # tests/pcie compiles src/pcie.c with PCIE_T602X_WIRELESS_HOST_TEST;
        # an unguarded adt/smc reference breaks that entire suite.
        guard = "#ifndef PCIE_T602X_WIRELESS_HOST_TEST"
        end = "#endif /* !PCIE_T602X_WIRELESS_HOST_TEST */"
        self.assertEqual(self.PCIE.count(guard), 3)
        self.assertEqual(self.PCIE.count(end), 2)
        anchor = self.PCIE.index(
            "Everything from here to the matching #endif reads the ADT")
        opened = self.PCIE.index(guard, anchor)
        closed = self.PCIE.index(end, opened)
        region = self.PCIE[opened:closed]
        for helper in ("pcie_node_controls_bridge", "pcie_find_rail_owner",
                       "pcie_program_bridge_bus_numbers",
                       "pcie_enable_port_rail",
                       "static bool pcie_wireless_profile"):
            self.assertIn(helper, region, helper)
        # The callback-driven helpers the host tests exercise stay outside it.
        for host_testable in ("int pcie_port_release_perst",
                              "int pcie_port_start_link",
                              "void pcie_perst_delays_from_adt"):
            self.assertNotIn(host_testable, region, host_testable)

if __name__ == "__main__":
    unittest.main()
