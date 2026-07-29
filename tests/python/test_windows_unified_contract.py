from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class WindowsUnifiedContractTests(unittest.TestCase):
    def test_default_image_contains_safe_capabilities(self) -> None:
        config = (ROOT / "config.h").read_text(encoding="utf-8")
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        self.assertIn("#define ENABLE_NATIVE_AIC_PASSTHROUGH", config)
        self.assertIn("#define ENABLE_J414S_WINDOWS_MTP_HANDOFF", config)
        self.assertIn("#define ENABLE_J414S_WINDOWS_WIRELESS_HANDOFF", config)
        self.assertIn("#define ENABLE_J414S_WINDOWS_USB_HOST_HANDOFF", config)
        for obj in (
            "mtp_handoff.o",
            "wireless_handoff.o",
            "bcm4388_handoff.o",
            "hv_tpm.o",
            "kboot_gpu.o",
            "tps6598x_host_policy.o",
        ):
            self.assertIn(obj, makefile)

    def test_descriptor_handoff_core_is_linked_but_dormant(self) -> None:
        core = (ROOT / "src/bcm4388_handoff.c").read_text(encoding="utf-8")
        legacy_api = "bcm4388_legacy_dormant_handoff_install("
        self.assertIn(f"int {legacy_api}", core)
        for source in (ROOT / "src").glob("*.c"):
            if source.name != "bcm4388_handoff.c":
                self.assertNotIn(
                    legacy_api,
                    source.read_text(encoding="utf-8"),
                    source.name,
                )
        proxy = (ROOT / "proxyclient/m1n1/proxy.py").read_text(encoding="utf-8")
        self.assertNotIn("legacy_dormant_handoff", proxy)

    def test_mutating_capabilities_are_explicit(self) -> None:
        proxy = (ROOT / "proxyclient/m1n1/proxy.py").read_text(encoding="utf-8")
        wireless = (ROOT / "src/wireless_handoff.c").read_text(encoding="utf-8")
        self.assertIn("def wireless_handoff_init(self, reservation_base=None", proxy)
        self.assertIn("def top_of_memory_alloc(self, size)", proxy)
        self.assertNotIn("0x10022000000ULL", wireless)
        self.assertIn("base < guest_top + SZ_16K", wireless)
        self.assertIn("base + size > physical_top", wireless)

    def test_builder_is_commit_addressed_and_fail_closed(self) -> None:
        builder = (ROOT / "tools/build-j414s-windows-unified.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('BRANCH = "feature/j414s-windows-unified"', builder)
        self.assertIn('"schema": "ntasi.j414s.m1n1-unified.v1"', builder)
        self.assertIn("output_root = args.output.resolve() / commit", builder)
        self.assertIn('artifact_dir = output_root / "artifacts"', builder)
        self.assertIn('f"BUILD_DIR={build_dir}"', builder)
        self.assertIn("refusing a modified m1n1 source tree", builder)
        self.assertIn("BCM4388_DORMANT_ORIGIN", builder)
        self.assertIn(
            '"authoritative_wireless_contract": "dynamic_reserved_wireless_handoff_v2"',
            builder,
        )
        self.assertIn("legacy_reference_fixed_layout_no_current_abi_no_call_site", builder)
        self.assertIn('"mainline_snapshot"', builder)
        self.assertIn('USB_ROLE_SWAP_EXPERIMENT = "f17a15d1"', builder)
        self.assertIn('USB_INTERNAL_PHY_HANDOFF = "da86932a"', builder)
        self.assertIn(
            'J414S_ADT_SHA256 = "93d96b4a3ea736288278606b723f263361c6ae6c3d5c4f24f08f6f7a73f4b66e"',
            builder,
        )
        self.assertIn('"regression_recovery": True', builder)

    def test_xhc2_typec_policy_is_bounded_and_does_not_touch_proxy(self) -> None:
        hv = (ROOT / "src/hv.c").read_text(encoding="utf-8")
        usb = (ROOT / "src/usb.c").read_text(encoding="utf-8")
        tps = (ROOT / "src/tps6598x.c").read_text(encoding="utf-8")
        policy = (ROOT / "src/tps6598x_host_policy.c").read_text(encoding="utf-8")

        self.assertIn("platform_is_j414s()", hv)
        self.assertIn("usb_hpm_handoff_host(uartproxy_iodev)", hv)
        self.assertIn("(s32)idx == preserved_index", usb)
        self.assertIn('adt_getprop(adt, node, "rid"', usb)
        self.assertIn('adt_getprop(adt, node, "port-number"', usb)
        self.assertIn('adt_getprop(adt, node, "port-location"', usb)
        self.assertIn("J414S_USB_CONTROLLER_COUNT", usb)
        self.assertIn("TPS_REG_SYSTEM_CONFIG", tps)
        self.assertIn("System Configuration readback mismatch", tps)
        self.assertIn("timeout_expired(timeout)", tps)
        self.assertIn("TPS6598X_HOST_POLICY_ERR_ROLE", policy)
        self.assertNotIn('tps6598x_command(dev, "GAID"', tps)

        verifier = (ROOT / "tools/verify-j414s-usb-host-adt.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"hpm2": {"rid": 2, "port-number": 3, "port-location": "right"}', verifier)
        self.assertIn('hpm5.getprop("port-location") is not None', verifier)


if __name__ == "__main__":
    unittest.main()
