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
        for obj in ("mtp_handoff.o", "wireless_handoff.o", "hv_tpm.o", "kboot_gpu.o"):
            self.assertIn(obj, makefile)

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


if __name__ == "__main__":
    unittest.main()
