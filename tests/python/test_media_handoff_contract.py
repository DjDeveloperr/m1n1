# SPDX-License-Identifier: MIT
"""Static and ADT-driven contract checks for the J414s media profile.

Two things are pinned here.

1. The *source* contract of ``src/media_handoff.c``: that it is double-gated,
   that it writes nothing without an explicit flag, and that it does not do any
   of the things this machine cannot afford (touch a DART, lower a power
   domain, mask an interrupt, or drive a speaker path).

2. The *data* contract: the two DRAM audits the device side performs, replayed
   offline against the pinned live ADT capture so a layout change is caught on
   the host instead of on a boot that costs a power cycle.

The ADT-driven half skips cleanly when the private hardware-evidence capture is
not present, because that capture lives outside this repository.
"""

from pathlib import Path
import os
import re
import sys
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "proxyclient"))

from m1n1.media_handoff import (  # noqa: E402
    FLAG_ALL,
    FLAG_MCA_CLOCK_MUXES,
    FLAG_PROBE_GATED_DARTS,
    MediaHandoffError,
    aop_dram_segments_from_adt,
    check_isp_carveout,
    dram_extent,
    isp_carveout_from_adt,
    parse_segment_ranges,
)

SOURCE = (ROOT / "src" / "media_handoff.c").read_text(encoding="utf-8")
HEADER = (ROOT / "src" / "media_handoff.h").read_text(encoding="utf-8")
CONFIG = (ROOT / "config.h").read_text(encoding="utf-8")
HV = (ROOT / "src" / "hv.c").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.c").read_text(encoding="utf-8")
KBOOT = (ROOT / "src" / "kboot.c").read_text(encoding="utf-8")
PROXY_C = (ROOT / "src" / "proxy.c").read_text(encoding="utf-8")
PROXY_H = (ROOT / "src" / "proxy.h").read_text(encoding="utf-8")
PROXY_PY = (ROOT / "proxyclient" / "m1n1" / "proxy.py").read_text(encoding="utf-8")
MAKEFILE = (ROOT / "Makefile").read_text(encoding="utf-8")

# Measured live on this J414s through the proxy (u.ba) and corroborated by the
# m1n1 boot banner; identical to the values pinned in
# tests/python/test_wireless_handoff_derivation.py.
J414S_PHYS_BASE = 0x10001E40000
J414S_MEM_SIZE = 0x3D945C000
J414S_DRAM_BASE = 0x10000000000

# From the pinned ADT's /arm-io/isp0 segment-ranges: __TEXT 0x100009fc000+0x934000
# and __DATA 0x1000196c000+0x314000.
J414S_ISP_CARVEOUT = (0x100009FC000, 0x10001C80000)

ADT_CAPTURE = Path(
    os.path.expanduser(
        "~/Library/Application Support/ntasi/private-hardware-evidence/"
        "20260729-142731-j414s-ans-v5/j414s-adt.bin"
    )
)
ADT_SHA256 = "93d96b4a3ea736288278606b723f263361c6ae6c3d5c4f24f08f6f7a73f4b66e"


def _seg(phys, iova, remap, size):
    import struct

    return struct.pack("<QQQII", phys, iova, remap, size, 0)


class MediaHandoffGateTests(unittest.TestCase):
    def test_double_gated_on_the_windows_profile_and_the_exact_board(self):
        self.assertIn("#define ENABLE_J414S_WINDOWS_MEDIA_HANDOFF", CONFIG)
        self.assertIn(
            "defined(ENABLE_NATIVE_AIC_PASSTHROUGH) && "
            "defined(ENABLE_J414S_WINDOWS_MEDIA_HANDOFF)",
            SOURCE,
        )
        self.assertIn("if (!platform_is_j414s())", SOURCE)
        self.assertIn("media_handoff.o", MAKEFILE)

    def test_there_is_no_automatic_call_site_anywhere(self):
        """Compiling the capability must not be permission to use it.

        The wireless handoff established this: the only caller is the explicit
        proxy request.  If media_handoff_init() ever appears in hv.c, main.c or
        kboot.c, a plain boot would start touching audio and camera hardware.
        """
        for name, text in (("hv.c", HV), ("main.c", MAIN), ("kboot.c", KBOOT)):
            self.assertNotIn("media_handoff_init", text, name)
        self.assertIn("media_handoff_init((u32)request->args[0])", PROXY_C)
        self.assertIn("P_MEDIA_HANDOFF_INIT = 0x1600", PROXY_H)
        self.assertIn("P_MEDIA_HANDOFF_INIT = 0x1600", PROXY_PY)
        self.assertIn("def media_handoff_init(self, flags=0)", PROXY_PY)

    def test_flag_bits_agree_between_c_and_python(self):
        self.assertIn("#define MEDIA_HANDOFF_FLAG_PROBE_GATED_DARTS BIT(0)", HEADER)
        self.assertIn("#define MEDIA_HANDOFF_FLAG_MCA_CLOCK_MUXES BIT(1)", HEADER)
        self.assertEqual(FLAG_PROBE_GATED_DARTS, 1 << 0)
        self.assertEqual(FLAG_MCA_CLOCK_MUXES, 1 << 1)
        self.assertEqual(FLAG_ALL, 0b11)
        self.assertIn("if (flags & ~(u32)MEDIA_HANDOFF_FLAG_ALL)", SOURCE)

    def test_every_write_is_behind_a_flag(self):
        """flags == 0 must be a pure census.

        write32/mask32 appear exactly once between them, inside the clock-mux
        helper, and that helper has exactly one caller which is flag-guarded.
        """
        self.assertEqual(SOURCE.count("write32("), 0)
        self.assertEqual(SOURCE.count("write64("), 0)
        self.assertEqual(SOURCE.count("mask32("), 1)
        self.assertEqual(SOURCE.count("set32("), 0)
        self.assertEqual(SOURCE.count("clear32("), 0)
        muxes = SOURCE.index("static int media_set_mca_clock_muxes")
        self.assertGreater(SOURCE.index("mask32("), muxes)
        self.assertIn(
            "if (flags & MEDIA_HANDOFF_FLAG_MCA_CLOCK_MUXES) {", SOURCE
        )
        # Exactly one definition and exactly one (flag-guarded) call.
        self.assertEqual(SOURCE.count("media_set_mca_clock_muxes(void)"), 1)
        self.assertEqual(SOURCE.count("media_set_mca_clock_muxes()"), 1)


class MediaHandoffSafetyTests(unittest.TestCase):
    def test_no_dart_programming_and_no_power_lowering(self):
        """The three things that would break the devices being measured."""
        for forbidden in (
            "dart_init_adt",
            "dart_map",
            "dart_shutdown",
            "DART_T8110_TTBR_OFF",
            "pmgr_adt_power_disable",
            "pmgr_power_disable",
            "pmgr_set_mode",
            "PMGR_PS_PWRGATE",
            "PMGR_PS_CLKGATE",
            "dapf_init(",
        ):
            self.assertNotIn(forbidden, SOURCE, forbidden)
        self.assertIn("pmgr_adt_power_enable(path)", SOURCE)

    def test_the_include_list_is_the_contract(self):
        """dart.h/dapf.h/aic.h/usb.h/pcie.h/nvme.h must all be unreachable."""
        self.assertEqual(
            re.findall(r'#include "([^"]+)"', SOURCE),
            [
                "../config.h",
                "adt.h",
                "media_handoff.h",
                "platform_identity.h",
                "pmgr.h",
                "types.h",
                "utils.h",
                "xnuboot.h",
            ],
        )

    def test_no_interrupt_work_at_all(self):
        """The media ADT nodes' AIC lines must reach the guest unmasked.

        Nothing here may mask, unmask, retarget or software-trigger an AIC line.
        The lines the media profile publishes (569, 613-616, 628, 631 directly;
        1211/1213/1218/1221/1231 through CSRT aliases) are left exactly as
        firmware left them.
        """
        for forbidden in ("aic_set_mask", "aic_set_sw", "aic_write", "MASK_SET", "INTMSK"):
            self.assertNotIn(forbidden, SOURCE, forbidden)
        self.assertNotIn("#include \"aic.h\"", SOURCE)

    def test_speakers_can_never_be_driven(self):
        """The one write must not be able to reach an amplifier.

        The mux select is bounded to /arm-io/mca-switch reg[2], which is six
        PMGR clock-mux words.  i2c1/i2c3 (where the SN012776 amps live) and the
        MCA serialisers are never opened, and no ps_mcaN is ever raised -- in
        particular not ps_mca3, which AppleMcaAudio documents must never be
        raised because there is no fourth cluster.
        """
        self.assertIn("J414S_MCA_CLKMUX_BASE   0x28e03807cULL", SOURCE)
        self.assertIn("J414S_MCA_CLKMUX_SIZE   0x18ULL", SOURCE)
        self.assertIn("if (base != J414S_MCA_CLKMUX_BASE)", SOURCE)
        self.assertIn("u32 count = J414S_MCA_CLKMUX_SIZE / 4;", SOURCE)
        self.assertIn("MCA_CLK_MUX", SOURCE)
        # No PMGR path for any MCA cluster or amplifier bus is ever powered.
        for path in (
            '"/arm-io/mca0"',
            '"/arm-io/mca2"',
            '"/arm-io/i2c1"',
            '"/arm-io/i2c2"',
            '"/arm-io/i2c3"',
        ):
            enable = f"pmgr_adt_power_enable({path})"
            self.assertNotIn(enable, SOURCE)
        self.assertIn("presence_only", SOURCE)

    def test_boot_critical_devices_are_untouched(self):
        """No USB, ANS/NVMe or PCIe code or ADT path is reachable from here.

        Checked as code tokens, not as words: the prose does mention the USB
        DARTs' bypass hazard and pci.sys, and it should.
        """
        for forbidden in (
            "/arm-io/dart-usb",
            "/arm-io/dart-apcie",
            "/arm-io/usb-drd",
            "/arm-io/ans",
            "/arm-io/apcie",
            "usb_",
            "nvme_",
            "pcie_",
            "xhci",
            "#include \"usb.h\"",
            "#include \"nvme.h\"",
            "#include \"pcie.h\"",
        ):
            self.assertNotIn(forbidden, SOURCE, forbidden)
        # The only ADT paths this file can name.
        paths = sorted(set(re.findall(r'"(/arm-io/[a-z0-9\-]+)"', SOURCE)))
        self.assertEqual(
            paths,
            [
                "/arm-io/admac-aop-audio",
                "/arm-io/admac-sio",
                "/arm-io/aop",
                "/arm-io/dart-aop",
                "/arm-io/dart-isp0",
                "/arm-io/dart-sio",
                "/arm-io/i2c1",
                "/arm-io/i2c2",
                "/arm-io/i2c3",
                "/arm-io/isp0",
                "/arm-io/mca-switch",
                "/arm-io/mca0",
                "/arm-io/mca2",
                "/arm-io/nco",
            ],
        )

    def test_gated_dart_probe_never_lowers_the_gate(self):
        """Power-gating a DART discards TTBR/TCR, and AppleIsp adopts one."""
        self.assertIn("deliberately NOT lower", SOURCE)
        self.assertIn("gates are left RAISED", SOURCE)


class MediaHandoffLayoutTests(unittest.TestCase):
    def test_pinned_addresses_match_the_measured_hardware(self):
        for literal in (
            "J414S_MCA_SWITCH_BASE   0x39b600000ULL",
            "J414S_MCA_CLUSTER_BASE  0x39b500000ULL",
            "J414S_NCO_BASE          0x28e03c000ULL",
            "J414S_ADMAC_SIO_BASE    0x39b400000ULL",
            "J414S_DART_SIO_BASE     0x39b008000ULL",
            "J414S_AOP_ASC_BASE      0x2a6400000ULL",
            "J414S_DART_AOP_BASE     0x2a6808000ULL",
            "J414S_ADMAC_AOP_BASE    0x2a6980000ULL",
            "J414S_ISP_COPROC_BASE   0x384000000ULL",
            "J414S_ISP_PMGR_BASE     0x290280000ULL",
            "J414S_DART_ISP0_BASE    0x3860e8000ULL",
            "J414S_AOP_SRAM_BASE 0x2a6c00000ULL",
        ):
            self.assertIn(literal, SOURCE, literal)

    def test_stream_numbers_come_from_the_adt_mappers(self):
        self.assertIn("#define SIO_DART_SID_ADMAC 2", SOURCE)
        self.assertIn("#define AOP_DART_SID_CORE  0", SOURCE)
        self.assertIn("#define AOP_DART_SID_ADMAC 10", SOURCE)
        self.assertIn("#define ISP_DART_SID_MAIN  0", SOURCE)

    def test_aop_sram_is_pinned_because_adt_get_reg_mistranslates_it(self):
        """reg[2] is already absolute; translating it yields a bogus address.

        This is the same class of trap as the MTP fixed-buffer window, and the
        wrong answer (0x4a6c00000) must never appear in the source.
        """
        self.assertIn("0x4a6c00000", SOURCE)  # named only as the wrong answer
        self.assertIn("adt_get_reg() dutifully \"translates\" it", SOURCE)
        self.assertNotIn("adt_get_reg(adt, path, \"reg\", 2, &sram", SOURCE)


class MediaHandoffSegmentAuditTests(unittest.TestCase):
    def test_parse_segment_ranges_rejects_a_ragged_blob(self):
        with self.assertRaises(MediaHandoffError):
            parse_segment_ranges(b"\x00" * 31)
        with self.assertRaises(MediaHandoffError):
            parse_segment_ranges(b"")

    def test_sram_segments_do_not_widen_the_dram_extent(self):
        """The AOP's on-chip text must not be mistaken for allocatable DRAM."""
        segs = parse_segment_ranges(
            _seg(0x2A6C00000, 0x1000000, 0x2A6C00000, 0x8B000)
            + _seg(0x10000834000, 0x112E000, 0x10000000000, 0x14000)
        )
        self.assertEqual(
            dram_extent(segs, J414S_DRAM_BASE), (0x10000834000, 0x10000848000)
        )

    def test_a_node_with_only_sram_segments_has_no_dram_extent(self):
        segs = parse_segment_ranges(_seg(0x2A6C00000, 0x1000000, 0x2A6C00000, 0x8B000))
        self.assertIsNone(dram_extent(segs, J414S_DRAM_BASE))

    def test_the_isp_carveout_clears_the_guest_window_by_1_75_MiB(self):
        margin = check_isp_carveout(
            J414S_ISP_CARVEOUT, J414S_PHYS_BASE, J414S_MEM_SIZE
        )
        self.assertEqual(margin, 0x1C0000)

    def test_an_overlapping_isp_carveout_is_refused(self):
        """The invariant is a layout accident; if it ever breaks, fail loud."""
        with self.assertRaises(MediaHandoffError):
            check_isp_carveout(
                J414S_ISP_CARVEOUT, J414S_ISP_CARVEOUT[0], J414S_MEM_SIZE
            )
        # A carveout that starts one page above phys_base is inside the window.
        with self.assertRaises(MediaHandoffError):
            check_isp_carveout(
                (J414S_PHYS_BASE + 0x4000, J414S_PHYS_BASE + 0x8000),
                J414S_PHYS_BASE,
                J414S_MEM_SIZE,
            )


@unittest.skipUnless(ADT_CAPTURE.exists(), f"no pinned ADT capture at {ADT_CAPTURE}")
class MediaHandoffPinnedAdtTests(unittest.TestCase):
    """Replay the device-side audits against the real captured ADT."""

    @classmethod
    def setUpClass(cls):
        import hashlib

        from m1n1.adt import load_adt

        blob = ADT_CAPTURE.read_bytes()
        assert hashlib.sha256(blob).hexdigest() == ADT_SHA256, "ADT capture changed"
        cls.adt = load_adt(blob)

    def test_isp_carveout_extent_matches_the_pinned_value(self):
        self.assertEqual(
            isp_carveout_from_adt(self.adt, J414S_DRAM_BASE), J414S_ISP_CARVEOUT
        )

    def test_isp_carveout_clears_the_measured_boot_args_window(self):
        carveout = isp_carveout_from_adt(self.adt, J414S_DRAM_BASE)
        self.assertEqual(
            check_isp_carveout(carveout, J414S_PHYS_BASE, J414S_MEM_SIZE), 0x1C0000
        )

    def test_the_aop_os_log_segment_lands_inside_allocatable_dram(self):
        """Recorded, not enforced -- see media_audit_aop_segments().

        The AOP is running and one of its DRAM segments sits inside the window
        m1n1 allocates from.  Every boot on this machine has already run that
        way, so this is an observation to act on deliberately, not a reason to
        fail a handoff.  The assertion exists so the day it changes, it is
        noticed here first.
        """
        extent = aop_dram_segments_from_adt(self.adt, J414S_DRAM_BASE)
        self.assertIsNotNone(extent)
        lo, hi = extent
        self.assertEqual(lo, 0x10000834000)
        self.assertGreater(hi, J414S_PHYS_BASE)
        self.assertIn("WARNING: AOP DRAM segments", SOURCE)

    def test_media_reg_windows_match_the_pinned_adt(self):
        expected = {
            ("/arm-io/mca-switch", 0): 0x39B600000,
            ("/arm-io/mca-switch", 1): 0x39B500000,
            ("/arm-io/mca-switch", 2): 0x28E03807C,
            ("/arm-io/nco", 0): 0x28E03C000,
            ("/arm-io/admac-sio", 0): 0x39B400000,
            ("/arm-io/dart-sio", 0): 0x39B008000,
            ("/arm-io/aop", 0): 0x2A6400000,
            ("/arm-io/aop", 1): 0x2A6050000,
            ("/arm-io/dart-aop", 0): 0x2A6808000,
            ("/arm-io/admac-aop-audio", 0): 0x2A6980000,
            ("/arm-io/isp0", 0): 0x384000000,
            ("/arm-io/isp0", 1): 0x290280000,
            ("/arm-io/dart-isp0", 0): 0x3860E8000,
        }
        for (path, index), base in expected.items():
            self.assertEqual(self.adt[path].get_reg(index)[0], base, f"{path}[{index}]")

    def test_dart_stream_numbers_come_from_the_iommu_mappers(self):
        self.assertEqual(self.adt["/arm-io/dart-sio/mapper-admac"].reg, 2)
        self.assertEqual(self.adt["/arm-io/dart-aop/mapper-aop"].reg, 0)
        self.assertEqual(self.adt["/arm-io/dart-aop/mapper-aop-admac"].reg, 10)
        self.assertEqual(self.adt["/arm-io/dart-isp0/mapper-isp0"].reg, 0)

    def test_the_aop_node_has_no_power_domain(self):
        """AppleAopAudio states there is no ps_aop; the ADT agrees."""
        aop = self.adt["/arm-io/aop"]
        self.assertIsNone(aop.clock_gates)
        self.assertIsNone(aop.power_gates)

    def test_media_nodes_keep_their_interrupts_for_the_guest(self):
        """m1n1 hands the guest this same ADT; the lines must survive.

        setup_adt() deletes only proxy-USB/ATC and CPU nodes and load_raw()
        edits only __OS_LOG out of segment-ranges, so every media node's
        `interrupts` property reaches Mu untouched.
        """
        expected = {
            "/arm-io/isp0": 569,
            "/arm-io/dart-aop": 628,
            "/arm-io/admac-aop-audio": 631,
            "/arm-io/mca0": 1211,
            "/arm-io/mca2": 1213,
            "/arm-io/admac-sio": 1218,
            "/arm-io/i2c2": 1221,
            "/arm-io/dart-sio": 1231,
        }
        for path, first in expected.items():
            self.assertEqual(self.adt[path].interrupts[0], first, path)
        self.assertEqual(list(self.adt["/arm-io/aop"].interrupts), [614, 613, 616, 615])

    def test_aop_sram_literal_matches_the_first_firmware_segment(self):
        segs = parse_segment_ranges(
            self.adt["/arm-io/aop"]._properties["segment-ranges"]
        )
        self.assertEqual(segs[0]["phys"], 0x2A6C00000)
        self.assertEqual(segs[0]["remap"], segs[0]["phys"])
        self.assertLessEqual(
            segs[1]["phys"] + segs[1]["size"], 0x2A6C00000 + 0x250000
        )


if __name__ == "__main__":
    unittest.main()
