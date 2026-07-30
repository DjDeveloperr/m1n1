# SPDX-License-Identifier: MIT
"""The m1n1 <-> Mu wireless reservation derivation must agree bit for bit.

Nothing is exchanged between the two sides at runtime: m1n1 installs the SID-1
domain before Mu exists.  Both therefore run the same formula over the same
boot_args fields, and this file is the proof that they land on the same address.

The Mu side is reimplemented here from
``mu-j414s-windows-unified``,
``Silicon/Apple/T602XFamilyPkg/Library/MemoryInitPeiLib/MemoryInitPeiLib.c``
(``NtasiDeriveWirelessReservation``), deliberately transcribed from the C rather
than calling the shared helper, so that a future edit to
``proxyclient/m1n1/wireless_handoff.py`` that drifts away from Mu fails here
instead of on hardware.
"""

from pathlib import Path
import re
import sys
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "proxyclient"))

from m1n1.wireless_handoff import (  # noqa: E402
    GUARD_SIZE,
    PAGE_SIZE,
    RESERVATION_SIZE,
    WirelessHandoffDerivationError,
    derive_agreed_reservation,
    derive_reservation,
    physical_memory_top,
)

SOURCE = (ROOT / "src" / "wireless_handoff.c").read_text(encoding="utf-8")

SIZE_4GB = 1 << 32
SIZE_16KB = 0x4000
MAX_UINT64 = (1 << 64) - 1

# Measured live on the J414s target through the m1n1 proxy (u.ba), and
# corroborated by the m1n1 boot banner in the project's own host transcripts:
#   revision 2, phys_base 0x10001e40000, mem_size 0x3d945c000,
#   mem_size_actual 0x400000000, "MMU: RAM base: 0x10000000000",
#   "MMU: Top of normal RAM: 0x103db29c000".
J414S_M1N1_PHYS_BASE = 0x10001E40000
J414S_MEM_SIZE = 0x3D945C000
J414S_MEM_SIZE_ACTUAL = 0x400000000
J414S_GUEST_TOP = 0x103DB29C000
J414S_PHYS_TOP = 0x10400000000
J414S_EXPECTED_BASE = 0x103FFFF0000
# HV.load_raw() rewrites the guest phys_base to u.heap_top; this is the value
# observed with the launcher's proxy heap ("Physical memory: 0x1003e660000 ..").
J414S_GUEST_PHYS_BASE = 0x1003E660000


def mu_derive_wireless_reservation(system_memory_base, system_memory_top,
                                   mem_size_actual):
    """Transcription of Mu's NtasiDeriveWirelessReservation().

    Returns ``(base, size)`` on success or ``(0, 0)`` where Mu returns FALSE
    and withholds wireless.
    """
    if mem_size_actual == 0 or mem_size_actual > (1 << 40):
        return 0, 0

    phys_top = (system_memory_base & ~(SIZE_4GB - 1)) + mem_size_actual
    if phys_top <= RESERVATION_SIZE:
        return 0, 0

    candidate_base = (phys_top - RESERVATION_SIZE) & ~(PAGE_SIZE - 1)
    guest_top_with_margin = system_memory_top + SIZE_16KB

    if candidate_base < guest_top_with_margin:
        return 0, 0
    if candidate_base & (PAGE_SIZE - 1):
        return 0, 0
    if candidate_base + RESERVATION_SIZE > phys_top:
        return 0, 0

    return candidate_base, RESERVATION_SIZE


def m1n1_validate_reservation(base, size, phys_base, mem_size,
                              mem_size_actual):
    """Transcription of m1n1's wlan_validate_reservation() numeric predicate.

    Excludes the ADT/MCC claim check, which has no host-side inputs.
    """
    phys_top = (phys_base & ~(SIZE_4GB - 1)) + mem_size_actual
    guest_top = phys_base + mem_size

    if size != RESERVATION_SIZE:
        return False
    if base & (PAGE_SIZE - 1):
        return False
    if base > MAX_UINT64 - size:
        return False
    if base < guest_top + SIZE_16KB or base + size > phys_top:
        return False

    canonical = 0
    if phys_top > RESERVATION_SIZE:
        canonical = (phys_top - RESERVATION_SIZE) & ~(PAGE_SIZE - 1)
    return canonical != 0 and base == canonical


# (name, m1n1 phys_base, mem_size, mem_size_actual, guest phys_base)
PLAUSIBLE_CASES = (
    ("J414s measured 16 GiB", J414S_M1N1_PHYS_BASE, J414S_MEM_SIZE,
     J414S_MEM_SIZE_ACTUAL, J414S_GUEST_PHYS_BASE),
    ("8 GiB", 0x10001E40000, 0x1D945C000, 0x200000000, 0x1003E660000),
    ("16 GiB, larger m1n1 footprint", 0x10002000000, 0x3D0000000,
     0x400000000, 0x100A0000000),
    ("24 GiB", 0x10001E40000, 0x5D945C000, 0x600000000, 0x1003E660000),
    ("32 GiB", 0x10001E40000, 0x7D945C000, 0x800000000, 0x1003E660000),
    ("64 GiB", 0x10001E40000, 0xFD945C000, 0x1000000000, 0x1003E660000),
    ("96 GiB", 0x10001E40000, 0x17D945C000, 0x1800000000, 0x1003E660000),
    # mem_size_actual not a multiple of the DART granule: the ALIGN_DOWN in
    # both implementations has to do real work here.
    ("unaligned mem_size_actual", 0x10001E40000, 0x3D945C000,
     0x400000000 - 0x1000, 0x1003E660000),
)


class DerivationAgreementTests(unittest.TestCase):
    def test_j414s_measured_case(self):
        """The real machine: mem_size_actual = 0x400000000 (16 GiB)."""
        self.assertEqual(
            physical_memory_top(J414S_M1N1_PHYS_BASE, J414S_MEM_SIZE_ACTUAL),
            J414S_PHYS_TOP,
        )
        self.assertEqual(
            J414S_M1N1_PHYS_BASE + J414S_MEM_SIZE, J414S_GUEST_TOP)

        base, size = derive_reservation(
            J414S_M1N1_PHYS_BASE, J414S_MEM_SIZE, J414S_MEM_SIZE_ACTUAL)
        self.assertEqual((base, size), (J414S_EXPECTED_BASE, RESERVATION_SIZE))

        mu_base, mu_size = mu_derive_wireless_reservation(
            J414S_GUEST_PHYS_BASE,
            # Mu's SystemMemoryTop == the hypervisor's guest top, which
            # HV.load_raw() keeps equal to m1n1's own boot_args top.
            J414S_GUEST_TOP,
            J414S_MEM_SIZE_ACTUAL,
        )
        self.assertEqual((mu_base, mu_size), (base, size))
        self.assertTrue(m1n1_validate_reservation(
            base, size, J414S_M1N1_PHYS_BASE, J414S_MEM_SIZE,
            J414S_MEM_SIZE_ACTUAL))

    def test_all_three_implementations_agree(self):
        for name, phys_base, mem_size, actual, guest_phys_base in \
                PLAUSIBLE_CASES:
            with self.subTest(case=name):
                base, size = derive_reservation(phys_base, mem_size, actual,
                                                name)
                guest_top = phys_base + mem_size
                mu_base, mu_size = mu_derive_wireless_reservation(
                    guest_phys_base, guest_top, actual)
                self.assertEqual(
                    (mu_base, mu_size), (base, size),
                    f"{name}: Mu derives {mu_base:#x}, shared helper "
                    f"derives {base:#x}")
                self.assertTrue(
                    m1n1_validate_reservation(base, size, phys_base, mem_size,
                                              actual),
                    f"{name}: m1n1 would reject its own derivation")
                self.assertEqual(base % PAGE_SIZE, 0)
                self.assertEqual(size, RESERVATION_SIZE)
                self.assertGreaterEqual(base, guest_top + GUARD_SIZE)
                self.assertLessEqual(
                    base + size, physical_memory_top(phys_base, actual))

    def test_agreed_helper_accepts_the_live_pair(self):
        agreed = derive_agreed_reservation(
            {
                "name": "m1n1",
                "phys_base": J414S_M1N1_PHYS_BASE,
                "mem_size": J414S_MEM_SIZE,
                "mem_size_actual": J414S_MEM_SIZE_ACTUAL,
            },
            {
                "name": "Mu",
                "phys_base": J414S_GUEST_PHYS_BASE,
                "mem_size": J414S_GUEST_TOP - J414S_GUEST_PHYS_BASE,
                "mem_size_actual": J414S_MEM_SIZE_ACTUAL,
            },
        )
        self.assertEqual(agreed["base"], J414S_EXPECTED_BASE)
        self.assertEqual(agreed["size"], RESERVATION_SIZE)
        self.assertEqual(agreed["guest_memory_top"], J414S_GUEST_TOP)
        self.assertEqual(agreed["physical_memory_top"], J414S_PHYS_TOP)
        self.assertEqual(agreed["l1_physical"], J414S_EXPECTED_BASE)
        self.assertEqual(agreed["msi_l2_physical"],
                         J414S_EXPECTED_BASE + 0x4000)
        self.assertEqual(agreed["descriptor_physical"],
                         J414S_EXPECTED_BASE + 0xC000)

    def test_a_4gib_straddling_guest_phys_base_is_loud(self):
        """The one way the two sides can disagree must raise, not paper over."""
        # A proxy heap large enough to push the guest phys_base past the next
        # 4 GiB boundary makes Mu's ram_base differ from m1n1's.
        straddling = 0x10100000000
        with self.assertRaises(WirelessHandoffDerivationError) as caught:
            derive_agreed_reservation(
                {
                    "name": "m1n1",
                    "phys_base": J414S_M1N1_PHYS_BASE,
                    "mem_size": J414S_MEM_SIZE,
                    "mem_size_actual": J414S_MEM_SIZE_ACTUAL,
                },
                {
                    "name": "Mu",
                    "phys_base": straddling,
                    "mem_size": J414S_GUEST_TOP - straddling,
                    "mem_size_actual": J414S_MEM_SIZE_ACTUAL,
                },
            )
        self.assertIn("DISAGREE", str(caught.exception))

    def test_guest_memory_top_drift_is_loud(self):
        with self.assertRaises(WirelessHandoffDerivationError) as caught:
            derive_agreed_reservation(
                {
                    "name": "m1n1",
                    "phys_base": J414S_M1N1_PHYS_BASE,
                    "mem_size": J414S_MEM_SIZE,
                    "mem_size_actual": J414S_MEM_SIZE_ACTUAL,
                },
                {
                    "name": "Mu",
                    "phys_base": J414S_GUEST_PHYS_BASE,
                    # One extra top_of_memory_alloc() after the guest boot_args
                    # were built.
                    "mem_size": J414S_GUEST_TOP - J414S_GUEST_PHYS_BASE
                    - 0x4000,
                    "mem_size_actual": J414S_MEM_SIZE_ACTUAL,
                },
            )
        self.assertIn("guest_memory_top", str(caught.exception))

    def test_unusable_inputs_are_refused(self):
        for phys_base, mem_size, actual in (
            (J414S_M1N1_PHYS_BASE, J414S_MEM_SIZE, 0),
            (J414S_M1N1_PHYS_BASE, J414S_MEM_SIZE, (1 << 40) + 1),
            # mem_size covering all of mem_size_actual leaves no headroom.
            (0x10000000000, 0x400000000, 0x400000000),
        ):
            with self.subTest(actual=actual, mem_size=mem_size):
                with self.assertRaises(WirelessHandoffDerivationError):
                    derive_reservation(phys_base, mem_size, actual)
                self.assertEqual(
                    mu_derive_wireless_reservation(
                        phys_base, phys_base + mem_size, actual),
                    (0, 0),
                )


class ProducerContractTests(unittest.TestCase):
    """The device-side C must keep the properties this test relies on."""

    def test_c_side_refuses_a_non_canonical_base(self):
        self.assertIn("wlan_canonical_reservation_base", SOURCE)
        self.assertIn("WLAN_ERR_RESERVATION_NOT_CANONICAL", SOURCE)
        self.assertIn("base != canonical", SOURCE)
        # The formula itself, so a silent edit to either operand fails here.
        self.assertIsNotNone(re.search(
            r"ALIGN_DOWN\(physical_top - WLAN_PT_CARVEOUT_SIZE,\s*"
            r"WLAN_PT_ALIGNMENT\)", SOURCE))
        self.assertIsNotNone(re.search(
            r"ALIGN_DOWN\(cur_boot_args\.phys_base,\s*BIT\(32\)\)\s*\+\s*"
            r"mem_size_actual", SOURCE))

    def test_c_side_cleans_the_reservation_to_dram(self):
        self.assertIn("dc_civac_range", SOURCE)
        self.assertIn('sysop("dsb sy")', SOURCE)
        build = SOURCE[SOURCE.index("static int wlan_build_tables"):
                       SOURCE.index("static int wlan_publish_descriptor")]
        self.assertIn(
            "wlan_publish_range(wlan_pt_carveout_phys, WLAN_PT_CARVEOUT_SIZE)",
            build)
        self.assertNotIn("dma_wmb", build)
        publish = SOURCE[SOURCE.index("static int wlan_publish_descriptor"):
                         SOURCE.index("static int wlan_flush_sid1")]
        self.assertIn("wlan_publish_range(", publish)
        self.assertNotIn("dma_wmb", publish)

    def test_c_side_refuses_firmware_claimed_ranges(self):
        self.assertIn("wlan_range_is_claimed", SOURCE)
        self.assertIn("WLAN_ERR_RESERVATION_CLAIMED", SOURCE)
        self.assertIn("mcc_carveouts", SOURCE)
        self.assertIn("pmap-io-ranges", SOURCE)


if __name__ == "__main__":
    unittest.main()
