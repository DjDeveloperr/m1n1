# SPDX-License-Identifier: MIT
"""J414s BCM4388 wireless DART handoff (ABI v2): reservation derivation.

This module is the single host-side source of truth for the ONE physical
address that m1n1 and Mu must agree on without ever talking to each other.

Why a derivation and not a message
----------------------------------
m1n1 installs the SID-1 deny-all domain at EL2 *before* Mu runs, because
``pci.sys`` enables endpoint bus mastering before any KMDF provider can start.
So Mu cannot hand m1n1 a base (it does not exist yet) and m1n1 cannot hand Mu
one (there is no channel; Mu's ``PcdAppleWirelessDartPageTableBase`` is derived
live in ``MemoryInitPeiLib``).  Both sides instead compute the same address
from the same ``boot_args`` inputs::

    phys_top = ALIGN_DOWN(boot_args.phys_base, 4 GiB) + boot_args.mem_size_actual
    base     = ALIGN_DOWN(phys_top - 0x10000, 0x4000)

Mu's implementation is ``NtasiDeriveWirelessReservation()`` in
``mu-j414s-windows-unified``,
``Silicon/Apple/T602XFamilyPkg/Library/MemoryInitPeiLib/MemoryInitPeiLib.c``::

    PhysTop       = (SystemMemoryBase & ~(SIZE_4GB - 1)) + MemSizeActual;
    CandidateBase = (PhysTop - NTASI_WIRELESS_HANDOFF_V2_RESERVATION_SIZE) &
                    ~(NTASI_WIRELESS_HANDOFF_V2_PAGE_SIZE - 1);

with the guards reproduced verbatim in :func:`derive_reservation` below.
``SystemMemoryBase``/``SystemMemorySize`` come straight out of the ``boot_args``
struct m1n1 hands the guest -- ``EarlySetup()`` in
``Silicon/Apple/AppleSiliconPkg/PrePi/AdtParser.c`` sets
``*SystemMemoryBase = BootArgs->phys_base`` and
``*SystemMemorySize = BootArgs->mem_size`` -- and ``MemSizeActual`` is read
from the same struct via the revision union.

The disagreement hazard, stated plainly
---------------------------------------
The two sides do NOT read the same ``boot_args`` struct:

* m1n1's device-side ``wlan_validate_reservation()`` uses its own
  ``cur_boot_args`` (``phys_base`` as iBoot supplied it).
* Mu reads the *guest* ``boot_args`` the hypervisor synthesised.
  ``HV.load_raw()`` overwrites ``tba.phys_base`` with ``u.heap_top`` and
  ``tba.mem_size`` with ``u.ba.phys_base + u.ba.mem_size - heap_top``.

The derived base is identical only because ``ALIGN_DOWN(x, 4 GiB)`` collapses
both ``phys_base`` values onto the same ``ram_base``.  On J414s with a 768 MiB
proxy heap that holds with ~3 GiB to spare, but it is a property of the current
layout, not an invariant: a large enough ``--proxy-heap-size`` would push
``heap_top`` across a 4 GiB boundary and the two sides would silently pick
different addresses.  :func:`derive_agreed_reservation` therefore computes the
derivation from *both* structs and raises rather than choosing one.  m1n1's C
side independently refuses any base that is not its own canonical derivation,
so a disagreement can only ever produce a loud refusal, never a handoff
installed where Mu will not look.
"""

RESERVATION_SIZE = 0x10000
PAGE_SIZE = 0x4000
#: Guard the reservation must sit above the reduced boot_args top (m1n1's
#: SZ_16K in wlan_validate_reservation(), Mu's SIZE_16KB).
GUARD_SIZE = 0x4000
FOUR_GIB = 1 << 32
#: Mu rejects a mem_size_actual above this as unusable.
MAX_MEM_SIZE_ACTUAL = 1 << 40

DESCRIPTOR_OFFSET = 0xC000
DESCRIPTOR_SIZE = 96
DESCRIPTOR_FORMAT = "<IHHIHHQQQQQQQQIIII"
SIGNATURE = 0x3248574E  # "NWH2"
VERSION = 2
FLAG_INSTALLED = 1
SID = 1
PAGE_SHIFT = 14
DART_BASE = 0x594000000
L1_OFFSET = 0x0000
MSI_L2_OFFSET = 0x4000
CLIENT_L2_OFFSET = 0x8000

#: Byte offset of guest_memory_top inside the descriptor.
GUEST_MEMORY_TOP_OFFSET = 32

__all__ = [
    "WirelessHandoffDerivationError",
    "RESERVATION_SIZE",
    "PAGE_SIZE",
    "GUARD_SIZE",
    "DESCRIPTOR_OFFSET",
    "DESCRIPTOR_SIZE",
    "GUEST_MEMORY_TOP_OFFSET",
    "ram_base",
    "physical_memory_top",
    "derive_reservation",
    "derive_agreed_reservation",
    "boot_args_view",
]


class WirelessHandoffDerivationError(Exception):
    """The reservation cannot be derived, or the two sides disagree."""


def ram_base(phys_base):
    """ALIGN_DOWN(phys_base, 4 GiB) -- m1n1's ram_base, Mu's PhysTop anchor."""
    return phys_base & ~(FOUR_GIB - 1)


def physical_memory_top(phys_base, mem_size_actual):
    """Mu's ``PhysTop``; identical to m1n1's ``wlan_physical_memory_top()``."""
    return ram_base(phys_base) + mem_size_actual


def boot_args_view(ba, name):
    """Extract the three fields the derivation depends on from a BootArgs."""
    try:
        view = {
            "name": name,
            "phys_base": int(ba.phys_base),
            "mem_size": int(ba.mem_size),
            "mem_size_actual": int(ba.mem_size_actual),
        }
    except AttributeError as exc:
        raise WirelessHandoffDerivationError(
            f"{name} boot_args is missing a field the wireless handoff "
            f"derivation needs: {exc}"
        ) from exc
    return view


def derive_reservation(phys_base, mem_size, mem_size_actual, name="boot_args"):
    """Derive the reservation exactly the way Mu's PEI code does.

    Every guard below mirrors ``NtasiDeriveWirelessReservation()`` in the same
    order, and raises with the same reason Mu would log before withholding
    wireless.  Returns ``(base, size)``.
    """
    if mem_size_actual == 0 or mem_size_actual > MAX_MEM_SIZE_ACTUAL:
        raise WirelessHandoffDerivationError(
            f"{name}: mem_size_actual unusable ({mem_size_actual:#x})"
        )

    phys_top = physical_memory_top(phys_base, mem_size_actual)
    if phys_top <= RESERVATION_SIZE:
        raise WirelessHandoffDerivationError(
            f"{name}: computed PhysTop {phys_top:#x} is too small"
        )

    base = (phys_top - RESERVATION_SIZE) & ~(PAGE_SIZE - 1)

    guest_top = phys_base + mem_size
    guest_top_with_margin = guest_top + GUARD_SIZE
    if base < guest_top_with_margin:
        raise WirelessHandoffDerivationError(
            f"{name}: derived base {base:#x} is below guest_top+16KiB "
            f"{guest_top_with_margin:#x}"
        )
    if base & (PAGE_SIZE - 1):
        raise WirelessHandoffDerivationError(
            f"{name}: derived base {base:#x} is not 16 KiB aligned"
        )
    if base + RESERVATION_SIZE > phys_top:
        raise WirelessHandoffDerivationError(
            f"{name}: derived reservation {base:#x}+{RESERVATION_SIZE:#x} "
            f"exceeds PhysTop {phys_top:#x}"
        )

    return base, RESERVATION_SIZE


def derive_agreed_reservation(m1n1_view, mu_view):
    """Derive from both boot_args structs and require bit-for-bit agreement.

    ``m1n1_view``/``mu_view`` are mappings with ``phys_base``, ``mem_size``,
    ``mem_size_actual`` and ``name`` -- see :func:`boot_args_view`.  The first
    describes what m1n1's device-side validator will recompute, the second what
    Mu's PEI will recompute.

    Raises :class:`WirelessHandoffDerivationError` on any disagreement instead
    of picking a winner; the failure text names both inputs and both results.
    """
    m1n1_base, size = derive_reservation(
        m1n1_view["phys_base"],
        m1n1_view["mem_size"],
        m1n1_view["mem_size_actual"],
        m1n1_view.get("name", "m1n1"),
    )
    mu_base, mu_size = derive_reservation(
        mu_view["phys_base"],
        mu_view["mem_size"],
        mu_view["mem_size_actual"],
        mu_view.get("name", "Mu"),
    )

    if (m1n1_base, size) != (mu_base, mu_size):
        raise WirelessHandoffDerivationError(
            "wireless handoff derivations DISAGREE and the handoff cannot be "
            "installed: m1n1 would validate against "
            f"{m1n1_base:#x}+{size:#x} (phys_base "
            f"{m1n1_view['phys_base']:#x}, ram_base "
            f"{ram_base(m1n1_view['phys_base']):#x}, mem_size_actual "
            f"{m1n1_view['mem_size_actual']:#x}) while Mu would derive "
            f"{mu_base:#x}+{mu_size:#x} (phys_base "
            f"{mu_view['phys_base']:#x}, ram_base "
            f"{ram_base(mu_view['phys_base']):#x}, mem_size_actual "
            f"{mu_view['mem_size_actual']:#x}). The two boot_args structs land "
            "in different 4 GiB regions; reduce --proxy-heap-size or fix the "
            "hypervisor guest phys_base before booting."
        )

    m1n1_guest_top = m1n1_view["phys_base"] + m1n1_view["mem_size"]
    mu_guest_top = mu_view["phys_base"] + mu_view["mem_size"]
    if m1n1_guest_top != mu_guest_top:
        raise WirelessHandoffDerivationError(
            "wireless handoff guest_memory_top would not match: m1n1 will "
            f"stamp {m1n1_guest_top:#x} into the descriptor but Mu will "
            f"compute SystemMemoryTop = {mu_guest_top:#x} and reject it. "
            "Something reduced boot_args.mem_size between the hypervisor "
            "building the guest boot_args and the handoff being installed."
        )

    return {
        "base": m1n1_base,
        "size": size,
        "physical_memory_top": physical_memory_top(
            m1n1_view["phys_base"], m1n1_view["mem_size_actual"]
        ),
        "guest_memory_top": m1n1_guest_top,
        "l1_physical": m1n1_base + L1_OFFSET,
        "msi_l2_physical": m1n1_base + MSI_L2_OFFSET,
        "descriptor_physical": m1n1_base + DESCRIPTOR_OFFSET,
    }
