#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
ATC PHY (Type-C PHY) probe/driver experiment for T6020 (J414s / M2 Pro).

Drives the in-m1n1 ATCPHY driver (src/atcphy.c) over the proxy and dumps
decoded PHY state before/after. See docs/j414s-atcphy.md for the register
provenance and the safety rules; the short version:

  dump           read-only, safe at any time (default action)
  hpm            read-only STATUS from the HPM PD controller (orientation)
  tunables       read-only list of the port's ADT tunable_* blobs
  orient         programs USB2-mode crossbar/orientation; never touches the
                 PIPE mux; safe even while a guest OS owns the port
  usb3           FULL USB3 bring-up including the pipehandler PIPE-mux BIST
                 switch. Requires --allow-pipe-switch. NEVER do this while a
                 guest's xHCI driver is bound to this port (0x144 bugcheck
                 class risk); intended for proxy-only sessions.
  arm            arm USB3 mode for re-apply at guest handoff: booting a
                 guest re-parks the PIPE mux on DUMMY (usb_phy_handoff_host)
                 and would silently undo a proxy-applied usb3; arming makes
                 m1n1 re-apply USB3 right after that parking, before the
                 guest owns the port. Does not touch hardware by itself.
  disarm         cancel a previous arm
  off            atcphy_apply_mode(OFF): PHY stays powered, lanes parked
  power-off      upstream-faithful full power-down (recover-to-baseline)

Orientation: pass --flipped/--not-flipped to force, or the default 'auto'
reads it live from the HPM (right port = hpm2). Port default is 2 (the
J414s right-side port, the one broken under Windows).

Examples (proxy-only session, no guest running):
  M1N1DEVICE=/dev/cu.usbmodemXXX python3 atcphy_probe.py dump
  python3 atcphy_probe.py hpm
  python3 atcphy_probe.py orient          # auto orientation from hpm2
  python3 atcphy_probe.py usb3 --allow-pipe-switch
  python3 atcphy_probe.py power-off
"""

import argparse
import pathlib
import sys

sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("action", nargs="?", default="dump",
                    choices=["dump", "hpm", "tunables", "orient", "usb3",
                             "arm", "disarm", "off", "power-off"])
parser.add_argument("--port", type=int, default=2,
                    help="ATC port index (default 2 = J414s right side)")
parser.add_argument("--flipped", dest="flipped", action="store_true",
                    default=None, help="force flipped orientation")
parser.add_argument("--not-flipped", dest="flipped", action="store_false",
                    help="force normal orientation")
parser.add_argument("--allow-pipe-switch", action="store_true",
                    help="required for 'usb3'; see safety note above")
args = parser.parse_args()

if args.action == "usb3" and not args.allow_pipe_switch:
    parser.error("usb3 requires --allow-pipe-switch (read the safety note "
                 "in --help first)")

from m1n1.setup import *  # noqa: E402,F403
from m1n1.atcphy import (  # noqa: E402
    ATCPHY, ATCPHYMode, hpm_status, hpm_orientation,
)

phy = ATCPHY(u, args.port)
print(f"atcphy{args.port}: window cross-check vs in-m1n1 driver OK")


def resolve_orientation():
    if args.flipped is not None:
        return args.flipped
    present, flipped = hpm_orientation(u, args.port)
    print(f"hpm{args.port}: plug_present={present} upside_down={flipped}")
    if not present:
        print("WARNING: no plug present; programming 'normal' orientation")
    return flipped


if args.action == "dump":
    phy.dump()

elif args.action == "hpm":
    b0, decoded = hpm_status(u, args.port)
    print(f"hpm{args.port} STATUS byte0 = {b0:#04x}")
    for k, v in decoded.items():
        print(f"  {k:18s} = {v}")

elif args.action == "tunables":
    print(f"atcphy{args.port} ADT tunables:")
    phy.dump_tunables()

elif args.action == "arm":
    flipped = resolve_orientation()
    phy.arm_guest_mode(ATCPHYMode.USB3, flipped, armed=True)
    print(f"atcphy{args.port}: USB3 (flipped={flipped}) armed for guest "
          "handoff. Watch the m1n1 console for the 're-applying' line when "
          "the guest boots; nothing is written to the PHY until then.")

elif args.action == "disarm":
    phy.arm_guest_mode(ATCPHYMode.USB3, False, armed=False)
    print(f"atcphy{args.port}: guest mode disarmed")

elif args.action == "orient":
    flipped = resolve_orientation()
    print("--- before ---")
    phy.dump()
    phy.set_orientation(flipped)
    print("--- after ---")
    phy.dump()

elif args.action == "usb3":
    flipped = resolve_orientation()
    print("--- before ---")
    phy.dump()
    phy.apply_mode(ATCPHYMode.USB3, flipped, allow_pipe_switch=True)
    print("--- after ---")
    phy.dump()

elif args.action == "off":
    print("--- before ---")
    phy.dump()
    phy.apply_mode(ATCPHYMode.OFF)
    print("--- after ---")
    phy.dump()

elif args.action == "power-off":
    print("--- before ---")
    phy.dump()
    phy.power_off()
    print("--- after (NOTE: core-window reads may be stale/undefined with "
          "the domains asleep) ---")
    phy.dump()
