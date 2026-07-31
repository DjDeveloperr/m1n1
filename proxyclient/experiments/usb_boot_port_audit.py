# SPDX-License-Identifier: MIT
"""
Read-only forensics for the J414s Windows BOOT port (usb-drd1 / XHC1 /
left-side USB-C, the port the Windows USB boot SSD hangs off).

MOTIVATION (2026-07-30 0x144 investigation): m1n1's guest handoff
(usb_phy_handoff_host, src/usb.c) resets the USB2 PHY of every non-console
port and parks its SuperSpeed PIPE mux on the DUMMY backend, so the Windows
boot disk runs USB 2.0 High-Speed for the whole session -- confirmed by the
live descriptor capture (bcdUSB=0x0210, MPS0=64, bulk MPS=512).  The
intermittent BUGCODE_USB3_DRIVER 0x144 is a High-Speed link that drops
under load and then fails re-enumeration (PORTSC: CCS=1, PED=0, PLS=7
Polling, port speed=Full).  This tool captures, without writing a single
register, everything needed to compare a "good" boot against a "bad" boot
and to decide whether the PHY/fabric is running untuned:

  snapshot(u)         one-shot dump: XHC1 op regs, both root ports
                      (PORTSC/PORTPMSC/PORTLI), DWC_USB31 globals
                      (GSTS/GUSB2PHYCFG/GDBGLTSSM/GBUSERRADDR), the port-1
                      USB2 PHY block, and the pipehandler mux state.
  sample(u, secs)     poll PORTSC/USBSTS/GSTS and print timestamped CHANGES
                      only -- catches transient link events (PLC/PEC/CSC)
                      that the guest driver clears before a human can look.
  audit_tunables(u)   for every ADT tunable_* blob on /arm-io/atc-phy1,
                      read the target registers live and report whether each
                      RMW record is currently SATISFIED ((reg & mask) ==
                      value).  Global-scope blobs (AXI2AF fabric bridge,
                      ATC fabric, ACIOPHY_TOP, CMN, PLLs) are the ones that
                      matter for USB2-only operation; USB3/CIO lane blobs
                      are expected to be unapplied while the SS mux is
                      parked on DUMMY and are reported separately.

Usage from the HV shell (guest running or held -- all reads side-effect
free):

    exec(open("/Users/dj/Developer/m1n1/proxyclient/experiments/"
              "usb_boot_port_audit.py").read())
    snapshot(u)
    audit_tunables(u)
    sample(u, 30)

or standalone against a bare proxy (no guest):  python3 usb_boot_port_audit.py

Register provenance: xHCI 1.2 sec 5.4 (operational/port registers; note
CRCR reads as 0 by spec 5.4.5 -- a crcr==0 read NEVER proves a reset);
DWC3/DWC_USB31 global block at +0xC100 with GSNPSID at +0xC120 (matches
the deployed forensics tool) and GDBGLTSSM at +0xC164; ATCPHY offsets from
m1n1 src/atcphy_core.c (itself from Asahi atc.c @ 030248d39b40 with live
ADT cross-checks, docs/j414s-atcphy.md sec 2).  Tunable wire format:
12-byte LE records {offset:24|size:8, mask:32, value:32}, size must be 32,
register address = block_base + vocab.block_offset + record.offset --
byte-for-byte the layout atcphy_tunable_validate() enforces.
"""

import struct
import time

# Hardware-anchored constants (grade A: live captures in the sibling repo's
# debug-snapshots).  The script re-derives every address from the live ADT
# and REFUSES to run if the derivation disagrees with these anchors -- this
# project's standing "verify decodes, don't trust them" rule.
BOOT_PORT = 1
KNOWN_DRD1_BASE = 0xB02280000
DWC_USB31_GSNPSID = 0x33313130  # "0131" = DWC_USB31 v1.30, measured live

# DWC3 global block (offsets from the xHCI base)
GLB = 0xC100
OFF_GSBUSCFG0 = GLB + 0x00
OFF_GCTL = GLB + 0x10
OFF_GSTS = GLB + 0x18
OFF_GSNPSID = GLB + 0x20
OFF_GBUSERRADDRLO = GLB + 0x30
OFF_GBUSERRADDRHI = GLB + 0x34
OFF_GDBGLTSSM = GLB + 0x64
OFF_GUSB2PHYCFG0 = GLB + 0x100
OFF_GUSB3PIPECTL0 = GLB + 0x1C0

PLS_NAMES = {
    0: "U0", 1: "U1", 2: "U2", 3: "U3/suspend", 4: "Disabled", 5: "RxDetect",
    6: "Inactive", 7: "Polling", 8: "Recovery", 9: "HotReset",
    10: "ComplianceMode", 11: "TestMode", 15: "Resume",
}
SPEED_NAMES = {0: "-", 1: "Full", 2: "Low", 3: "High", 4: "SuperSpeed",
               5: "SuperSpeedPlus"}

# atc-phy ADT reg[] indices (must match src/atcphy.c / m1n1.atcphy.ATCPHY)
ATC_REG_USB2PHY = 0
ATC_REG_CORE = 3
ATC_REG_AXI2AF = 24
DRD_REG_PIPEHANDLER = 3

# Python mirror of atcphy_tunable_table[] (src/atcphy_core.c).  Only the
# fields the audit needs: adt property name -> (block, block_offset,
# block_size, scope).  block: "core" or "axi2af".  scope: "global" or the
# lane family; USB2-only operation is judged on "global" alone.
TUNABLE_VOCAB = {
    "tunable_ATC0AXI2AF":          ("axi2af", 0x0,     0x4000, "global"),
    "tunable_ATC_FABRIC":          ("core",   0x45000, 0x4000, "global"),
    "tunable_USB_ACIOPHY_TOP":     ("core",   0x0,     0x4000, "global"),
    "tunable_AUS_CMN_SHM":         ("core",   0xa00,   0x4000, "global"),
    "tunable_AUS_CMN_TOP":         ("core",   0x800,   0x4000, "global"),
    "tunable_AUSPLL_CORE":         ("core",   0x2200,  0x4000, "global"),
    "tunable_AUSPLL_TOP":          ("core",   0x2000,  0x4000, "global"),
    "tunable_CIO3PLL_CORE":        ("core",   0x2a00,  0x4000, "global"),
    "tunable_CIO3PLL_TOP":         ("core",   0x2800,  0x4000, "global"),
    "tunable_CIO_CIO3PLL_TOP":     ("core",   0x2800,  0x4000, "global"),
    "tunable_DP_LN0_AUSPMA_TX_TOP": ("core", 0xc000,  0x1000, "lane_dp"),
    "tunable_DP_LN1_AUSPMA_TX_TOP": ("core", 0x13000, 0x1000, "lane_dp"),
    "tunable_USB_LN0_AUSPMA_TX_TOP": ("core", 0xc000,  0x1000, "lane_usb3"),
    "tunable_USB_LN0_AUSPMA_RX_TOP": ("core", 0x9000,  0x1000, "lane_usb3"),
    "tunable_USB_LN0_AUSPMA_RX_SHM": ("core", 0xb000,  0x1000, "lane_usb3"),
    "tunable_USB_LN0_AUSPMA_RX_EQ":  ("core", 0xa000,  0x1000, "lane_usb3"),
    "tunable_USB_LN1_AUSPMA_TX_TOP": ("core", 0x13000, 0x1000, "lane_usb3"),
    "tunable_USB_LN1_AUSPMA_RX_TOP": ("core", 0x10000, 0x1000, "lane_usb3"),
    "tunable_USB_LN1_AUSPMA_RX_SHM": ("core", 0x12000, 0x1000, "lane_usb3"),
    "tunable_USB_LN1_AUSPMA_RX_EQ":  ("core", 0x11000, 0x1000, "lane_usb3"),
    "tunable_CIO_LN0_AUSPMA_TX_TOP": ("core", 0xc000,  0x1000, "lane_cio"),
    "tunable_CIO_LN0_AUSPMA_RX_TOP": ("core", 0x9000,  0x1000, "lane_cio"),
    "tunable_CIO_LN0_AUSPMA_RX_SHM": ("core", 0xb000,  0x1000, "lane_cio"),
    "tunable_CIO_LN0_AUSPMA_RX_EQ":  ("core", 0xa000,  0x1000, "lane_cio"),
    "tunable_CIO_LN1_AUSPMA_TX_TOP": ("core", 0x13000, 0x1000, "lane_cio"),
    "tunable_CIO_LN1_AUSPMA_RX_TOP": ("core", 0x10000, 0x1000, "lane_cio"),
    "tunable_CIO_LN1_AUSPMA_RX_SHM": ("core", 0x12000, 0x1000, "lane_cio"),
    "tunable_CIO_LN1_AUSPMA_RX_EQ":  ("core", 0x11000, 0x1000, "lane_cio"),
}


def _bases(u, port=BOOT_PORT):
    """Resolve and anchor-check every MMIO base this tool touches."""
    drd = u.adt[f"/arm-io/usb-drd{port}"]
    atc = u.adt[f"/arm-io/atc-phy{port}"]
    xhci = drd.get_reg(0)[0]
    if port == BOOT_PORT and xhci != KNOWN_DRD1_BASE:
        raise Exception(
            f"usb-drd{port} reg[0] decodes to {xhci:#x}, contradicting the "
            f"hardware-measured {KNOWN_DRD1_BASE:#x} -- decode is wrong, "
            "refusing to read anything")
    snpsid = u.proxy.read32(xhci + OFF_GSNPSID)
    if snpsid != DWC_USB31_GSNPSID:
        raise Exception(
            f"GSNPSID at {xhci + OFF_GSNPSID:#x} reads {snpsid:#x}, expected "
            f"{DWC_USB31_GSNPSID:#x} -- wrong block or unclocked controller, "
            "refusing")
    return {
        "xhci": xhci,
        "usb2phy": atc.get_reg(ATC_REG_USB2PHY)[0],
        "core": atc.get_reg(ATC_REG_CORE)[0],
        "axi2af": atc.get_reg(ATC_REG_AXI2AF)[0],
        "pipehandler": drd.get_reg(DRD_REG_PIPEHANDLER)[0],
    }


def _port_regs(u, xhci, portno):
    caplen = u.proxy.read32(xhci) & 0xFF
    op = xhci + caplen
    base = op + 0x400 + 0x10 * (portno - 1)
    sc = u.proxy.read32(base)
    pmsc = u.proxy.read32(base + 4)
    li = u.proxy.read32(base + 8)
    return sc, pmsc, li


def _fmt_portsc(sc):
    pls = (sc >> 5) & 0xF
    speed = (sc >> 10) & 0xF
    flags = []
    for name, bit in (("CCS", 0), ("PED", 1), ("OCA", 3), ("PR", 4),
                      ("PP", 9), ("CSC", 17), ("PEC", 18), ("WRC", 19),
                      ("OCC", 20), ("PRC", 21), ("PLC", 22), ("CEC", 23)):
        if sc & (1 << bit):
            flags.append(name)
    return (f"{sc:#010x} [{' '.join(flags) or '-'}] "
            f"PLS={PLS_NAMES.get(pls, pls)} "
            f"speed={SPEED_NAMES.get(speed, speed)}")


def snapshot(u, port=BOOT_PORT):
    """One-shot read-only dump.  Save the output; diff good vs bad boots."""
    b = _bases(u, port)
    r32 = u.proxy.read32
    xhci = b["xhci"]
    caplen = r32(xhci) & 0xFF
    op = xhci + caplen

    print(f"=== usb-drd{port} @ {xhci:#x} (op=+{caplen:#x}) ===")
    print(f"  USBCMD          = {r32(op + 0x00):#010x}")
    print(f"  USBSTS          = {r32(op + 0x04):#010x}")
    for n in (1, 2):
        sc, pmsc, li = _port_regs(u, xhci, n)
        print(f"  PORTSC{n}         = {_fmt_portsc(sc)}")
        # USB2 ports: PORTPMSC bit16 = HLE (hardware LPM enable), bits[15:8]
        # BESL/HIRD, bits[7:4] L1 device slot, bits[2:0] L1S.  USB3 ports:
        # [7:0] U1 timeout, [15:8] U2 timeout.  Decode both ways; the reader
        # knows which protocol the port is.
        print(f"  PORTPMSC{n}       = {pmsc:#010x} "
              f"(usb2: HLE={int(bool(pmsc & (1 << 16)))} L1S={pmsc & 7} | "
              f"usb3: U1TO={pmsc & 0xFF} U2TO={(pmsc >> 8) & 0xFF})")
        print(f"  PORTLI{n}         = {li:#010x} (SS link error count "
              f"{li & 0xFFFF}; reserved/0 on USB2 ports)")
    gsts = r32(xhci + OFF_GSTS)
    print(f"  GSTS            = {gsts:#010x} "
          f"(BUSERRADDRVLD={int(bool(gsts & (1 << 4)))} "
          f"CSRTIMEOUT={int(bool(gsts & (1 << 5)))})")
    print(f"  GBUSERRADDR     = "
          f"{(r32(xhci + OFF_GBUSERRADDRHI) << 32) | r32(xhci + OFF_GBUSERRADDRLO):#x}")
    print(f"  GCTL            = {r32(xhci + OFF_GCTL):#010x}")
    print(f"  GSBUSCFG0       = {r32(xhci + OFF_GSBUSCFG0):#010x}")
    print(f"  GUSB2PHYCFG0    = {r32(xhci + OFF_GUSB2PHYCFG0):#010x}")
    print(f"  GUSB3PIPECTL0   = {r32(xhci + OFF_GUSB3PIPECTL0):#010x}")
    ltssm = r32(xhci + OFF_GDBGLTSSM)
    print(f"  GDBGLTSSM       = {ltssm:#010x}")

    print(f"=== atc-phy{port} usb2phy @ {b['usb2phy']:#x} ===")
    print(f"  USBCTL          = {r32(b['usb2phy'] + 0x00):#010x}")
    print(f"  CTL             = {r32(b['usb2phy'] + 0x04):#010x}")
    print(f"  SIG             = {r32(b['usb2phy'] + 0x08):#010x}")
    print(f"  MISCTUNE        = {r32(b['usb2phy'] + 0x1C):#010x}")

    mux = r32(b["pipehandler"] + 0x0C)
    clk = {0: "OFF", 1: "USB3", 2: "USB4", 4: "DUMMY"}.get((mux >> 3) & 7,
                                                           (mux >> 3) & 7)
    data = {0: "USB3", 1: "USB4", 2: "DUMMY"}.get(mux & 7, mux & 7)
    print(f"=== pipehandler @ {b['pipehandler']:#x} ===")
    print(f"  MUX_CTRL        = {mux:#010x} (clk={clk} data={data}; "
          "DUMMY/DUMMY == SuperSpeed parked, port is USB2-only)")


def sample(u, seconds=30, interval=0.5, port=BOOT_PORT):
    """Poll PORTSC/USBSTS/GSTS; print timestamped lines only on CHANGE.

    Run this on a healthy boot while pushing disk I/O.  Any transient CSC/
    PEC/PLC latch, PLS excursion, or GSTS.BUSERRADDRVLD flip is a
    pre-failure margin signal the guest driver would otherwise clear before
    a human sees it.  Purely read-only; safe on a live guest.
    """
    b = _bases(u, port)
    r32 = u.proxy.read32
    xhci = b["xhci"]
    caplen = r32(xhci) & 0xFF
    op = xhci + caplen
    last = None
    t0 = time.time()
    n_polls = 0
    print(f"sampling usb-drd{port} for {seconds}s at {interval}s "
          "(printing changes only)...")
    while time.time() - t0 < seconds:
        cur = (r32(op + 0x400), r32(op + 0x410), r32(op + 0x04),
               r32(xhci + OFF_GSTS), r32(xhci + OFF_GDBGLTSSM))
        if cur != last:
            print(f"[{time.time() - t0:8.3f}s] "
                  f"PORTSC1={_fmt_portsc(cur[0])} PORTSC2={_fmt_portsc(cur[1])} "
                  f"USBSTS={cur[2]:#x} GSTS={cur[3]:#x} LTSSM={cur[4]:#x}")
            last = cur
        n_polls += 1
        time.sleep(interval)
    print(f"done: {n_polls} polls, {seconds}s window")


def _prop_bytes(node, name):
    val = node._properties.get(name)
    if isinstance(val, (bytes, bytearray)):
        return bytes(val)
    return None


def audit_tunables(u, port=BOOT_PORT):
    """Report whether each ADT tunable RMW record is live-satisfied.

    A record is SATISFIED when (read32(block_base + block_offset + offset)
    & mask) == value -- i.e. the register currently holds what an
    atcphy_apply_mode() tunable pass would have left there.  All-global-
    satisfied strongly implies the boot chain (iBoot) tuned this PHY;
    global mismatches mean the boot disk is running on an untuned PHY/
    fabric and applying them (NTASI_ATCPHY_ARM_GUEST_MODE_LEFT, sibling
    repo tools/m1n1-windows-debug.py) is the motivated fix experiment.

    Read-only.  Reads the AXI2AF window (always-clocked fabric bridge on
    the live DMA path) and the ACIOPHY core window (clocked whenever the
    port is in use; the deployed atcphy dump reads it live already).
    """
    b = _bases(u, port)
    node = u.adt[f"/arm-io/atc-phy{port}"]
    r32 = u.proxy.read32
    totals = {}
    print(f"=== atc-phy{port} tunable audit "
          f"(core={b['core']:#x} axi2af={b['axi2af']:#x}) ===")
    for name in sorted(node._properties.keys()):
        if not name.startswith("tunable"):
            continue
        vocab = TUNABLE_VOCAB.get(name)
        blob = _prop_bytes(node, name)
        if vocab is None:
            print(f"  {name:32s} UNKNOWN to the vocabulary -- flagging, "
                  "not guessing an address")
            continue
        if blob is None:
            print(f"  {name:32s} unparsed property type -- skipped")
            continue
        block, block_offset, block_size, scope = vocab
        if len(blob) == 0:
            print(f"  {name:32s} empty blob (valid no-op)")
            continue
        if len(blob) % 12 != 0:
            print(f"  {name:32s} {len(blob)} bytes NOT a multiple of 12 -- "
                  "malformed, skipped")
            continue
        base = b[block]
        ok = bad = invalid = 0
        mismatches = []
        for i in range(len(blob) // 12):
            word0, mask, value = struct.unpack_from("<III", blob, i * 12)
            offset = word0 & 0xFFFFFF
            size = (word0 >> 24) & 0xFF
            if size != 32 or offset % 4 or offset + 4 > block_size:
                invalid += 1
                continue
            addr = base + block_offset + offset
            live = r32(addr)
            if (live & mask) == (value & mask):
                ok += 1
            else:
                bad += 1
                if len(mismatches) < 4:
                    mismatches.append(
                        f"+{block_offset + offset:#x}: live={live:#x} "
                        f"want=({value:#x}&{mask:#x})")
        verdict = "SATISFIED" if bad == 0 and invalid == 0 else "MISMATCH"
        t = totals.setdefault(scope, [0, 0])
        t[0] += ok
        t[1] += bad
        print(f"  {name:32s} [{scope:9s}] {ok:3d} ok / {bad:3d} mismatched"
              f"{f' / {invalid} invalid' if invalid else ''}  -> {verdict}")
        for m in mismatches:
            print(f"      {m}")
    print("  ---- summary ----")
    for scope, (ok, bad) in sorted(totals.items()):
        note = ""
        if scope != "global":
            note = ("  (expected unapplied while the SS mux is parked on "
                    "DUMMY -- informational only)")
        print(f"  {scope:9s}: {ok} satisfied, {bad} mismatched{note}")
    g = totals.get("global")
    if g is None:
        print("  VERDICT: no global tunables found -- investigate the ADT")
    elif g[1] == 0:
        print("  VERDICT: boot-port PHY/fabric IS tuned (all global records "
              "satisfied) -- the untuned-PHY hypothesis is DISPROVED for "
              "this port; weigh cable/enclosure/power next")
    else:
        print("  VERDICT: boot-port PHY/fabric is NOT fully tuned "
              f"({g[1]} global records unsatisfied) -- the boot disk runs "
              "on an untuned PHY/fabric; the motivated fix is the armed "
              "USB2-mode tunable apply (see sibling repo "
              "tools/m1n1-windows-debug.py NTASI_ATCPHY_ARM_GUEST_MODE_LEFT)")


if __name__ == "__main__":
    from m1n1.setup import p, u  # noqa: F401  (standalone proxy session)
    snapshot(u)
    audit_tunables(u)
