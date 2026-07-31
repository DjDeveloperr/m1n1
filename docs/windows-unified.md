# Unified J414s Windows m1n1

The authoritative source is the single checkout `/Users/dj/Developer/m1n1`
on branch `feature/j414s-windows-unified`.  This branch descends from the
hardware-proven `windows-native-aic` lineage and is the only m1n1 source that
new Windows hardware runs should build or chainload.

## Default capabilities

One image contains the non-conflicting capabilities needed by every profile:

- exact J414s/T6020 identity and ten-core sparse-topology handling;
- native AIC2 handoff, Fast-IPI, timer reflection, and startup carrier fixes;
- retained DCP framebuffer and DART ownership handoff;
- both non-proxy xHCI controllers, their DARTs, USB2 PHY host role, and the
  external Type-C source/DFP policy needed to power a right-side USB-C device;
- J414s MTP/DockChannel firmware, keyboard, trackpad, and backlight preboot;
- PCIe initialization plus the guarded ANS and BCM4388 proxy helpers;
- a dormant, versioned BCM4388 SID1 descriptor/rollback transaction core;
- GPU DT/initdata/calibration production;
- an EL2 TPM 2.0 CRB with the host engine bridge; and
- the J414s media-profile census (MCA/ADMAC audio, AOP microphones, ISP
  camera), which has no automatic call site at all.

Compiling a capability is not permission to mutate its device.  Baseline MTP
and the non-proxy USB Type-C host policy are the only automatic J414s
peripheral handoffs.  ANS/PCIe endpoint operations,
wireless DART setup, GPU calibration, and TPM attachment require explicit host
calls.  Each explicit call is exact-board gated and fail-closed.

The Type-C handoff is deliberately narrower than a general USB-C driver.  On
J414s only, after m1n1 releases each unused DWC3/DART, it reads the complete
17-byte CD3217/TPS6598x System Configuration register and changes only a
dual-role Sink/UFP `PortInfo` value to its matching Source/DFP value.  The
controller specification defines that write as a disconnect/reconnect with
the new settings.  The exact proxy-selected HPM is skipped, sink-only and
disabled power paths are rejected, and an exact full-register readback is
mandatory before HPM interrupts are restored.  No GAID/cold reset is issued.

This is regression recovery, not a new USB feature.  The original
`f17a15d1` hardware experiment explicitly issued `SWDF` and `SWSr` to the HPMs,
which could leave the connectors in working host/source state.  The later
`da86932a` Windows handoff made the internal USB2 PHY host role durable but did
not preserve that external HPM state; logs can therefore show XHC2 released,
mapped, and started while the connector still has no VBUS.  The new register
policy makes that formerly stateful step part of the unified baseline.

This makes right-side VBUS and USB2 hotplug a baseline candidate, not yet a
hardware success claim.  SuperSpeed remains disabled on the safe dummy PHY
backend.  The next physical gate must cover an adapter present at boot and an
unplug/replug cycle; if only the first succeeds, a resident Windows Type-C
policy driver is still required.

The legacy dormant BCM4388 descriptor producer from `b6616476` is deliberately
separate from the authoritative dynamic runtime Wi-Fi handoff. It has
host-tested fixed four-page ownership, CRC descriptor publication, and full
rollback semantics, but no runtime or proxy call site. Its exported test API
is named `bcm4388_legacy_dormant_handoff_install` to prevent it being confused
with the current Windows contract. Linking it does not configure PCIe, DART,
RID2SID, MSI, or endpoint BME. Its fixed layout is incompatible with the
current dynamic Mu/AppleDart ABI and must not be used by a hardware profile.

Wireless is stricter than the historical implementation.  There is no fixed
`0x10022000000` carveout.  A Wi-Fi profile must first call
`p.top_of_memory_alloc(0x10000)`, record the returned 16-KiB-aligned range in
its immutable pairing manifest and Mu DRT0 resources, then call
`p.wireless_handoff_init(base, 0x10000)`.  m1n1 rejects a range that is not
above the reduced guest SystemMemory top or outside physical DRAM.  Missing
that contract means DRT0 and wireless handoff are absent.

On success the authoritative producer writes an `NWH2` version-2 descriptor
at reservation offset `0xc000`. It binds the dynamic base/size, reduced guest
memory top, physical memory top, DART base, L1/MSI-L2 addresses, and CRC-32 of
both live tables plus the descriptor. The host must capture the complete
64-KiB reservation and seal it with
`tools/j414s-wireless-handoff-manifest.py`; Mu consumes that exact manifest on
the same m1n1 instance. A missing/corrupt descriptor or any base/CRC/version
mismatch forbids DRT0 and Windows. The legacy `BCM1` fixed-layout transaction
remains test-only and has no proxy or runtime call site.

GPU calibration is similarly explicit: run the drivers-repo GPU pass-one
tool, build Mu from the emitted live six-region manifest, and start that Mu on
the same m1n1 instance.  Merely chainloading this m1n1 image does not start GPU
firmware.

TPM is attached only through `hv.attach_tpm(...)`; no attachment means no CRB
mapping.  A refused or corrupt host store fails loud instead of presenting a
TPM that can lose state.

## Media profile

`AppleMcaAudio`, `AppleAopAudio` and `AppleIsp` each own every mutation of their
own device: MCA raises its own PMGR chain and installs its own translating
`dart-sio` SID-2 domain, ISP raises `ps_isp_*` and *adopts* `dart-isp0`'s
inherited page table, and AOP writes to no DART at all.  There is therefore no
firmware handoff to perform and no descriptor to publish -- unlike wireless,
nothing can DMA before its driver exists.

What is missing is evidence.  Three of those drivers' load-bearing
preconditions are only observable at EL2, before the guest exists, and are
currently assumed.  `p.media_handoff_init(flags)` checks them:

- the iBoot-placed ISP firmware carveout is proven to lie outside every byte
  m1n1 or the guest can allocate (a refusal if it ever stops being true);
- the AOP's DRAM segments are located against that same window and reported;
- `dart-aop` streams 0 and 10 are read and evaluated against
  `AppleAopAudio`'s exact `DVA == PA` precondition.

Two flags add optional behaviour on top; `flags = 0` writes nothing anywhere.
`MEDIA_HANDOFF_FLAG_PROBE_GATED_DARTS` extends the census to `dart-sio` and
`dart-isp0`, whose PMGR gates must be raised first and are then deliberately
left raised, because power-gating a DART discards the very translation
`AppleIsp` adopts.  `MEDIA_HANDOFF_FLAG_MCA_CLOCK_MUXES` programs the six MCA
clock muxes the way `clk_set_mca_muxes()` does on the kboot path this profile
never takes; it selects an NCO source and cannot enable MCLK, raise a `ps_mcaN`,
or reach an amplifier.

Compiling this capability is not permission to use it, and the rule is enforced
rather than documented: `tools/build-j414s-windows-unified.py` refuses an image
in which `media_handoff_init(` appears anywhere outside `src/media_handoff.c`
and the `src/proxy.c` dispatcher.  A boot that never issues the request behaves
exactly as it does today.  The profile publishes no interrupt from m1n1's side
and does not mask, retarget or software-trigger any AIC line.

## Build and seal

From this checkout:

```sh
python3 tools/build-j414s-windows-unified.py \
  --output /Users/dj/Developer/apple_silicon_nt_drivers/build/m1n1-unified
```

The build is incremental.  It copies `m1n1.macho`, `m1n1.elf`, and
`m1n1.bin` into a commit-addressed artifact directory and emits an immutable
manifest with source and file hashes.  Hardware launchers must pin both the
manifest SHA-256 and Mach-O SHA-256; they must never select another checkout
by convention or fallback.

The manifest also pins the Windows-native-AIC ancestor, the original and
integrated dormant BCM4388 commits, and the audited local `main` head,
merge-base, and cherry-distinct update count. See
[`windows-unified-main-update-debt.md`](windows-unified-main-update-debt.md)
before advancing the upstream base.
