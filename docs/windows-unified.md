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
- J414s MTP/DockChannel firmware, keyboard, trackpad, and backlight preboot;
- PCIe initialization plus the guarded ANS and BCM4388 proxy helpers;
- GPU DT/initdata/calibration production; and
- an EL2 TPM 2.0 CRB with the host engine bridge.

Compiling a capability is not permission to mutate its device.  Baseline MTP
is the sole automatic J414s peripheral handoff.  ANS/PCIe endpoint operations,
wireless DART setup, GPU calibration, and TPM attachment require explicit host
calls.  Each explicit call is exact-board gated and fail-closed.

Wireless is stricter than the historical implementation.  There is no fixed
`0x10022000000` carveout.  A Wi-Fi profile must first call
`p.top_of_memory_alloc(0x10000)`, record the returned 16-KiB-aligned range in
its immutable pairing manifest and Mu DRT0 resources, then call
`p.wireless_handoff_init(base, 0x10000)`.  m1n1 rejects a range that is not
above the reduced guest SystemMemory top or outside physical DRAM.  Missing
that contract means DRT0 and wireless handoff are absent.

GPU calibration is similarly explicit: run the drivers-repo GPU pass-one
tool, build Mu from the emitted live six-region manifest, and start that Mu on
the same m1n1 instance.  Merely chainloading this m1n1 image does not start GPU
firmware.

TPM is attached only through `hv.attach_tpm(...)`; no attachment means no CRB
mapping.  A refused or corrupt host store fails loud instead of presenting a
TPM that can lose state.

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
