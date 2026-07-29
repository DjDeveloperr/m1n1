# J414s Windows MTP/DockChannel preboot handoff

This document specifies the ownership transfer used by the
`ENABLE_J414S_WINDOWS_MTP_HANDOFF` build option.  It exists solely for the
Windows-native-AIC profile on the `J414sAP` M2 Pro MacBook Pro.  It is compiled
only with `ENABLE_NATIVE_AIC_PASSTHROUGH` and is a runtime no-op unless the
live ADT matches the measured J414s identity tuple: `/chosen` has chip ID
`0x6020`, board ID `4`, and no `target-type` payload; the root has target type
`J414s`, model `Mac14,9`, and the exact compatible byte sequence
`J414sAP\0Mac14,9\0AppleARM\0`. Any missing, extra, or reordered identity
value leaves the handoff disabled.

Normal m1n1/macOS/Linux boots therefore do not touch the MTP ASC, its DART, or
the DockChannel transport.

## What m1n1 establishes

`hv_init()` invokes `mtp_handoff_init()` after host stage-2 setup and before
the guest runs.  The helper:

1. powers the MTP ASC, `dart-mtp`, and `dockchannel-mtp` only when their ADT
   node declares PMGR clock gates;
2. programs DAPF from `/arm-io/dart-mtp` register index 1;
3. enables DART-MTP stream 1 with fresh page tables and a private IOVA window
   `[0x02000000, 0x12000000)` for AP-allocated RTKit system buffers, while
   admitting IOP-owned fixed buffers only within the ADT-declared MTP SRAM
   aperture;
4. boots the MTP ASC via a bounded generic m1n1 RTKit handshake and waits for
   both the IOP and AP to acknowledge `ON`; and
5. polls only the non-consuming remote FIFO `RX_COUNT` until initial `INIT`
   data is present, then stops touching the transport.

The ASC startup is the MTP IOP firmware start: like upstream Linux's generic
`apple-rtkit-helper`, m1n1 sets `ASC_CPU_CONTROL.RUN` and performs the RTKit
HELLO/EPMAP/power handshake.  It does **not** invent, upload, or replace an
interface firmware image.

## DockChannel state handed to Windows

The preboot stage treats the remote FIFO as immutable handoff state. It never
writes a DockChannel IRQ mask, IRQ flag, TX/RX threshold, or data register.
In particular, it never reads `RX_8` or `RX_32`; either operation would consume
the MTP `INIT` packets that identify keyboard, touch, STM, GPIO, and optional
firmware requirements.

J414s has the following expected physical resources, read from the live ADT and
checked by m1n1 before it changes state:

| Resource | Physical address | Minimum mapping | Windows use |
| --- | ---: | ---: | --- |
| DockChannel parent IRQ registers | `0x2a9b14000` | `0x1000` bytes | parent interrupt block |
| Remote FIFO configuration | `0x2a9b30000` | `0x1000` bytes | channel 1 thresholds |
| Remote FIFO data | `0x2a9b34000` | `0x1000` bytes | channel 1 TX/RX and `RX_COUNT` |
| MTP SRAM | `0x2a9c00000` | `0x100000` bytes | IOP-owned fixed RTKit buffers |
| DockChannel index | `1` | — | MTP transport |
| Interrupt GSIV | `677` | — | ACPI interrupt resource |

The live J414s ADT reports the IRQ and configuration apertures as `0x1000`
bytes each; the data aperture is `config + 0x4000`.  The ACPI `AppleMtpHid`
device must describe these **physical** ranges and GSIV 677.  Its driver takes
the initial `INIT` queue as the source of truth and then owns all ring/IRQ
configuration.

At handoff the required persistent state is:

- MTP ASC CPU is running and RTKit's IOP/AP power state is `ON`.
- DAPF permits the ADT-defined MTP DART aperture.
- DART-MTP stream 1 has translation enabled and retains m1n1's RTKit system
  buffer mappings. Preallocated physical buffer requests are accepted only
  inside MTP SRAM `0x2a9c00000..0x2a9cfffff`; all other requests continue
  through normal DART translation.
- The remote DockChannel index-1 FIFO is untouched; queued `INIT` data remains
  available to Windows.
- No m1n1 MTP/DockChannel service loop remains active after guest entry.

## GPIO and interface firmware boundary

The upstream Asahi Linux `dockchannel-hid` driver confirms that MTP GPIO
requests and optional multi-touch firmware are *per-interface* protocol work:
the GPIO identifiers and names arrive in `INIT`, and the firmware load command
(`0x95`) is sent only after the receiving HID interface is opened.  Preemptively
servicing either from m1n1 would require consuming `INIT`, breaking the handoff.

Consequently this m1n1 stage intentionally leaves GPIO pulses, `0xa0` request
acknowledgements, and optional `0x95` interface firmware uploads to the Windows
AppleMtpHid transport driver.  That is the only clean ownership split.  A
platform-specific firmware/GPIO policy must be implemented in that driver (or
in a separately specified, data-driven preboot profile) before assuming every
trackpad interface needs no follow-up protocol work.  The keyboard can still
work whenever its advertised endpoint needs no such host-side bootstrap.

The implementation was cross-checked against Asahi Linux's
`drivers/soc/apple/rtkit-helper.c` and
`drivers/hid/dockchannel-hid/dockchannel-hid.c` at commit
`e8efe09d4f378992c890d181d65e2ed8d8cb1194`.

## Failure policy

An unexpected ADT map, DAPF/DART setup failure, failed bounded RTKit boot, or
absence of queued INIT data rolls the partial ASC/DART allocation back and
leaves the handoff disabled. The ASC is stopped before RTKit buffers or DART
mappings are released, and rollback never waits for a quiesce acknowledgement.
This is intentional: presenting Windows with a half-initialized MTP transport
is less safe than not publishing m1n1-owned state at all. The helper logs its final
resource map and non-consuming initial `RX_COUNT`; those two lines are the
pre-run validation evidence for the next hardware test.

Capture the m1n1 console and validate that evidence without touching the live
target:

```sh
tools/validate_mtp_handoff_log.py path/to/m1n1-console.log
```

The checker requires exactly one success record, the live J414s resource map,
and a nonzero INIT `RX_COUNT`. It rejects stale addresses, bounded setup
failures, missing terminal output, duplicate runs, and zero-byte handoffs.
