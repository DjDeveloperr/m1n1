# hv_xfer: a bulk host&lt;-&gt;guest data channel

## The problem, measured

The guest talks to the host through m1n1's emulated vUART. That page is a
trapping stage-2 hook (`hv_map_vuart` -> `hv_map_hook`, `src/hv_vuart.c:195`),
so **every byte the guest reads or writes is a VM exit into EL2**, serviced by
`handle_vuart()`.

Measured on J414s, 2026-08-01:

* ~94 us per byte, end to end.
* Host baud rate is irrelevant: identical throughput at 115200 and at
  12,000,000 baud. The cost is the trap, not the signalling.
* Effective file transfer 2.6-5.0 KB/s. A 164 KB driver package takes 62 s.
* m1n1 also drops bytes when its TX ring fills (`vuart: TX buffer full, host is
  not draining`).

Since iterating on drivers means pushing 40-200 KB packages repeatedly, this is
the dominant cost in the development loop.

## The mechanism

Stop trapping per byte. Two pieces:

1. **A window of ordinary DRAM**, mapped into the guest as a *normal* stage-2
   mapping (`hv_map_hw`), not a hook. Guest loads and stores to it are
   full-speed native memory accesses that never trap.
2. **One hooked 16 KiB doorbell page.** The guest writes a command word; that
   single trap runs `hv_exc_proxy()`, the tethered Mac moves the whole window
   in one `REQ_MEMWRITE`/`REQ_MEMREAD`, and the guest resumes.

The cost becomes **one VM exit per window instead of one per byte**.

### Why the bulk move rides the proxy pipe, not the vUART pipe

The original sketch had m1n1 perform the bulk USB transfer itself on
`IODEV_USB_VUART` (CDC ACM pipe 1). That is the wrong pipe, for four reasons:

* Pipe 1 is the guest's console lifeline and carries an unframed byte stream.
  Interleaving a binary bulk protocol with console output on the same pipe
  invites exactly the corruption class that is hardest to debug over a single
  serial link.
* `usb_dwc3_read()` on that pipe blocks with no timeout, and `hv_exc_proxy()`
  *suspends the HV watchdog* for its duration (`_hv_exc_proxy`,
  `src/hv_exc.c:666`). A host that stops feeding it wedges EL2 with nothing to
  bark.
* It would need a new framing and checksum protocol written from scratch on
  both sides.
* **The bulk primitive already exists.** `REQ_MEMWRITE` (`src/uartproxy.c:231`)
  takes an address, a size and a checksum, then does a single
  `iodev_read(iodev, (void *)addr, size)` straight into physical memory. It is
  the same path that uploads the whole kernel image at boot, it has exception
  guards, checksums (or an end sentinel with `PROXY_FEAT_DISABLE_DATA_CSUMS`),
  and a working host implementation in `UartInterface.writemem()`.

So the design uses `hv_exc_proxy()` -> `uartproxy_run()` -> `REQ_MEMWRITE` /
`REQ_MEMREAD` on pipe 0. **Pipe 0 keeps working exactly as before**: this is
the same event mechanism `hv_tpm.c` and `hv_virtio.c` already use, and
`run_guest.py` is unmodified.

Free bonus: the host gzips each chunk and m1n1 decompresses it in EL2 with the
existing `P_GZDEC` op (`ProxyTransport.writemem()`). PE files compress 2-4x, so
the wire time roughly halves again. Falls back to a plain write when
compression does not win.

## Where the window lives, and why the guest cannot clobber it

`HV.load_raw()` hands the guest a `boot_args` with
`phys_base = u.heap_top` and `mem_size = (u.ba.phys_base + u.ba.mem_size) -
heap_top`. Guest RAM is mapped by two tracers:

* `RAM-LOW`  = `[ram_base, u.ba.phys_base)`
* `RAM-HIGH` = `[u.heap_top, ram_base + mem_size_actual)`

The span `[u.ba.phys_base, u.heap_top)` -- m1n1's own image and the 768 MiB
proxy heap -- is **not mapped into the guest at all and is not in the guest's
memory map**. That is the carveout, and it already exists; nothing new has to
be reserved. `attach_xfer()` allocates the window with `u.heap.memalign()` out
of that heap and maps it into the guest identity (IPA == PA) with `hv_map_hw`.

Consequences:

* Windows never sees the window in its firmware memory map, so it will never
  allocate it. Nothing can clobber it.
* This is a different mechanism from the wireless DART handoff reservation,
  which lives *above* `guest_memory_top` in the `[mem_size, mem_size_actual)`
  gap and must be derived identically by m1n1 and by Mu's PEI
  (`proxyclient/m1n1/wireless_handoff.py`). That derivation exists because Mu
  needs the address without a channel. Here there is a channel -- the doorbell
  advertises the address -- so no cross-repo agreement is needed and Mu does
  not change.

**Size.** Default 1 MiB, `attach_xfer(size=...)`, hard cap 64 MiB
(`HV_XFER_MAX_WINDOW`). 1 MiB holds any current driver package in one shot and
is 0.13% of the proxy heap. There is no reason to go large: at one exit per
window, 256 KiB already reduces a 164 KB package to a single transfer.

**Optional: the guest supplies its own buffer.** `CMD_SET_WINDOW` rebinds the
channel to a guest-allocated buffer (`UWIN_LO/HI/SIZE`). EL2 walks stage 2 for
every 16 KiB page and refuses anything that is not mapped, not physically
contiguous, or **not inside DRAM** -- that last check is the one that matters,
because `/arm-io` is mapped HW too and contiguity alone would let a guest aim a
bulk write at an MMIO aperture. With this, the driver can hand over a cached
non-paged contiguous allocation and skip `MmMapIoSpaceEx` entirely.

## The doorbell protocol

One 16 KiB hooked page. All registers 32-bit; a 64-bit access to an aligned
pair reads/writes both halves. Offsets are ABI, mirrored in
`proxyclient/m1n1/hv/xfer.py` and asserted equal by the host tests.

| Off | Name | Acc | Meaning |
|-----|------|-----|---------|
| 0x00 | `ID` | RO | `0x46585648` (`'HVXF'`) |
| 0x04 | `VERSION` | RO | 1 |
| 0x08 | `FEATURES` | RO | bit0 `HOST` (a host answered), bit1 `SET_WINDOW` |
| 0x0c | `PAGESIZE` | RO | 0x4000 |
| 0x10 | `WIN_LO`/`WIN_HI` | RO | active window, guest-physical |
| 0x18 | `WIN_SIZE` | RO | bytes |
| 0x1c | `MAX_XFER` | RO | largest `LEN` a command accepts |
| 0x20 | `CMD` | WO | write executes; read returns the last command |
| 0x24 | `OFF` | RW | byte offset into the window |
| 0x28 | `LEN` | RW | byte count |
| 0x2c | `TAG` | RW | stream id; bit 31 at OPEN = guest->host |
| 0x30 | `STATUS` | RO | s32 result of the last command |
| 0x34 | `RESULT` | RO | bytes actually moved |
| 0x38 | `SEQ` | RO | completed-command counter |
| 0x3c | `ERRORS` | RO | failed-command counter |
| 0x40 | `UWIN_LO`/`HI`/`SIZE` | RW | guest-supplied window for `SET_WINDOW` |
| 0x80 | `NAME[64]` | RW | NUL-padded ASCII stream name |

Commands (written to `CMD`, 32-bit stores only):

| # | Name | Host round trip | Effect |
|---|------|-----------------|--------|
| 0 | `NOP` | no | nothing |
| 1 | `PING` | yes | proves the whole path; `RESULT` = host protocol version |
| 2 | `OPEN` | yes | open the stream named in `NAME` (`TAG` bit 31 = write); `RESULT` = total length for reads |
| 3 | `GET` | yes | host fills `window[OFF .. OFF+LEN)`; `RESULT` = bytes written, 0 = EOF |
| 4 | `PUT` | yes | host consumes `window[OFF .. OFF+LEN)` |
| 5 | `CLOSE` | yes | host commits the file; `RESULT` = total bytes |
| 6 | `SET_WINDOW` | no | rebind to `UWIN_*`, validated in EL2 |
| 7 | `RESET_WINDOW` | no | rebind to the m1n1-owned window |

Status (s32): `0` ok, `-1` INVAL, `-2` NODEV, `-3` IO, `-4` RANGE, `-5` FAULT,
`-6` BADCMD.

A `CMD` write must be an isolated 32-bit store. A wider store would also cover
`OFF`, and there is no ordering in which both "`OFF` was set first" and "this
is one access" are true, so it is refused rather than guessed at.

Typical host->guest sequence, one 164 KB package with a 1 MiB window: write
`NAME`, `TAG=1`, `CMD=OPEN`; `OFF=0`, `LEN=0x100000`, `CMD=GET` (moves
everything); `CMD=GET` again (returns 0 = EOF); `CMD=CLOSE`. Four host round
trips, about a dozen trapping MMIO accesses in total.

## What can go wrong

**Guest clobbering the window.** It cannot, in the default configuration: the
window is outside the guest's memory map (see above). In `SET_WINDOW` mode the
buffer is the guest's own, so clobbering it is the guest's problem and cannot
corrupt m1n1.

**Cache coherency between EL2 and the guest.** This is the real hazard and it
is handled explicitly. m1n1 maps all DRAM Normal write-back at EL2
(`mmu_add_default_mappings`, `src/memory.c:448`); the stage-2 entry leaves the
guest free to pick its own attributes, and without S2FWB the *guest's* choice
wins for the combined attribute.

* If the guest maps the window **Normal write-back inner-shareable** -- which
  it should, and which `MmMapIoSpaceEx(..., PAGE_READWRITE)` without
  `PAGE_NOCACHE`, or a cached contiguous allocation, gives -- the two views are
  coherent in hardware and no maintenance is needed.
* If the guest maps it Device or non-cacheable, it is *not* coherent with EL2's
  cacheable view and would read stale bytes.

Rather than trust the guest, the device maintains one invariant: **after any
command that touched the window, EL2 holds no cache lines for the touched
range.** `xfer_window_sync()` does `dsb sy` + `dc civac` over the range, always
*after* the access, never before. Clean-and-invalidate is lossless under either
attribute choice: the clean flushes anything dirty (EL2's own writes, or the
guest's -- data cache maintenance by VA is broadcast within the shareability
domain) before the invalidate drops the line. An invalidate *before* a read
would have been wrong, because it could discard lines the guest legitimately
dirtied; that ordering is asserted by the host tests.

Cost: 1 MiB is 16384 `dc civac` ops, tens of microseconds, negligible next to
the transfer.

**Transfer size limits.** `REQ_MEMWRITE` streams into a 1 MiB CDC ring
(`CDC_BUFFER_SIZE`, `src/usb_dwc3.c:25`) while pumping the event loop, so there
is no hard per-transfer limit -- multi-MB `writemem` is already the boot path.
The 64 MiB cap on the window exists to bound the stage-2 page walk that
`SET_WINDOW` performs, not the transfer.

**The guest crashes mid-transfer.** Nothing in EL2 is left inconsistent: state
lives in the device struct and is reset by the next `OPEN`. On the host, an
inbound file is written as `<name>.part` and only `fsync`ed and atomically
renamed on `CLOSE`, so a crash leaves a visible `.part` and never a truncated
file that looks complete. Reopening the same tag drops the stale stream so a
retry just works.

**The host dies mid-transfer.** EL2 is blocked inside `uartproxy_run()` with
the watchdog suspended, and the machine needs a physical reset. This is not a
new exposure: it is true of every proxy operation today, including
`hv.interrupt()` and any tracer. It is the reason the bulk move is *not* done
on the vUART pipe, where it would be a new and much more likely failure mode.

**The host handler is missing.** `HV.init()` registers `handle_xfer`
unconditionally, even with no channel attached, and it answers `-ENODEV`. An
`HV_XFER` event with no handler would park the guest in `uartproxy_run()`
forever; this cannot happen.

**The device is never mapped.** Then the doorbell IPA is unmapped and a guest
that pokes it takes a stage-2 abort. The guest driver must therefore be gated
on the same operator opt-in as the boot profile -- see the guest section.

## Fallback

The vUART path is **untouched**. `src/hv_vuart.c` is not modified, the console
and the PowerShell agent behave identically, and nothing in the boot chain
depends on `hv_xfer`. If `attach_xfer()` is never called, m1n1 behaves exactly
as before. If it is called and fails, it raises before mapping anything and the
guest simply has no bulk channel. If a command fails at runtime, the guest sees
a negative `STATUS` and can fall back to the vUART for that transfer.

## Building and testing

Host tests, no hardware:

```sh
cd ~/Developer/m1n1
./tests/hv_xfer/run-host-tests.sh
```

That builds the EL2 device natively (plain and ASan/UBSan) against a stubbed
m1n1 environment and runs 13 device test groups, then runs 31 Python tests
covering the protocol engine and asserting register/command/status parity
against `src/hv_xfer.h`.

Firmware build: unchanged, `make` as usual. `src/hv_xfer.o` is in `OBJECTS`.
Note that `tools/run-m2-pro-mu.sh` verifies the m1n1 tree is clean and at
`M1N1_SOURCE_COMMIT` and checks the artifact manifest, so this needs a commit,
a rebuild and a new pin in the boot script -- there is no way to try it without
one rebuild.

### Enabling it

In the drivers repo's `tools/m1n1-windows-debug.py`, after `hv.init()`:

```python
_xfer = os.environ.get("NTASI_HV_XFER", "0")
if _xfer == "1":
    base = hv.attach_xfer(
        size=int(os.environ.get("NTASI_HV_XFER_SIZE", 1 << 20), 0),
        inbox=os.environ.get("NTASI_HV_XFER_INBOX",
                             os.path.expanduser("~/ntasi-inbox")))
    for pkg in os.environ.get("NTASI_HV_XFER_PUBLISH", "").split(":"):
        if pkg:
            hv.xfer_dev.publish_file(pkg)
    print(f"hv_xfer doorbell at {base:#x}")
```

`attach_xfer()` picks the doorbell base with `alloc_mmio_base()` unless
`M1N1_HV_XFER_BASE` is set. Pin it explicitly once you know the value: the
guest driver needs a stable address.

### First hardware test: smallest possible

**Do not write any Windows code yet.** The first test needs no guest driver at
all and no guest involvement:

1. Boot with `NTASI_HV_XFER=1` and stop at the hypervisor shell.
2. Run `hv.xfer_selftest()`.

That drives exactly the transport a `GET`/`PUT` uses -- `REQ_MEMWRITE` and
`REQ_MEMREAD` into the shared window -- verifies every byte, and prints the
achieved KiB/s in each direction. It touches no guest state. If it prints a
number in the MB/s range, the entire premise is confirmed and the remaining
work is purely the guest driver.

Then, still with no driver:

3. `hv.xfer_dev.publish(b"hello", b"...")` and check `hv.xfer_dev.stats()`.

Only after that is it worth booting Windows and reading `ID`/`VERSION`/`WIN_LO`
from the doorbell.

## The guest side (Windows), for you to implement

A kernel-mode helper is unavoidable: usermode cannot map physical memory on
Windows (`\Device\PhysicalMemory` has been kernel-only since Server 2003 SP1)
and cannot issue MMIO. The helper is small -- one device object, one MMIO
mapping, one buffer mapping, and read/write.

**Discovery.** Map the doorbell page and check `ID == 0x46585648` and
`VERSION == 1` before touching anything else. `ID` reads 0 (never all-ones) for
an unimplemented register, so a wrong base is distinguishable from a missing
device -- but a *completely unmapped* base takes a stage-2 abort, so gate the
driver's start on a registry value the deploy step sets, matching the boot
profile. Read the doorbell base from
`HKLM\SYSTEM\CurrentControlSet\Services\<svc>\Parameters\DoorbellBase`;
`alloc_mmio_base()` is deterministic for a given ADT, so the value is stable
across boots, and the host prints it every boot for cross-checking.

**Mapping.**

```c
PHYSICAL_ADDRESS db = { .QuadPart = DoorbellBase };
regs = MmMapIoSpaceEx(db, 0x4000, PAGE_READWRITE | PAGE_NOCACHE);
```

The doorbell is emulated MMIO: map it **non-cached**, and use
`READ_REGISTER_ULONG` / `WRITE_REGISTER_ULONG` so the compiler cannot merge or
reorder register accesses.

For the window, pick one:

* *m1n1-owned window* (simplest): read `WIN_LO`/`WIN_HI`/`WIN_SIZE`, then
  `MmMapIoSpaceEx(win, size, PAGE_READWRITE)` -- **no** `PAGE_NOCACHE`. This is
  real DRAM; mapping it cached is what makes it coherent with EL2 and fast.
* *guest-owned window* (recommended once it works): allocate with
  `MmAllocateContiguousMemorySpecifyCacheNode(size, lo, hi, boundary, MmCached,
  MM_ANY_NODE_OK)`, take `MmGetPhysicalAddress()`, write it to
  `UWIN_LO/HI/SIZE` and issue `CMD_SET_WINDOW`. Non-paged, contiguous, cached
  and owned by you; no `MmMapIoSpaceEx` on the data path at all. EL2 validates
  the range and answers `-5 (FAULT)` if it is not acceptable.

Either way the window must be **Normal write-back**, 16 KiB aligned, with a 16
KiB multiple size.

**Issuing a command.**

```c
for (i = 0; i < 64; i++) WRITE_REGISTER_UCHAR(regs + 0x80 + i, name[i]);
WRITE_REGISTER_ULONG(regs + 0x2c, tag);
WRITE_REGISTER_ULONG(regs + 0x24, off);
WRITE_REGISTER_ULONG(regs + 0x28, len);
WRITE_REGISTER_ULONG(regs + 0x20, cmd);        /* traps; blocks until done */
status = (LONG)READ_REGISTER_ULONG(regs + 0x30);
result =       READ_REGISTER_ULONG(regs + 0x34);
```

The `CMD` store **blocks** for the whole host round trip -- the vCPU is parked
inside `hv_exc_proxy()`. Expect tens of milliseconds for a full window. So:

* Run it at `PASSIVE_LEVEL`, from a worker thread or a serialised I/O queue,
  never from a DPC and never holding a spin lock.
* Serialise commands: one outstanding command per device (a fast mutex is
  enough). The device has one register file; two threads would interleave
  `OFF`/`LEN`/`CMD`.
* Do not touch the window between writing `CMD` and reading `STATUS`.
* Windows' own watchdogs still run during the stall, so keep a single window
  under a few hundred KB until you have measured the real latency.

**Usermode surface.** Buffered `ReadFile`/`WriteFile` on the device object is
the friendliest thing for the PowerShell agent -- one extra `memcpy` of a few
hundred KB costs microseconds and lets the agent do
`[System.IO.File]::ReadAllBytes('\\.\HvXfer')`. A `METHOD_BUFFERED` IOCTL
carrying `{name, direction}` to open a stream, then plain reads/writes, then a
close IOCTL, is a complete design.

**Direction summary.** Host->guest is `OPEN` (read mode) then repeated `GET`
until `RESULT == 0`; guest->host is `OPEN` with `TAG` bit 31 set, then repeated
`PUT`, then `CLOSE`. Only `CLOSE` commits the host file.

## Alternative front end: `BRK #0x4242` hypercall

`HV.enable_xfer_hvcall()` exposes the same engine through the BRK hypercall ABI
this project already uses for its Windows debug hooks
(`hv.add_hvcall`, `tools/m1n1-windows-debug.py`). It needs no MMIO base
agreement and no stage-2 doorbell page:

```
x0 = 0x58464552 ('XFER')   x1 = command       x2 = window guest-physical base
x3 = window size           x4 = offset        x5 = length
x6 = tag                   x7 = guest-physical address of a 64-byte name, or 0
brk #0x4242
-> x0 = status, x1 = bytes moved
```

The window is validated exactly as `SET_WINDOW` validates it, and the same
`dc civac` invariant is applied from the host side with `p.dc_civac()`.

Use it if you would rather not settle on a doorbell address yet, or from a
context where mapping MMIO is awkward. The trade-off is that it has no
discovery register: if the host side is not enabled, the `BRK` falls through to
EL1 and Windows sees a debug break. Only issue it when the operator turned the
channel on for that boot.

## Files

| Path | What |
|------|------|
| `src/hv_xfer.h` | ABI: registers, commands, statuses, event struct |
| `src/hv_xfer.c` | the EL2 device |
| `src/proxy.c`, `src/proxy.h` | `P_HV_MAP_XFER` |
| `src/hv.h` | `HV_XFER` event type |
| `proxyclient/m1n1/hv/xfer.py` | protocol engine + `ProxyTransport` |
| `proxyclient/m1n1/hv/__init__.py` | `attach_xfer`, `handle_xfer`, `enable_xfer_hvcall`, `xfer_selftest` |
| `proxyclient/m1n1/hv/types.py` | `HV_EVENT.XFER` |
| `proxyclient/m1n1/proxy.py` | `hv_map_xfer()` |
| `tests/hv_xfer/` | host tests (C device + Python engine) |
| `proxyclient/tests/test_hv_xfer.py` | Python engine and ABI-parity tests |
