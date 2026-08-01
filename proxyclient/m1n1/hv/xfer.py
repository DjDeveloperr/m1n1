# SPDX-License-Identifier: MIT
"""Host side of the bulk host<->guest data channel (src/hv_xfer.c).

The guest writes one command word to a hooked doorbell page; EL2 raises one
HV_XFER proxy event; this module moves the whole window in a single
REQ_MEMWRITE/REQ_MEMREAD over the proxy pipe and answers. The bytes never go
through the vUART, so the cost is one VM exit per window instead of one per
byte.

Everything here is deliberately transport-agnostic: :class:`XferHostDevice`
only ever talks to a small ``transport`` object with ``readmem``/``writemem``,
so the whole protocol can be unit-tested on a laptop with no hardware at all
(see ``proxyclient/tests/test_hv_xfer.py``).

Safety posture: the guest names streams, so the guest could otherwise name a
host file. It cannot. Reads are served ONLY from blobs the operator explicitly
published; writes land ONLY inside one inbox directory, under a sanitised
basename, as ``<name>.part`` until the guest closes the stream. A guest that
dies mid-transfer therefore leaves a ``.part`` file, never a truncated file
that looks complete.
"""

import gzip
import os
import re
import time
from pathlib import Path

from construct import Struct, Int64ul, Int32ul, Int32sl

__all__ = [
    "XferExcInfo", "XferHostDevice", "ProxyTransport",
    "HV_XFER_ID", "HV_XFER_VERSION", "HV_XFER_REGS_SIZE", "HV_XFER_NAME_SIZE",
    "CMD_NOP", "CMD_PING", "CMD_OPEN", "CMD_GET", "CMD_PUT", "CMD_CLOSE",
    "CMD_SET_WINDOW", "CMD_RESET_WINDOW",
    "ST_OK", "ST_INVAL", "ST_NODEV", "ST_IO", "ST_RANGE", "ST_FAULT", "ST_BADCMD",
    "REG",
]

# Mirrors struct hv_xfer_exc_info in src/hv_xfer.h -- keep the two in step.
XferExcInfo = Struct(
    "devbase" / Int64ul,
    "win_phys" / Int64ul,
    "win_size" / Int64ul,
    "name_phys" / Int64ul,
    "cmd" / Int32ul,
    "tag" / Int32ul,
    "off" / Int32ul,
    # `length` here is the C struct's `len`; renamed only to keep Container
    # attribute access away from the builtin.
    "length" / Int32ul,
    "result" / Int32ul,
    "status" / Int32sl,
)

HV_XFER_ID = 0x46585648  # 'HVXF'
HV_XFER_VERSION = 1
HV_XFER_REGS_SIZE = 0x4000
HV_XFER_PAGE_SIZE = 0x4000
HV_XFER_NAME_SIZE = 64

#: Doorbell register offsets, mirroring the HV_XFER_REG_* defines.
REG = {
    "ID": 0x000, "VERSION": 0x004, "FEATURES": 0x008, "PAGESIZE": 0x00c,
    "WIN_LO": 0x010, "WIN_HI": 0x014, "WIN_SIZE": 0x018, "MAX_XFER": 0x01c,
    "CMD": 0x020, "OFF": 0x024, "LEN": 0x028, "TAG": 0x02c,
    "STATUS": 0x030, "RESULT": 0x034, "SEQ": 0x038, "ERRORS": 0x03c,
    "UWIN_LO": 0x040, "UWIN_HI": 0x044, "UWIN_SIZE": 0x048,
    "NAME": 0x080,
}

CMD_NOP = 0
CMD_PING = 1
CMD_OPEN = 2
CMD_GET = 3
CMD_PUT = 4
CMD_CLOSE = 5
CMD_SET_WINDOW = 6
CMD_RESET_WINDOW = 7

ST_OK = 0
ST_INVAL = -1
ST_NODEV = -2
ST_IO = -3
ST_RANGE = -4
ST_FAULT = -5
ST_BADCMD = -6

#: TAG bit 31 selects the direction at OPEN time.
TAG_WRITE = 1 << 31
TAG_ID_MASK = 0x7fffffff

_SAFE_NAME = re.compile(r"\A[A-Za-z0-9][A-Za-z0-9._+-]{0,62}\Z")


def sanitise_name(raw):
    """Turn the guest's 64 raw bytes into a name that cannot escape the inbox.

    Returns None for anything that is not a plain basename. Refusing is the
    only correct answer: a guest that can name ``../../System32/...`` on the
    host is a far worse bug than a failed transfer.
    """
    if isinstance(raw, (bytes, bytearray)):
        raw = bytes(raw).split(b"\0", 1)[0]
        try:
            raw = raw.decode("ascii")
        except UnicodeDecodeError:
            return None
    raw = raw.strip()
    if not _SAFE_NAME.match(raw):
        return None
    if raw in (".", "..") or "/" in raw or "\\" in raw:
        return None
    return raw


class ProxyTransport:
    """The real transport: m1n1's proxy, with optional in-target gunzip.

    ``gzdec`` is m1n1's existing P_GZDEC op, the same one
    ``ProxyUtils.compressed_writemem()`` uses to land the kernel image. Driver
    packages are PE files and compress 2-4x, so this roughly halves the wire
    time for free -- the decompression runs in EL2 while the guest is already
    stopped.
    """

    #: Below this, framing and the gzip header cost more than they save.
    COMPRESS_MIN = 4096
    #: Only bother if compression actually wins by a clear margin.
    COMPRESS_RATIO = 0.9

    def __init__(self, iface, proxy, utils, compress=True):
        self.iface = iface
        self.p = proxy
        self.u = utils
        self.compress = compress
        self.wire_bytes = 0

    def readmem(self, addr, size):
        data = self.iface.readmem(addr, size)
        self.wire_bytes += len(data)
        return data

    def writemem(self, addr, data):
        if self.compress and len(data) >= self.COMPRESS_MIN:
            payload = gzip.compress(data, compresslevel=1)
            if len(payload) < len(data) * self.COMPRESS_RATIO:
                with self.u.heap.guarded_malloc(len(payload)) as scratch:
                    self.iface.writemem(scratch, payload)
                    self.wire_bytes += len(payload)
                    out = self.p.gzdec(scratch, len(payload), addr, len(data))
                    if out != len(data):
                        raise IOError(
                            f"gzdec produced {out} bytes, expected {len(data)}")
                return
        self.iface.writemem(addr, data)
        self.wire_bytes += len(data)


class _ReadStream:
    def __init__(self, name, data):
        self.name = name
        self.data = data
        self.pos = 0


class _WriteStream:
    def __init__(self, name, path):
        self.name = name
        self.path = Path(path)
        self.part = self.path.with_name(self.path.name + ".part")
        self.fh = open(self.part, "wb")
        self.written = 0

    def commit(self):
        self.fh.flush()
        os.fsync(self.fh.fileno())
        self.fh.close()
        os.replace(self.part, self.path)
        return self.written

    def abandon(self):
        try:
            self.fh.close()
        except Exception:
            pass


class XferHostDevice:
    """Serves HV_XFER commands. One instance per mapped channel.

    Publish what the guest may read, point it at an inbox for what the guest
    writes, and hand it to :meth:`m1n1.hv.HV.attach_xfer`.
    """

    def __init__(self, inbox=None, compress=True, verbose=True):
        self.published = {}
        self.inbox = Path(inbox) if inbox is not None else None
        self.compress = compress
        self.verbose = verbose
        self.streams = {}
        self.commands = 0
        self.failures = 0
        self.bytes_out = 0
        self.bytes_in = 0
        self.seconds = 0.0

    # -- staging -----------------------------------------------------------

    def publish(self, name, data):
        """Make ``data`` available to the guest under ``name``."""
        clean = sanitise_name(name)
        if clean is None:
            raise ValueError(f"unsafe publish name {name!r}")
        self.published[clean] = bytes(data)
        return clean

    def publish_file(self, path, name=None):
        path = Path(path)
        return self.publish(name or path.name, path.read_bytes())

    def publish_dir(self, path, pattern="*"):
        """Publish every matching regular file in ``path`` by basename."""
        published = []
        for entry in sorted(Path(path).glob(pattern)):
            if entry.is_file():
                published.append(self.publish_file(entry))
        return published

    def set_inbox(self, path):
        self.inbox = Path(path)
        self.inbox.mkdir(parents=True, exist_ok=True)

    def stats(self):
        rate_out = self.bytes_out / self.seconds if self.seconds else 0.0
        rate_in = self.bytes_in / self.seconds if self.seconds else 0.0
        return {
            "commands": self.commands,
            "failures": self.failures,
            "bytes_out": self.bytes_out,
            "bytes_in": self.bytes_in,
            "seconds": self.seconds,
            "rate_out": rate_out,
            "rate_in": rate_in,
        }

    def log(self, msg):
        if self.verbose:
            print(f"hv_xfer: {msg}")

    # -- protocol ----------------------------------------------------------

    def dispatch(self, transport, cmd, tag, off, length, win_phys, win_size,
                 name_phys=0):
        """Execute one doorbell command. Returns ``(status, result)``.

        Never raises: every failure becomes a negative status, because the
        guest's vCPU is parked waiting for this answer and an exception here
        would drop it into the hypervisor shell instead.
        """
        start = time.monotonic()
        self.commands += 1
        try:
            status, result = self._dispatch(transport, cmd, tag, off, length,
                                            win_phys, win_size, name_phys)
        except Exception as exc:  # noqa: BLE001 - see docstring
            self.log(f"command {cmd} raised: {exc!r}")
            status, result = ST_IO, 0
        self.seconds += time.monotonic() - start
        if status != ST_OK:
            self.failures += 1
        return status, result

    def _dispatch(self, transport, cmd, tag, off, length, win_phys, win_size,
                  name_phys):
        if cmd == CMD_NOP:
            return ST_OK, 0

        if cmd == CMD_PING:
            return ST_OK, HV_XFER_VERSION

        if cmd == CMD_OPEN:
            return self._open(transport, tag, name_phys)

        sid = tag & TAG_ID_MASK
        stream = self.streams.get(sid)

        if cmd == CMD_CLOSE:
            if stream is None:
                return ST_INVAL, 0
            del self.streams[sid]
            if isinstance(stream, _WriteStream):
                total = stream.commit()
                self.log(f"closed inbox stream {stream.name!r} ({total} bytes)")
                return ST_OK, total & 0xffffffff
            self.log(f"closed outbox stream {stream.name!r} at {stream.pos}")
            return ST_OK, stream.pos & 0xffffffff

        if cmd in (CMD_GET, CMD_PUT):
            if stream is None:
                return ST_INVAL, 0
            if off >= win_size or length == 0 or length > win_size - off:
                return ST_RANGE, 0
            if cmd == CMD_GET:
                if not isinstance(stream, _ReadStream):
                    return ST_INVAL, 0
                chunk = stream.data[stream.pos:stream.pos + length]
                if not chunk:
                    return ST_OK, 0  # EOF
                transport.writemem(win_phys + off, chunk)
                stream.pos += len(chunk)
                self.bytes_out += len(chunk)
                return ST_OK, len(chunk)
            if not isinstance(stream, _WriteStream):
                return ST_INVAL, 0
            data = transport.readmem(win_phys + off, length)
            if len(data) != length:
                return ST_IO, 0
            stream.fh.write(data)
            stream.written += length
            self.bytes_in += length
            return ST_OK, length

        return ST_BADCMD, 0

    def _open(self, transport, tag, name_phys):
        if not name_phys:
            return ST_INVAL, 0
        raw = transport.readmem(name_phys, HV_XFER_NAME_SIZE)
        name = sanitise_name(raw)
        if name is None:
            self.log(f"refusing unsafe stream name {bytes(raw)!r}")
            return ST_INVAL, 0

        sid = tag & TAG_ID_MASK
        old = self.streams.pop(sid, None)
        if old is not None:
            # A guest that crashed mid-transfer never sent CLOSE. Dropping the
            # stale stream here is what makes a retry work; a .part file is
            # left behind on purpose so the failure is visible.
            if isinstance(old, _WriteStream):
                old.abandon()
            self.log(f"stream {sid} reopened, dropping stale {old.name!r}")

        if tag & TAG_WRITE:
            if self.inbox is None:
                self.log("PUT stream requested but no inbox is configured")
                return ST_NODEV, 0
            self.inbox.mkdir(parents=True, exist_ok=True)
            self.streams[sid] = _WriteStream(name, self.inbox / name)
            self.log(f"inbox stream {sid} -> {self.inbox / name}")
            return ST_OK, 0

        data = self.published.get(name)
        if data is None:
            self.log(f"guest asked for unpublished blob {name!r}")
            return ST_NODEV, 0
        self.streams[sid] = _ReadStream(name, data)
        self.log(f"outbox stream {sid} -> {name!r} ({len(data)} bytes)")
        return ST_OK, len(data) & 0xffffffff

    def close_all(self):
        """Abandon every open stream. Committed files are left alone."""
        for stream in self.streams.values():
            if isinstance(stream, _WriteStream):
                stream.abandon()
        self.streams.clear()
