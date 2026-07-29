# SPDX-License-Identifier: MIT
import signal
import struct
import unittest

from m1n1.proxy import Feature, UartInterface


class _FakeDevice:
    def __init__(self, events):
        self.events = events

    def write(self, data):
        self.events.append(("write", bytes(data)))
        return len(data)


class ProxyWriteMemSignalTests(unittest.TestCase):
    def _exercise(self, signum, exception):
        iface = UartInterface.__new__(UartInterface)
        events = []
        iface.dev = _FakeDevice(events)
        iface.debug = False
        iface.enabled_features = Feature(0)
        iface.data_checksum = lambda data: 0x12345678

        def cmd(opcode, request):
            events.append(("cmd", opcode, request))
            signal.raise_signal(signum)

        def reply(opcode):
            events.append(("reply", opcode))
            return bytes(24)

        iface.cmd = cmd
        iface.reply = reply

        previous = signal.getsignal(signum)
        with self.assertRaises(exception) as raised:
            iface.writemem(0x12340000, b"payload")
        self.assertIs(signal.getsignal(signum), previous)
        if exception is SystemExit:
            self.assertEqual(raised.exception.code, 128 + signum)

        self.assertEqual(events[0][0], "cmd")
        self.assertEqual(events[1], ("write", b"payload"))
        self.assertEqual(events[2], ("reply", iface.REQ_MEMWRITE))
        address, size, checksum = struct.unpack("<QQI", events[0][2])
        self.assertEqual(address, 0x12340000)
        self.assertEqual(size, 7)
        self.assertEqual(checksum, 0x12345678)

    def test_sigint_is_replayed_only_after_memwrite_reply(self):
        self._exercise(signal.SIGINT, KeyboardInterrupt)

    def test_sigterm_is_replayed_only_after_memwrite_reply(self):
        self._exercise(signal.SIGTERM, SystemExit)

    def test_sighup_is_replayed_only_after_memwrite_reply(self):
        self._exercise(signal.SIGHUP, SystemExit)


if __name__ == "__main__":
    unittest.main()
