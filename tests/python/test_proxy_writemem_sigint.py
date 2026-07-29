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
    def test_sigint_is_replayed_only_after_memwrite_reply(self):
        iface = UartInterface.__new__(UartInterface)
        events = []
        iface.dev = _FakeDevice(events)
        iface.debug = False
        iface.enabled_features = Feature(0)
        iface.data_checksum = lambda data: 0x12345678

        def cmd(opcode, request):
            events.append(("cmd", opcode, request))
            signal.raise_signal(signal.SIGINT)

        def reply(opcode):
            events.append(("reply", opcode))
            return bytes(24)

        iface.cmd = cmd
        iface.reply = reply

        with self.assertRaises(KeyboardInterrupt):
            iface.writemem(0x12340000, b"payload")

        self.assertEqual(events[0][0], "cmd")
        self.assertEqual(events[1], ("write", b"payload"))
        self.assertEqual(events[2], ("reply", iface.REQ_MEMWRITE))
        address, size, checksum = struct.unpack("<QQI", events[0][2])
        self.assertEqual(address, 0x12340000)
        self.assertEqual(size, 7)
        self.assertEqual(checksum, 0x12345678)


if __name__ == "__main__":
    unittest.main()
