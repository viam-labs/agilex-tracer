"""Unit tests for Tracer CAN protocol packing (no hardware required)."""

from __future__ import annotations

import struct
import unittest

from src.models import protocol as proto
from src.models.can_client import parse_can_attrs, resolve_backend


class ProtocolTests(unittest.TestCase):
    def test_encode_motion(self):
        data = proto.encode_motion(0.5, -0.25)
        self.assertEqual(len(data), 8)
        linear, angular = struct.unpack(">hh", data[0:4])
        self.assertEqual(linear, 500)
        self.assertEqual(angular, -250)

    def test_encode_motion_clamps(self):
        data = proto.encode_motion(5.0, 5.0)
        linear, angular = struct.unpack(">hh", data[0:4])
        self.assertEqual(linear, proto.MAX_LINEAR_MM_S)
        self.assertEqual(angular, proto.MAX_ANGULAR_MRAD_S)

    def test_decode_motion_feedback(self):
        payload = struct.pack(">hh", -300, 100) + bytes(4)
        fb = proto.decode_motion_feedback(payload, now=1.0)
        self.assertIsNotNone(fb)
        assert fb is not None
        self.assertAlmostEqual(fb.linear_m_s, -0.3)
        self.assertAlmostEqual(fb.angular_rad_s, 0.1)

    def test_decode_odometer(self):
        payload = struct.pack(">ii", 1234, -56)
        odom = proto.decode_odometer(payload, now=2.0)
        self.assertIsNotNone(odom)
        assert odom is not None
        self.assertEqual(odom.left_mm, 1234)
        self.assertEqual(odom.right_mm, -56)

    def test_encode_light(self):
        data = proto.encode_light(True, proto.LightMode.CUSTOM, 40, 7)
        self.assertEqual(data[0], 0x01)
        self.assertEqual(data[1], int(proto.LightMode.CUSTOM))
        self.assertEqual(data[2], 40)
        self.assertEqual(data[7], 7)

    def test_encode_control_mode(self):
        data = proto.encode_control_mode(proto.ControlMode.CAN_COMMAND)
        self.assertEqual(len(data), 8)
        self.assertEqual(data[0], 0x01)
        self.assertEqual(data[1:], bytes(7))

    def test_decode_system_status(self):
        payload = (
            bytes([0x00, 0x01])
            + struct.pack(">H", 245)
            + struct.pack(">H", 1 << 15)
            + bytes([0x00, 3])
        )
        status = proto.decode_system_status(payload, now=3.0)
        self.assertIsNotNone(status)
        assert status is not None
        self.assertEqual(status.control_mode, 1)
        self.assertAlmostEqual(status.battery_voltage, 24.5)
        self.assertTrue(status.emergency_stop)

    def test_decode_battery_voltage_scales(self):
        # Manual ×10 encoding (0.1 V).
        self.assertAlmostEqual(proto.decode_battery_voltage(265), 26.5)
        # Some Tracer firmwares send ×100 (0.01 V) → raw 2650.
        self.assertAlmostEqual(proto.decode_battery_voltage(2650), 26.5)


class BackendTests(unittest.TestCase):
    def test_resolve_explicit(self):
        self.assertEqual(resolve_backend("socketcan", "0"), "socketcan")
        self.assertEqual(resolve_backend("slcan", "can0"), "slcan")

    def test_resolve_auto_serial(self):
        self.assertEqual(resolve_backend("auto", "/dev/ttyUSB0"), "slcan")
        self.assertEqual(resolve_backend("auto", "can0"), "socketcan")

    def test_parse_attrs_aliases(self):
        backend, channel, bitrate, auto_up = parse_can_attrs(
            {"can_backend": "slcan", "can_channel": "/dev/ttyACM0", "can_bitrate": 500000}
        )
        self.assertEqual(backend, "slcan")
        self.assertEqual(channel, "/dev/ttyACM0")
        self.assertEqual(bitrate, 500000)
        self.assertTrue(auto_up)

        backend, channel, bitrate, auto_up = parse_can_attrs(
            {"can_interface": "can0", "can_auto_up": False}
        )
        self.assertEqual(backend, "socketcan")
        self.assertEqual(channel, "can0")
        self.assertEqual(bitrate, 500000)
        self.assertFalse(auto_up)

    def test_reject_gs_usb(self):
        with self.assertRaises(ValueError):
            resolve_backend("gs_usb", "0")


if __name__ == "__main__":
    unittest.main()
