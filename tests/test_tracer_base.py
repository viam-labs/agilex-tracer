"""Open-loop Spin / MoveStraight timing tests (no hardware required)."""

from __future__ import annotations

import math
import unittest
from unittest import mock

from src.models import protocol as proto
from src.models import tracer_base
from src.models.tracer_base import TracerBase, _open_loop_speed


class FakeClient:
    def __init__(self):
        self.motions = []

    def enable_can_control(self):
        pass

    def set_motion(self, linear_m_s, angular_rad_s, enable=True):
        self.motions.append((linear_m_s, angular_rad_s, enable))


class OpenLoopSpeedTests(unittest.TestCase):
    def test_under_limit_keeps_requested_speed(self):
        self.assertAlmostEqual(_open_loop_speed(0.3, 1.0, 1.0), 0.3)

    def test_uses_absolute_value(self):
        self.assertAlmostEqual(_open_loop_speed(-0.3, 1.0, 1.0), 0.3)

    def test_clamps_to_configured_limit(self):
        self.assertAlmostEqual(_open_loop_speed(0.8, 0.5, 1.0), 0.5)

    def test_clamps_to_protocol_limit_above_configured(self):
        self.assertAlmostEqual(_open_loop_speed(3.0, 2.0, 1.0), 1.0)

    def test_unset_configured_limit_falls_back_to_protocol(self):
        self.assertAlmostEqual(_open_loop_speed(3.0, 0.0, 1.0), 1.0)


class OpenLoopMoveTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.base = TracerBase("base")
        self.client = FakeClient()
        self.base._client = self.client
        patcher = mock.patch.object(tracer_base.asyncio, "sleep", new=mock.AsyncMock())
        self.sleep = patcher.start()
        self.addCleanup(patcher.stop)

    def slept_s(self) -> float:
        self.sleep.assert_awaited_once()
        return self.sleep.await_args.args[0]

    async def test_spin_within_limit(self):
        await self.base.spin(45, 30)
        self.assertAlmostEqual(self.slept_s(), 1.5)
        self.assertAlmostEqual(self.client.motions[0][1], math.radians(30))

    async def test_spin_above_protocol_limit_times_from_clamped_speed(self):
        # 90 deg/s is above the 1 rad/s chassis cap; timing from 90 deg/s
        # would stop after 0.5 s, about 28.6 deg instead of 45.
        await self.base.spin(45, 90)
        max_rad_s = proto.MAX_ANGULAR_MRAD_S / 1000.0
        self.assertAlmostEqual(self.client.motions[0][1], max_rad_s)
        self.assertAlmostEqual(self.slept_s(), math.radians(45) / max_rad_s)

    async def test_spin_respects_configured_limit(self):
        self.base._max_angular_rad_s = 0.5
        await self.base.spin(90, 90)
        self.assertAlmostEqual(self.client.motions[0][1], 0.5)
        self.assertAlmostEqual(self.slept_s(), math.radians(90) / 0.5)

    async def test_spin_negative_angle_turns_clockwise(self):
        await self.base.spin(-45, 30)
        self.assertAlmostEqual(self.client.motions[0][1], -math.radians(30))

    async def test_spin_stops_at_end(self):
        await self.base.spin(45, 30)
        self.assertEqual(self.client.motions[-1], (0.0, 0.0, False))

    async def test_move_straight_respects_configured_limit(self):
        self.base._max_linear_m_s = 0.5
        await self.base.move_straight(1000, 1000)
        self.assertAlmostEqual(self.client.motions[0][0], 0.5)
        self.assertAlmostEqual(self.slept_s(), 2.0)

    async def test_move_straight_backward(self):
        await self.base.move_straight(-500, 250)
        self.assertAlmostEqual(self.client.motions[0][0], -0.25)
        self.assertAlmostEqual(self.slept_s(), 2.0)


if __name__ == "__main__":
    unittest.main()
