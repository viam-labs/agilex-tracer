"""AgileX Tracer differential base over SocketCAN."""

from __future__ import annotations

import asyncio
import math
import time
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.base import Base
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName, Vector3
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from .can_client import (
    TracerCanClient,
    clamp_linear_angular,
    control_mode_name,
    get_client,
    parse_can_attrs,
    release_client,
)
from . import protocol as proto


def _attr_float(attrs: Mapping[str, Any], key: str, default: float) -> float:
    if key not in attrs or attrs[key] is None:
        return default
    return float(attrs[key])


def _attr_str(attrs: Mapping[str, Any], key: str, default: str) -> str:
    if key not in attrs or attrs[key] is None:
        return default
    return str(attrs[key]).strip() or default


def _forward_from_linear(linear: Vector3) -> float:
    """Viam wheeled convention: +Y forward. Fall back to +X if Y is unset."""
    y = float(linear.y)
    x = float(linear.x)
    if abs(y) > 1e-9 or abs(x) <= 1e-9:
        return y
    return x


class TracerBase(Base, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "agilex-tracer"), "base")

    def __init__(self, name: str):
        super().__init__(name)
        self._client: Optional[TracerCanClient] = None
        self._backend = "socketcan"
        self._channel = "can0"
        self._bitrate = 500000
        self._width_m = proto.DEFAULT_TRACK_WIDTH_M
        self._wheel_circumference_m = proto.DEFAULT_WHEEL_CIRCUMFERENCE_M
        self._max_linear_m_s = proto.MAX_LINEAR_MM_S / 1000.0
        self._max_angular_rad_s = proto.MAX_ANGULAR_MRAD_S / 1000.0
        self._lock = asyncio.Lock()
        self._move_task: Optional[asyncio.Task] = None

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        base = cls(config.name)
        base.reconfigure(config, dependencies)
        return base

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        try:
            parse_can_attrs(attrs)
        except Exception as exc:
            raise Exception(str(exc)) from exc
        width = _attr_float(attrs, "width_meters", proto.DEFAULT_TRACK_WIDTH_M)
        if width <= 0:
            raise Exception("width_meters must be > 0")
        circ = _attr_float(
            attrs, "wheel_circumference_meters", proto.DEFAULT_WHEEL_CIRCUMFERENCE_M
        )
        if circ <= 0:
            raise Exception("wheel_circumference_meters must be > 0")
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        attrs = struct_to_dict(config.attributes)
        backend, channel, bitrate, auto_up = parse_can_attrs(attrs)
        self._width_m = _attr_float(attrs, "width_meters", proto.DEFAULT_TRACK_WIDTH_M)
        self._wheel_circumference_m = _attr_float(
            attrs, "wheel_circumference_meters", proto.DEFAULT_WHEEL_CIRCUMFERENCE_M
        )
        self._max_linear_m_s = _attr_float(
            attrs, "max_linear_m_s", proto.MAX_LINEAR_MM_S / 1000.0
        )
        self._max_angular_rad_s = _attr_float(
            attrs, "max_angular_rad_s", proto.MAX_ANGULAR_MRAD_S / 1000.0
        )

        same = (
            self._client is not None
            and self._client.backend == backend
            and self._client.channel == channel
            and self._client.bitrate == bitrate
        )
        if self._client is not None and not same:
            release_client(self._client)
            self._client = None

        if self._client is None:
            self._client = get_client(
                backend, channel, bitrate, logger=self.logger, auto_up=auto_up
            )
        self._backend = backend
        self._channel = channel
        self._bitrate = bitrate
        self.logger.info(
            "Tracer base on %s:%s@%d width=%.4fm wheel_circ=%.4fm",
            self._backend,
            self._channel,
            self._bitrate,
            self._width_m,
            self._wheel_circumference_m,
        )

    async def close(self):
        await self._cancel_move_task()
        if self._client is not None:
            self._client.set_motion(0.0, 0.0, enable=False)
            release_client(self._client)
            self._client = None

    def _require_client(self) -> TracerCanClient:
        if self._client is None:
            raise RuntimeError("Tracer CAN client is not configured")
        return self._client

    async def _cancel_move_task(self) -> None:
        task = self._move_task
        self._move_task = None
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    def _apply_velocity_m_s(self, linear_m_s: float, angular_rad_s: float) -> None:
        client = self._require_client()
        linear_m_s, angular_rad_s = clamp_linear_angular(
            linear_m_s,
            angular_rad_s,
            self._max_linear_m_s,
            self._max_angular_rad_s,
        )
        # Ensure command mode each time we drive (remote may have stolen control).
        try:
            client.enable_can_control()
        except Exception as exc:
            self.logger.warning("enable_can_control failed: %s", exc)
        client.set_motion(linear_m_s, angular_rad_s, enable=True)

    async def set_power(
        self,
        linear: Vector3,
        angular: Vector3,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        await self._cancel_move_task()
        forward = _forward_from_linear(linear)
        linear_m_s = forward * self._max_linear_m_s
        angular_rad_s = float(angular.z) * self._max_angular_rad_s
        async with self._lock:
            await asyncio.to_thread(self._apply_velocity_m_s, linear_m_s, angular_rad_s)

    async def set_velocity(
        self,
        linear: Vector3,
        angular: Vector3,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        """Set velocity. Linear is mm/s (Y forward); angular is deg/s (Z)."""
        await self._cancel_move_task()
        linear_mm_s = _forward_from_linear(linear)
        angular_deg_s = float(angular.z)
        linear_m_s = linear_mm_s / 1000.0
        angular_rad_s = math.radians(angular_deg_s)
        async with self._lock:
            await asyncio.to_thread(self._apply_velocity_m_s, linear_m_s, angular_rad_s)

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        await self._cancel_move_task()
        client = self._require_client()
        async with self._lock:
            await asyncio.to_thread(client.set_motion, 0.0, 0.0, False)

    async def is_moving(self) -> bool:
        client = self._require_client()
        state = client.snapshot()
        _, _, commanded = client.commanded_motion()
        if commanded:
            return True
        return (
            abs(state.motion.linear_m_s) > 0.01
            or abs(state.motion.angular_rad_s) > 0.01
        )

    async def get_properties(
        self, *, timeout: Optional[float] = None, **kwargs
    ) -> Base.Properties:
        return Base.Properties(
            width_meters=self._width_m,
            turning_radius_meters=0.0,
            wheel_circumference_meters=self._wheel_circumference_m,
        )

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        return []

    async def move_straight(
        self,
        distance: int,
        velocity: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        """Open-loop straight move. Prefer wrapping with sensor-controlled for closed-loop."""
        if distance == 0 or velocity == 0:
            await self.stop()
            return

        await self._cancel_move_task()
        distance_m = abs(distance) / 1000.0
        speed_m_s = abs(velocity) / 1000.0
        direction = 1.0 if distance > 0 else -1.0
        if velocity < 0:
            direction *= -1.0
        duration_s = distance_m / max(speed_m_s, 1e-6)

        async def _run() -> None:
            try:
                async with self._lock:
                    await asyncio.to_thread(
                        self._apply_velocity_m_s, direction * speed_m_s, 0.0
                    )
                await asyncio.sleep(duration_s)
            finally:
                async with self._lock:
                    await asyncio.to_thread(
                        self._require_client().set_motion, 0.0, 0.0, False
                    )

        self._move_task = asyncio.create_task(_run())
        try:
            await self._move_task
        finally:
            self._move_task = None

    async def spin(
        self,
        angle: float,
        velocity: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        """Open-loop spin. Prefer wrapping with sensor-controlled for closed-loop."""
        if angle == 0 or velocity == 0:
            await self.stop()
            return

        await self._cancel_move_task()
        angle_rad = math.radians(abs(angle))
        speed_rad_s = math.radians(abs(velocity))
        direction = 1.0 if angle > 0 else -1.0
        if velocity < 0:
            direction *= -1.0
        duration_s = angle_rad / max(speed_rad_s, 1e-6)

        async def _run() -> None:
            try:
                async with self._lock:
                    await asyncio.to_thread(
                        self._apply_velocity_m_s, 0.0, direction * speed_rad_s
                    )
                await asyncio.sleep(duration_s)
            finally:
                async with self._lock:
                    await asyncio.to_thread(
                        self._require_client().set_motion, 0.0, 0.0, False
                    )

        self._move_task = asyncio.create_task(_run())
        try:
            await self._move_task
        finally:
            self._move_task = None

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        client = self._require_client()
        name = str(command.get("command", "")).strip().lower()
        if name in ("enable_can", "enable_can_control"):
            await asyncio.to_thread(client.enable_can_control)
            return {"ok": True}
        if name in ("clear_faults", "clear_error"):
            which = int(command.get("which", 0) or 0)
            await asyncio.to_thread(client.clear_faults, which)
            return {"ok": True, "which": which}
        if name in ("status", "get_status"):
            state = client.snapshot()
            mode = int(state.system.control_mode)
            return {
                "vehicle_state": state.system.vehicle_state,
                "control_mode": mode,
                "control_mode_name": control_mode_name(mode),
                "battery_voltage": state.system.battery_voltage,
                "fault_bits": state.system.fault_bits,
                "emergency_stop": state.system.emergency_stop,
                "can_backend": self._backend,
                "can_channel": self._channel,
                "can_bitrate": self._bitrate,
                "linear_m_s": state.motion.linear_m_s,
                "angular_rad_s": state.motion.angular_rad_s,
                "hint": (
                    "Motion needs control_mode=can_command (1). "
                    "If mode is remote (2), set FS SWB to command/navigation "
                    "or power the transmitter off, then DoCommand enable_can_control. "
                    "Lights work in remote mode; drive does not."
                ),
            }
        raise Exception(
            f"unknown command '{name}'. Supported: enable_can_control, clear_faults, get_status"
        )
