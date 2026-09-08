"""Wheeled odometry movement sensor from Tracer CAN tire odometers + velocity feedback.

Integrates left/right wheel travel (0x311, mm) with track width into a local pose,
matching Viam's builtin wheeled-odometry conventions so a sensor-controlled base can
use LinearVelocity / AngularVelocity / Orientation / Position feedback.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.movement_sensor import GeoPoint, MovementSensor, Orientation
from viam.errors import NotSupportedError
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, Vector3
from viam.proto.component.movementsensor import GetAccuracyResponse
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from .can_client import TracerCanClient, get_client, parse_can_attrs, release_client
from . import protocol as proto

_M_TO_KM = 0.001


def _attr_float(attrs: Mapping[str, Any], key: str, default: float) -> float:
    if key not in attrs or attrs[key] is None:
        return default
    return float(attrs[key])


def _wrap_yaw_rad(yaw: float) -> float:
    yaw = math.fmod(yaw, 2.0 * math.pi)
    if yaw < 0:
        yaw += 2.0 * math.pi
    return yaw


def _point_at_distance_and_bearing(
    origin_lat: float, origin_lng: float, distance_km: float, bearing_deg: float
) -> Tuple[float, float]:
    """Destination lat/lng given distance (km) and bearing (deg) from origin."""
    if distance_km == 0:
        return origin_lat, origin_lng
    r = 6371.0
    br = math.radians(bearing_deg)
    lat1 = math.radians(origin_lat)
    lng1 = math.radians(origin_lng)
    lat2 = math.asin(
        math.sin(lat1) * math.cos(distance_km / r)
        + math.cos(lat1) * math.sin(distance_km / r) * math.cos(br)
    )
    lng2 = lng1 + math.atan2(
        math.sin(br) * math.sin(distance_km / r) * math.cos(lat1),
        math.cos(distance_km / r) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lng2)


class TracerOdometry(MovementSensor, EasyResource):
    MODEL: ClassVar[Model] = Model(
        ModelFamily("viam-labs", "agilex-tracer"), "odometry"
    )

    def __init__(self, name: str):
        super().__init__(name)
        self._client: Optional[TracerCanClient] = None
        self._backend = "socketcan"
        self._channel = "can0"
        self._bitrate = 500000
        self._width_m = proto.DEFAULT_TRACK_WIDTH_M
        self._time_interval_s = 0.05

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._last_left_mm: Optional[int] = None
        self._last_right_mm: Optional[int] = None
        self._pos_x_m = 0.0
        self._pos_y_m = 0.0
        self._yaw_rad = 0.0
        self._lin_vel_y_m_s = 0.0
        self._ang_vel_z_deg_s = 0.0
        self._origin_lat = 0.0
        self._origin_lng = 0.0
        self._coord_lat = 0.0
        self._coord_lng = 0.0
        self._prefer_can_velocity = True

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        sensor = cls(config.name)
        sensor.reconfigure(config, dependencies)
        return sensor

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        try:
            parse_can_attrs(attrs)
        except Exception as exc:
            raise Exception(str(exc)) from exc
        if _attr_float(attrs, "width_meters", proto.DEFAULT_TRACK_WIDTH_M) <= 0:
            raise Exception("width_meters must be > 0")
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        attrs = struct_to_dict(config.attributes)
        backend, channel, bitrate = parse_can_attrs(attrs)
        self._width_m = _attr_float(attrs, "width_meters", proto.DEFAULT_TRACK_WIDTH_M)
        interval_ms = _attr_float(attrs, "time_interval_msec", 50.0)
        self._time_interval_s = max(0.01, interval_ms / 1000.0)
        prefer = attrs.get("prefer_can_velocity", True)
        if isinstance(prefer, str):
            self._prefer_can_velocity = prefer.strip().lower() in (
                "1",
                "true",
                "yes",
                "on",
            )
        else:
            self._prefer_can_velocity = bool(prefer) if prefer is not None else True

        self._stop_worker()
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
                backend, channel, bitrate, logger=self.logger
            )
        self._backend = backend
        self._channel = channel
        self._bitrate = bitrate
        self._start_worker()
        self.logger.info(
            "Tracer odometry on %s:%s@%d width=%.4fm interval=%.0fms",
            self._backend,
            self._channel,
            self._bitrate,
            self._width_m,
            self._time_interval_s * 1000.0,
        )

    def _stop_worker(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None

    def _start_worker(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._track_loop, name=f"tracer-odom-{self.name}", daemon=True
        )
        self._thread.start()

    async def close(self):
        self._stop_worker()
        if self._client is not None:
            release_client(self._client)
            self._client = None

    def _track_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._update_once()
            except Exception as exc:
                self.logger.warning("odometry update failed: %s", exc)
            self._stop.wait(self._time_interval_s)

    def _update_once(self) -> None:
        if self._client is None:
            return
        state = self._client.snapshot()
        odom = state.odometer
        if odom.last_update <= 0:
            return

        with self._lock:
            if self._last_left_mm is None or self._last_right_mm is None:
                self._last_left_mm = odom.left_mm
                self._last_right_mm = odom.right_mm
                return

            left_delta_m = (odom.left_mm - self._last_left_mm) / 1000.0
            right_delta_m = (odom.right_mm - self._last_right_mm) / 1000.0
            self._last_left_mm = odom.left_mm
            self._last_right_mm = odom.right_mm

            # Guard against CAN counter resets / wrap glitches.
            if abs(left_delta_m) > 5.0 or abs(right_delta_m) > 5.0:
                return

            center_dist = (left_delta_m + right_delta_m) / 2.0
            center_angle = (right_delta_m - left_delta_m) / self._width_m

            self._yaw_rad = _wrap_yaw_rad(self._yaw_rad + center_angle)
            # Match builtin wheeled-odometry: +Y forward, X flipped without compass.
            self._pos_x_m += -center_dist * math.sin(self._yaw_rad)
            self._pos_y_m += center_dist * math.cos(self._yaw_rad)

            distance_m = math.hypot(self._pos_x_m, self._pos_y_m)
            heading_deg = math.degrees(math.atan2(self._pos_x_m, self._pos_y_m))
            self._coord_lat, self._coord_lng = _point_at_distance_and_bearing(
                self._origin_lat,
                self._origin_lng,
                distance_m * _M_TO_KM,
                heading_deg,
            )

            dt = self._time_interval_s
            integrated_lin = center_dist / dt
            integrated_ang_deg = math.degrees(center_angle) / dt

            if self._prefer_can_velocity and state.motion.last_update > 0:
                age = time.time() - state.motion.last_update
                if age < 0.5:
                    self._lin_vel_y_m_s = state.motion.linear_m_s
                    self._ang_vel_z_deg_s = math.degrees(state.motion.angular_rad_s)
                else:
                    self._lin_vel_y_m_s = integrated_lin
                    self._ang_vel_z_deg_s = integrated_ang_deg
            else:
                self._lin_vel_y_m_s = integrated_lin
                self._ang_vel_z_deg_s = integrated_ang_deg

    def _reset_pose(self) -> None:
        with self._lock:
            self._pos_x_m = 0.0
            self._pos_y_m = 0.0
            self._yaw_rad = 0.0
            self._coord_lat = self._origin_lat
            self._coord_lng = self._origin_lng
            self._last_left_mm = None
            self._last_right_mm = None

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[GeoPoint, float]:
        with self._lock:
            if extra and extra.get("return_relative"):
                return GeoPoint(latitude=self._pos_y_m, longitude=self._pos_x_m), 0.0
            return GeoPoint(latitude=self._coord_lat, longitude=self._coord_lng), 0.0

    async def get_linear_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Vector3:
        with self._lock:
            # sensor-controlled reads Y as forward velocity (m/s).
            return Vector3(x=0.0, y=self._lin_vel_y_m_s, z=0.0)

    async def get_angular_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Vector3:
        with self._lock:
            return Vector3(x=0.0, y=0.0, z=self._ang_vel_z_deg_s)

    async def get_linear_acceleration(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Vector3:
        raise NotSupportedError(
            f"MovementSensor named {self.name} does not support returning linear acceleration"
        )

    async def get_compass_heading(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> float:
        raise NotSupportedError(
            f"MovementSensor named {self.name} does not support returning compass heading"
        )

    async def get_orientation(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Orientation:
        with self._lock:
            yaw_deg = math.degrees(self._yaw_rad)
        return Orientation(o_x=0.0, o_y=0.0, o_z=1.0, theta=yaw_deg)

    async def get_properties(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> MovementSensor.Properties:
        return MovementSensor.Properties(
            linear_velocity_supported=True,
            angular_velocity_supported=True,
            orientation_supported=True,
            position_supported=True,
            compass_heading_supported=False,
            linear_acceleration_supported=False,
        )

    async def get_accuracy(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> GetAccuracyResponse:
        return GetAccuracyResponse()

    async def get_readings(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, Any]:
        with self._lock:
            pos_x, pos_y = self._pos_x_m, self._pos_y_m
            yaw = self._yaw_rad
            lin = self._lin_vel_y_m_s
            ang = self._ang_vel_z_deg_s
        left_mm = right_mm = 0
        battery = 0.0
        if self._client is not None:
            snap = self._client.snapshot()
            left_mm = snap.odometer.left_mm
            right_mm = snap.odometer.right_mm
            battery = snap.system.battery_voltage
        return {
            "position_meters_X": pos_x,
            "position_meters_Y": pos_y,
            "yaw_deg": math.degrees(yaw),
            "linear_velocity_m_s": lin,
            "angular_velocity_deg_s": ang,
            "left_odometer_mm": left_mm,
            "right_odometer_mm": right_mm,
            "battery_voltage": battery,
        }

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        name = str(command.get("command", "")).strip().lower()
        if name in ("reset", "reset_odometry"):
            self._reset_pose()
            return {"ok": True}
        if name == "set_origin":
            lat = float(command.get("lat", 0.0) or 0.0)
            lng = float(command.get("long", command.get("lng", 0.0)) or 0.0)
            with self._lock:
                self._origin_lat = lat
                self._origin_lng = lng
                self._coord_lat = lat
                self._coord_lng = lng
            return {"ok": True, "lat": lat, "long": lng}
        raise Exception(f"unknown command '{name}'. Supported: reset_odometry, set_origin")
