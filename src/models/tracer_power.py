"""Tracer chassis battery / power sensor from system status (CAN 0x211)."""

from __future__ import annotations

from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.power_sensor import PowerSensor
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import SensorReading, ValueTypes, struct_to_dict

from .can_client import (
    TracerCanClient,
    control_mode_name,
    get_client,
    parse_can_attrs,
    release_client,
)
from . import protocol as proto


class TracerPower(PowerSensor, EasyResource):
    """Reports battery voltage from Tracer system status feedback.

    The chassis does not publish current or power on CAN, so those readings are
    omitted / returned as zero.
    """

    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "agilex-tracer"), "power")

    def __init__(self, name: str):
        super().__init__(name)
        self._client: Optional[TracerCanClient] = None
        self._backend = "socketcan"
        self._channel = "can0"
        self._bitrate = 500000

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
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        attrs = struct_to_dict(config.attributes)
        backend, channel, bitrate, auto_up = parse_can_attrs(attrs)
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
            "Tracer power on %s:%s@%d", self._backend, self._channel, self._bitrate
        )

    async def close(self):
        if self._client is not None:
            release_client(self._client)
            self._client = None

    def _require_client(self) -> TracerCanClient:
        if self._client is None:
            raise RuntimeError("Tracer CAN client is not configured")
        return self._client

    def _system(self) -> proto.SystemStatus:
        return self._require_client().snapshot().system

    async def get_voltage(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[float, bool]:
        return float(self._system().battery_voltage), False

    async def get_current(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[float, bool]:
        # Not available on Tracer system-status CAN; keep API satisfied.
        return 0.0, False

    async def get_power(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> float:
        # Not available on Tracer system-status CAN.
        return 0.0

    async def get_readings(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, SensorReading]:
        system = self._system()
        return {
            "volts": float(system.battery_voltage),
            "amps": 0.0,
            "watts": 0.0,
            "is_ac": False,
            "vehicle_state": int(system.vehicle_state),
            "control_mode": int(system.control_mode),
            "control_mode_name": control_mode_name(system.control_mode),
            "fault_bits": int(system.fault_bits),
            "emergency_stop": bool(system.emergency_stop),
            "under_voltage": bool(system.under_voltage),
            "under_voltage_alarm": bool(system.under_voltage_alarm),
        }

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        name = str(command.get("command", "")).strip().lower()
        if name in ("status", "get_status"):
            system = self._system()
            return {
                "battery_voltage": float(system.battery_voltage),
                "vehicle_state": int(system.vehicle_state),
                "control_mode": int(system.control_mode),
                "control_mode_name": control_mode_name(system.control_mode),
                "fault_bits": int(system.fault_bits),
                "emergency_stop": bool(system.emergency_stop),
                "under_voltage": bool(system.under_voltage),
                "under_voltage_alarm": bool(system.under_voltage_alarm),
                "can_backend": self._backend,
                "can_channel": self._channel,
                "can_bitrate": self._bitrate,
            }
        raise Exception(f"unknown command '{name}'. Supported: get_status")
