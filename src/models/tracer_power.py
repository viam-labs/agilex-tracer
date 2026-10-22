"""Tracer chassis battery / power sensor from system status + BMS CAN frames."""

from __future__ import annotations

import time
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

_BMS_STALE_S = 2.0


def _attr_float(attrs: Mapping[str, Any], key: str, default: float) -> float:
    if key not in attrs or attrs[key] is None:
        return default
    return float(attrs[key])


class TracerPower(PowerSensor, EasyResource):
    """Battery voltage/current from Tracer system status (0x211) + BMS (0x361).

    Charging is inferred when BMS current exceeds ``charging_current_threshold_amps``
    (default 0.5 A), which matches observed Tracer 2.0 behavior while powered.
    """

    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "agilex-tracer"), "power")

    def __init__(self, name: str):
        super().__init__(name)
        self._client: Optional[TracerCanClient] = None
        self._backend = "socketcan"
        self._channel = "can0"
        self._bitrate = 500000
        self._charge_threshold_a = proto.DEFAULT_CHARGING_CURRENT_THRESHOLD_A

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
        threshold = _attr_float(
            attrs,
            "charging_current_threshold_amps",
            proto.DEFAULT_CHARGING_CURRENT_THRESHOLD_A,
        )
        if threshold < 0:
            raise Exception("charging_current_threshold_amps must be >= 0")
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        attrs = struct_to_dict(config.attributes)
        backend, channel, bitrate, auto_up = parse_can_attrs(attrs)
        self._charge_threshold_a = _attr_float(
            attrs,
            "charging_current_threshold_amps",
            proto.DEFAULT_CHARGING_CURRENT_THRESHOLD_A,
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
            "Tracer power on %s:%s@%d charge_threshold=%.2fA",
            self._backend,
            self._channel,
            self._bitrate,
            self._charge_threshold_a,
        )

    async def close(self):
        if self._client is not None:
            release_client(self._client)
            self._client = None

    def _require_client(self) -> TracerCanClient:
        if self._client is None:
            raise RuntimeError("Tracer CAN client is not configured")
        return self._client

    def _snapshot(self) -> proto.TracerState:
        return self._require_client().snapshot()

    def _bms_fresh(self, bms: proto.BmsBasic) -> bool:
        return bms.last_update > 0 and (time.time() - bms.last_update) < _BMS_STALE_S

    def _volts(self, state: proto.TracerState) -> float:
        bms = state.bms_basic
        if self._bms_fresh(bms) and bms.voltage > 0:
            return float(bms.voltage)
        return float(state.system.battery_voltage)

    def _amps(self, state: proto.TracerState) -> float:
        bms = state.bms_basic
        if self._bms_fresh(bms):
            return float(bms.current_a)
        return 0.0

    async def get_voltage(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[float, bool]:
        return self._volts(self._snapshot()), False

    async def get_current(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[float, bool]:
        return self._amps(self._snapshot()), False

    async def get_power(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> float:
        state = self._snapshot()
        return self._volts(state) * self._amps(state)

    async def get_readings(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, SensorReading]:
        state = self._snapshot()
        system = state.system
        bms = state.bms_basic
        ext = state.bms_extended
        volts = self._volts(state)
        amps = self._amps(state)
        bms_age = (time.time() - bms.last_update) if bms.last_update > 0 else -1.0
        return {
            "volts": volts,
            "amps": amps,
            "watts": volts * amps,
            "is_ac": False,
            "is_charging": bms.is_charging(self._charge_threshold_a),
            "soc_percent": float(bms.soc_percent) if self._bms_fresh(bms) else 0.0,
            "soh_percent": float(bms.soh_percent) if self._bms_fresh(bms) else 0.0,
            "temperature_c": float(bms.temperature_c) if self._bms_fresh(bms) else 0.0,
            "bms_age_s": bms_age,
            "charging_current_threshold_amps": self._charge_threshold_a,
            "vehicle_state": int(system.vehicle_state),
            "control_mode": int(system.control_mode),
            "control_mode_name": control_mode_name(system.control_mode),
            "fault_bits": int(system.fault_bits),
            "emergency_stop": bool(system.emergency_stop),
            "under_voltage": bool(system.under_voltage),
            "under_voltage_alarm": bool(system.under_voltage_alarm),
            "bms_alarm_status_1": int(ext.alarm_status_1),
            "bms_alarm_status_2": int(ext.alarm_status_2),
            "bms_warn_status_1": int(ext.warn_status_1),
            "bms_warn_status_2": int(ext.warn_status_2),
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
            state = self._snapshot()
            system = state.system
            bms = state.bms_basic
            return {
                "battery_voltage": self._volts(state),
                "amps": self._amps(state),
                "watts": self._volts(state) * self._amps(state),
                "is_charging": bms.is_charging(self._charge_threshold_a),
                "soc_percent": float(bms.soc_percent),
                "soh_percent": float(bms.soh_percent),
                "temperature_c": float(bms.temperature_c),
                "bms_fresh": self._bms_fresh(bms),
                "charging_current_threshold_amps": self._charge_threshold_a,
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
