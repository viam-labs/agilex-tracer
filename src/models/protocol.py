"""AgileX Tracer / Tracer 2.0 CAN protocol (CAN 2.0B @ 500 kbit/s, Motorola/big-endian).

Frame IDs and packing match the TRACER 2.0 User Manual §3.3.1 and the
agilexrobotics/tracer_ros2 + ugv_sdk Protocol V2 conventions.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

# --- Command / feedback IDs -------------------------------------------------

CAN_ID_MOTION_CMD = 0x111
CAN_ID_LIGHT_CMD = 0x121
CAN_ID_SYSTEM_STATUS = 0x211
CAN_ID_MOTION_FEEDBACK = 0x221
CAN_ID_LIGHT_FEEDBACK = 0x231
CAN_ID_ODOMETER = 0x311
CAN_ID_CONTROL_MODE = 0x421
CAN_ID_CLEAR_FAULT = 0x441
CAN_ID_MOTOR_HS_1 = 0x251
CAN_ID_MOTOR_HS_2 = 0x252
CAN_ID_MOTOR_LS_1 = 0x261
CAN_ID_MOTOR_LS_2 = 0x262

MOTION_PERIOD_S = 0.02  # 20 ms keepalive (chassis times out at 500 ms)
LIGHT_PERIOD_S = 0.025  # 25 ms keepalive when light control is enabled

# Chassis limits from the manual (control frame effective ranges).
MAX_LINEAR_MM_S = 1800
MAX_ANGULAR_MRAD_S = 1000  # 0.001 rad/s units → ±1.0 rad/s

# Defaults from TRACER 2.0 mechanical specs.
DEFAULT_TRACK_WIDTH_M = 0.5174
# Approximate hub-wheel circumference; override via config for accuracy.
DEFAULT_WHEEL_CIRCUMFERENCE_M = 0.518


class VehicleState(IntEnum):
    NORMAL = 0x00
    EMERGENCY_STOP = 0x01
    SYSTEM_ABNORMAL = 0x02


class ControlMode(IntEnum):
    STANDBY = 0x00
    CAN_COMMAND = 0x01
    REMOTE = 0x02


class LightMode(IntEnum):
    OFF = 0x00
    ON = 0x01
    BREATHE = 0x02
    CUSTOM = 0x03


@dataclass
class SystemStatus:
    vehicle_state: int = 0
    control_mode: int = 0
    battery_voltage: float = 0.0
    fault_bits: int = 0
    count: int = 0
    last_update: float = 0.0

    @property
    def under_voltage(self) -> bool:
        return bool(self.fault_bits & (1 << 8))

    @property
    def under_voltage_alarm(self) -> bool:
        return bool(self.fault_bits & (1 << 9))

    @property
    def emergency_stop(self) -> bool:
        return bool(self.fault_bits & (1 << 15)) or self.vehicle_state == VehicleState.EMERGENCY_STOP


@dataclass
class MotionFeedback:
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0
    last_update: float = 0.0


@dataclass
class LightFeedback:
    enabled: bool = False
    mode: int = 0
    brightness: int = 0
    count: int = 0
    last_update: float = 0.0


@dataclass
class WheelOdometer:
    left_mm: int = 0
    right_mm: int = 0
    last_update: float = 0.0


@dataclass
class MotorRpm:
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    last_update: float = 0.0


@dataclass
class TracerState:
    system: SystemStatus = field(default_factory=SystemStatus)
    motion: MotionFeedback = field(default_factory=MotionFeedback)
    light: LightFeedback = field(default_factory=LightFeedback)
    odometer: WheelOdometer = field(default_factory=WheelOdometer)
    motor_rpm: MotorRpm = field(default_factory=MotorRpm)


def encode_motion(linear_m_s: float, angular_rad_s: float) -> bytes:
    """Pack motion command 0x111: linear mm/s and angular 0.001 rad/s."""
    linear_mm_s = int(round(linear_m_s * 1000.0))
    angular_mrad_s = int(round(angular_rad_s * 1000.0))
    linear_mm_s = max(-MAX_LINEAR_MM_S, min(MAX_LINEAR_MM_S, linear_mm_s))
    angular_mrad_s = max(-MAX_ANGULAR_MRAD_S, min(MAX_ANGULAR_MRAD_S, angular_mrad_s))
    return struct.pack(">hh", linear_mm_s, angular_mrad_s) + bytes(4)


def encode_light(enable: bool, mode: int, brightness: int, count: int) -> bytes:
    """Pack light command 0x121 (Protocol V2 LightCommandFrame).

    Layout matches ugv_sdk: enable, front_mode, front_custom, rear_mode,
    rear_custom, reserved, reserved, count.
    """
    brightness = max(0, min(100, int(brightness)))
    mode = int(mode) & 0xFF
    data = bytearray(8)
    data[0] = 0x01 if enable else 0x00
    data[1] = mode
    data[2] = brightness
    # Tracer has front light only; keep rear at CONST_OFF.
    data[3] = 0x00
    data[4] = 0x00
    data[5] = 0x00
    data[6] = 0x00
    data[7] = count & 0xFF
    return bytes(data)


def encode_control_mode(mode: int) -> bytes:
    """Pack control-mode command 0x421 (1-byte enable; pad to 8 for bus tools)."""
    data = bytearray(8)
    data[0] = int(mode) & 0xFF
    return bytes(data)


def encode_clear_fault(which: int = 0) -> bytes:
    """0 = clear all, 1 = motor 1, 2 = motor 2."""
    return bytes([int(which) & 0xFF])


def decode_battery_voltage(raw: int) -> float:
    """Convert system-status battery raw count to volts.

    Manual / ugv_sdk: actual voltage × 10 (0.1 V resolution). Some Tracer
    firmwares appear to send × 100 (0.01 V); a 24 V pack never exceeds ~30 V,
    so values that decode above 40 V with ×10 are treated as ×100.
    """
    raw = int(raw) & 0xFFFF
    volts = raw / 10.0
    if volts > 40.0:
        volts = raw / 100.0
    return volts


def decode_system_status(data: bytes, now: float) -> Optional[SystemStatus]:
    if len(data) < 8:
        return None
    voltage_raw = struct.unpack(">H", data[2:4])[0]
    fault = struct.unpack(">H", data[4:6])[0]
    return SystemStatus(
        vehicle_state=data[0],
        control_mode=data[1],
        battery_voltage=decode_battery_voltage(voltage_raw),
        fault_bits=fault,
        count=data[7],
        last_update=now,
    )


def decode_motion_feedback(data: bytes, now: float) -> Optional[MotionFeedback]:
    if len(data) < 4:
        return None
    linear_mm_s, angular_mrad_s = struct.unpack(">hh", data[0:4])
    return MotionFeedback(
        linear_m_s=linear_mm_s / 1000.0,
        angular_rad_s=angular_mrad_s / 1000.0,
        last_update=now,
    )


def decode_light_feedback(data: bytes, now: float) -> Optional[LightFeedback]:
    if len(data) < 8:
        return None
    return LightFeedback(
        enabled=bool(data[0]),
        mode=data[1],
        brightness=data[2],
        count=data[7],
        last_update=now,
    )


def decode_odometer(data: bytes, now: float) -> Optional[WheelOdometer]:
    if len(data) < 8:
        return None
    left_mm, right_mm = struct.unpack(">ii", data[0:8])
    return WheelOdometer(left_mm=left_mm, right_mm=right_mm, last_update=now)


def decode_motor_rpm(data: bytes) -> Optional[float]:
    if len(data) < 2:
        return None
    (rpm,) = struct.unpack(">h", data[0:2])
    return float(rpm)
