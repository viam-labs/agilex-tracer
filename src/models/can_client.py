"""Shared SocketCAN / slcan client for AgileX Tracer chassis components.

Linux only. Multiple Viam resources (base, odometry, lights) with the same bus
key share one bus handle, RX thread, and motion/light keepalive TX loops via
refcounting.

Backends (python-can):
- socketcan — Linux kernel CAN (can0); used by AgileX bring-up scripts
- slcan — LAWICEL serial adapters (/dev/ttyUSB*, /dev/ttyACM*)
- auto — serial path → slcan; otherwise socketcan
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, Mapping, Optional, Tuple

from . import protocol as proto

DEFAULT_BITRATE = 500000

_CONTROL_MODE_NAMES = {
    int(proto.ControlMode.STANDBY): "standby",
    int(proto.ControlMode.CAN_COMMAND): "can_command",
    int(proto.ControlMode.REMOTE): "remote",
}


def control_mode_name(mode: int) -> str:
    return _CONTROL_MODE_NAMES.get(int(mode), str(mode))


def _socketcan_iface_hint(channel: str) -> str:
    return (
        f"SocketCAN interface '{channel}' is down or not configured. Bring it up:\n"
        f"  sudo modprobe gs_usb\n"
        f"  sudo ip link set {channel} down\n"
        f"  sudo ip link set {channel} type can bitrate 500000\n"
        f"  sudo ip link set {channel} up\n"
        f"  ip link show {channel}"
    )


def _socketcan_operstate(channel: str) -> Optional[str]:
    path = f"/sys/class/net/{channel}/operstate"
    try:
        with open(path, encoding="utf-8") as fh:
            return fh.read().strip().lower()
    except OSError:
        return None


def _is_network_down(exc: BaseException) -> bool:
    text = str(exc).lower()
    if "network is down" in text or "enetdown" in text:
        return True
    errno = getattr(exc, "errno", None)
    if errno == 100:  # ENETDOWN on Linux
        return True
    cause = getattr(exc, "__cause__", None) or getattr(exc, "__context__", None)
    if isinstance(cause, BaseException) and cause is not exc:
        return _is_network_down(cause)
    return False


def _run_cmd(argv: list[str]) -> Tuple[int, str]:
    import subprocess

    try:
        proc = subprocess.run(
            argv,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)
    out = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, out.strip()


def _try_bringup_socketcan(channel: str, bitrate: int, log: Any = None) -> bool:
    """Best-effort bring-up of a SocketCAN iface (needs CAP_NET_ADMIN / root)."""

    def _log(msg: str, *args: Any) -> None:
        if log is None:
            return
        fn = getattr(log, "info", None) or getattr(log, "warning", None)
        if fn is not None:
            fn(msg, *args)

    _run_cmd(["modprobe", "gs_usb"])
    # Configure + up. `down` first clears a stale bitrate when re-plugging.
    _run_cmd(["ip", "link", "set", channel, "down"])
    code, out = _run_cmd(
        [
            "ip",
            "link",
            "set",
            channel,
            "up",
            "type",
            "can",
            "bitrate",
            str(int(bitrate)),
        ]
    )
    if code != 0:
        # Fallback without combined "up type can" (older iproute2).
        _run_cmd(
            [
                "ip",
                "link",
                "set",
                channel,
                "type",
                "can",
                "bitrate",
                str(int(bitrate)),
            ]
        )
        code, out = _run_cmd(["ip", "link", "set", channel, "up"])
    state = _socketcan_operstate(channel)
    ok = state in ("up", "unknown")
    if ok:
        _log("Brought up SocketCAN %s (operstate=%s)", channel, state)
    else:
        _log(
            "Could not bring up SocketCAN %s (operstate=%s, ip exit=%s): %s",
            channel,
            state,
            code,
            out or "(no output)",
        )
    return ok


def _attr_bool(attrs: Mapping[str, Any], key: str, default: bool) -> bool:
    if key not in attrs or attrs[key] is None:
        return default
    value = attrs[key]
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    return default


_registry_lock = threading.Lock()
_clients: Dict[str, "TracerCanClient"] = {}


def resolve_backend(backend: str, channel: str) -> str:
    """Normalize backend name; auto-select from channel when needed."""
    name = (backend or "auto").strip().lower()
    aliases = {
        "socketcan": "socketcan",
        "socket": "socketcan",
        "can": "socketcan",
        "slcan": "slcan",
        "serial": "slcan",
        "auto": "auto",
    }
    if name not in aliases:
        raise ValueError(
            f"unknown can_backend '{backend}'. Use socketcan, slcan, or auto "
            "(Linux only; macOS is not supported)"
        )
    resolved = aliases[name]
    if resolved != "auto":
        return resolved

    ch = (channel or "").strip()
    lower = ch.lower()
    if lower.startswith("/dev/") or lower.startswith("com") or "tty" in lower:
        return "slcan"
    return "socketcan"


def client_registry_key(backend: str, channel: str, bitrate: int) -> str:
    return f"{backend}:{channel}:{int(bitrate)}"


def parse_can_attrs(attrs: Mapping[str, Any]) -> Tuple[str, str, int, bool]:
    """Return (backend, channel, bitrate, can_auto_up) from component attributes."""
    backend = "auto"
    if "can_backend" in attrs and attrs["can_backend"] is not None:
        backend = str(attrs["can_backend"]).strip() or "auto"
    elif "backend" in attrs and attrs["backend"] is not None:
        backend = str(attrs["backend"]).strip() or "auto"

    channel = "can0"
    if "can_channel" in attrs and attrs["can_channel"] is not None:
        channel = str(attrs["can_channel"]).strip() or "can0"
    elif "can_interface" in attrs and attrs["can_interface"] is not None:
        channel = str(attrs["can_interface"]).strip() or "can0"
    elif "channel" in attrs and attrs["channel"] is not None:
        channel = str(attrs["channel"]).strip() or "can0"

    bitrate = DEFAULT_BITRATE
    for key in ("can_bitrate", "bitrate"):
        if key in attrs and attrs[key] is not None:
            bitrate = int(attrs[key])
            break

    # Default on: viam-server on robots is usually root and can0 is often down
    # until something brings it up after reboot / dongle re-plug.
    auto_up = _attr_bool(attrs, "can_auto_up", True)

    backend = resolve_backend(backend, channel)
    if not channel:
        raise ValueError("can_channel / can_interface must be non-empty")
    if bitrate <= 0:
        raise ValueError("can_bitrate must be > 0")
    return backend, channel, bitrate, auto_up


def get_client(
    backend: str,
    channel: str,
    bitrate: int = DEFAULT_BITRATE,
    logger: Any = None,
    auto_up: bool = True,
) -> "TracerCanClient":
    backend = resolve_backend(backend, channel)
    key = client_registry_key(backend, channel, bitrate)
    with _registry_lock:
        client = _clients.get(key)
        if client is None:
            client = TracerCanClient(
                backend, channel, bitrate, logger=logger, auto_up=auto_up
            )
            _clients[key] = client
        client._retain()
        if logger is not None:
            client.logger = logger
        return client


def release_client(client: "TracerCanClient") -> None:
    with _registry_lock:
        client._release()
        if client._refcount <= 0:
            key = client_registry_key(client.backend, client.channel, client.bitrate)
            client._shutdown()
            _clients.pop(key, None)


class TracerCanClient:
    def __init__(
        self,
        backend: str,
        channel: str,
        bitrate: int = DEFAULT_BITRATE,
        logger: Any = None,
        auto_up: bool = True,
    ):
        self.backend = backend
        self.channel = channel
        self.bitrate = int(bitrate)
        self.auto_up = bool(auto_up)
        # Back-compat alias used in status/do_command responses.
        self.interface = channel
        self.logger = logger
        self._refcount = 0
        self._bus = None
        self._state = proto.TracerState()
        self._state_lock = threading.Lock()

        self._cmd_lock = threading.Lock()
        self._linear_m_s = 0.0
        self._angular_rad_s = 0.0
        self._motion_enabled = False

        self._light_enable = False
        self._light_mode = int(proto.LightMode.OFF)
        self._light_brightness = 0
        self._light_count = 0
        self._light_enabled_tx = False

        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._motion_thread: Optional[threading.Thread] = None
        self._light_thread: Optional[threading.Thread] = None
        self._opened = False

    def _retain(self) -> None:
        self._refcount += 1
        if not self._opened:
            self._open()

    def _release(self) -> None:
        self._refcount = max(0, self._refcount - 1)

    def _log(self, level: str, msg: str, *args: Any) -> None:
        if self.logger is None:
            return
        fn = getattr(self.logger, level, None)
        if fn is not None:
            fn(msg, *args)

    def _open_bus(self):
        import can

        if self.backend == "socketcan":
            try:
                import can.interfaces.socketcan  # noqa: F401
            except ImportError as e:
                raise RuntimeError(
                    "SocketCAN backend unavailable. This module requires Linux "
                    "with a configured can0 interface (or can_backend=slcan)."
                ) from e
            state = _socketcan_operstate(self.channel)
            if self.auto_up and state != "up":
                _try_bringup_socketcan(self.channel, self.bitrate, self.logger)
                state = _socketcan_operstate(self.channel)
            try:
                return can.Bus(
                    channel=self.channel, interface="socketcan", bitrate=self.bitrate
                )
            except Exception as e:
                if _is_network_down(e):
                    raise RuntimeError(_socketcan_iface_hint(self.channel)) from e
                raise


        if self.backend == "slcan":
            try:
                import serial  # noqa: F401
                import can.interfaces.slcan  # noqa: F401
            except ImportError as e:
                raise RuntimeError(
                    "slcan backend requires pyserial "
                    "(pip install pyserial). Set can_channel to the serial device "
                    "e.g. /dev/ttyUSB0 or /dev/ttyACM0."
                ) from e
            return can.Bus(
                channel=self.channel,
                interface="slcan",
                bitrate=self.bitrate,
            )

        raise RuntimeError(f"unsupported CAN backend: {self.backend}")

    def _open(self) -> None:
        try:
            import can  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                "python-can is not installed. Re-run setup.sh / first_run."
            ) from e

        self._stop_event.clear()
        self._bus = self._open_bus()
        label = f"{self.backend}:{self.channel}@{self.bitrate}"
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name=f"tracer-rx-{label}", daemon=True
        )
        self._motion_thread = threading.Thread(
            target=self._motion_tx_loop,
            name=f"tracer-motion-tx-{label}",
            daemon=True,
        )
        self._light_thread = threading.Thread(
            target=self._light_tx_loop,
            name=f"tracer-light-tx-{label}",
            daemon=True,
        )
        self._rx_thread.start()
        self._motion_thread.start()
        self._light_thread.start()
        self._opened = True
        self._log("info", "Opened Tracer CAN %s", label)
        try:
            self.enable_can_control()
        except Exception as exc:
            self._log("warning", "Failed to enable CAN control mode: %s", exc)

    def _shutdown(self) -> None:
        self.set_motion(0.0, 0.0, enable=False)
        self.set_light(False, proto.LightMode.OFF, 0)
        self._stop_event.set()
        bus = self._bus
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass
        for thread in (self._rx_thread, self._motion_thread, self._light_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)
        self._rx_thread = None
        self._motion_thread = None
        self._light_thread = None
        self._bus = None
        self._opened = False
        self._log(
            "info",
            "Closed Tracer CAN %s:%s",
            self.backend,
            self.channel,
        )

    def _send(self, can_id: int, data: bytes) -> None:
        import can

        bus = self._bus
        if bus is None:
            raise RuntimeError(
                f"CAN bus {self.backend}:{self.channel} is not open"
            )
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            bus.send(msg, timeout=0.05)
        except Exception as e:
            if self.backend == "socketcan" and _is_network_down(e):
                raise RuntimeError(_socketcan_iface_hint(self.channel)) from e
            raise

    def enable_can_control(self) -> None:
        self._send(
            proto.CAN_ID_CONTROL_MODE,
            proto.encode_control_mode(proto.ControlMode.CAN_COMMAND),
        )

    def clear_faults(self, which: int = 0) -> None:
        self._send(proto.CAN_ID_CLEAR_FAULT, proto.encode_clear_fault(which))

    def set_motion(
        self, linear_m_s: float, angular_rad_s: float, enable: bool = True
    ) -> None:
        with self._cmd_lock:
            self._linear_m_s = float(linear_m_s)
            self._angular_rad_s = float(angular_rad_s)
            self._motion_enabled = bool(enable) and (
                abs(self._linear_m_s) > 1e-6 or abs(self._angular_rad_s) > 1e-6
            )
            linear = self._linear_m_s
            angular = self._angular_rad_s
            want_motion = self._motion_enabled
        try:
            # Chassis ignores 0x111 unless control mode is CAN command.
            if want_motion:
                self.enable_can_control()
            self._send(proto.CAN_ID_MOTION_CMD, proto.encode_motion(linear, angular))
        except Exception as exc:
            self._log("warning", "motion command send failed: %s", exc)


    def set_light(self, enable: bool, mode: int, brightness: int = 0) -> None:
        with self._cmd_lock:
            self._light_enable = bool(enable)
            self._light_mode = int(mode)
            self._light_brightness = int(brightness)
            # Keep transmitting while enabled so the 500ms chassis timeout does not
            # drop us back to RC light control.
            self._light_enabled_tx = bool(enable)
            self._light_count = (self._light_count + 1) & 0xFF
            enable_b = self._light_enable
            mode_i = self._light_mode
            brightness_i = self._light_brightness
            count = self._light_count
        # Push immediately (same pattern as motion) so the first command is not
        # delayed by the 25ms TX loop.
        try:
            self._send(
                proto.CAN_ID_LIGHT_CMD,
                proto.encode_light(enable_b, mode_i, brightness_i, count),
            )
            self._log(
                "info",
                "light cmd enable=%s mode=%s brightness=%s count=%s",
                enable_b,
                mode_i,
                brightness_i,
                count,
            )
        except Exception as exc:
            self._log("warning", "light command send failed: %s", exc)

    def snapshot(self) -> proto.TracerState:
        with self._state_lock:
            s = self._state
            return proto.TracerState(
                system=proto.SystemStatus(
                    vehicle_state=s.system.vehicle_state,
                    control_mode=s.system.control_mode,
                    battery_voltage=s.system.battery_voltage,
                    fault_bits=s.system.fault_bits,
                    count=s.system.count,
                    last_update=s.system.last_update,
                ),
                motion=proto.MotionFeedback(
                    linear_m_s=s.motion.linear_m_s,
                    angular_rad_s=s.motion.angular_rad_s,
                    last_update=s.motion.last_update,
                ),
                light=proto.LightFeedback(
                    enabled=s.light.enabled,
                    mode=s.light.mode,
                    brightness=s.light.brightness,
                    count=s.light.count,
                    last_update=s.light.last_update,
                ),
                odometer=proto.WheelOdometer(
                    left_mm=s.odometer.left_mm,
                    right_mm=s.odometer.right_mm,
                    last_update=s.odometer.last_update,
                ),
                motor_rpm=proto.MotorRpm(
                    left_rpm=s.motor_rpm.left_rpm,
                    right_rpm=s.motor_rpm.right_rpm,
                    last_update=s.motor_rpm.last_update,
                ),
            )

    def commanded_motion(self) -> Tuple[float, float, bool]:
        with self._cmd_lock:
            return self._linear_m_s, self._angular_rad_s, self._motion_enabled

    def _rx_loop(self) -> None:
        bus = self._bus
        if bus is None:
            return
        while not self._stop_event.is_set():
            try:
                msg = bus.recv(timeout=0.1)
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                self._log("warning", "CAN recv error: %s", exc)
                time.sleep(0.05)
                continue
            if msg is None:
                continue
            self._handle_message(msg.arbitration_id, bytes(msg.data))

    def _handle_message(self, can_id: int, data: bytes) -> None:
        now = time.time()
        with self._state_lock:
            if can_id == proto.CAN_ID_SYSTEM_STATUS:
                decoded = proto.decode_system_status(data, now)
                if decoded is not None:
                    self._state.system = decoded
            elif can_id == proto.CAN_ID_MOTION_FEEDBACK:
                decoded = proto.decode_motion_feedback(data, now)
                if decoded is not None:
                    self._state.motion = decoded
            elif can_id == proto.CAN_ID_LIGHT_FEEDBACK:
                decoded = proto.decode_light_feedback(data, now)
                if decoded is not None:
                    self._state.light = decoded
            elif can_id == proto.CAN_ID_ODOMETER:
                decoded = proto.decode_odometer(data, now)
                if decoded is not None:
                    self._state.odometer = decoded
            elif can_id == proto.CAN_ID_MOTOR_HS_1:
                rpm = proto.decode_motor_rpm(data)
                if rpm is not None:
                    self._state.motor_rpm.left_rpm = rpm
                    self._state.motor_rpm.last_update = now
            elif can_id == proto.CAN_ID_MOTOR_HS_2:
                rpm = proto.decode_motor_rpm(data)
                if rpm is not None:
                    self._state.motor_rpm.right_rpm = rpm
                    self._state.motor_rpm.last_update = now

    def _motion_tx_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._cmd_lock:
                enabled = self._motion_enabled
                linear = self._linear_m_s
                angular = self._angular_rad_s
            if enabled:
                try:
                    # Keep 0x421 alive; some chassis drop CAN mode after ~500ms.
                    self.enable_can_control()
                    self._send(
                        proto.CAN_ID_MOTION_CMD, proto.encode_motion(linear, angular)
                    )
                except Exception as exc:
                    self._log("warning", "motion TX failed: %s", exc)
            self._stop_event.wait(proto.MOTION_PERIOD_S)

    def _light_tx_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._cmd_lock:
                tx = self._light_enabled_tx
                enable = self._light_enable
                mode = self._light_mode
                brightness = self._light_brightness
                if tx:
                    # ugv_sdk increments count on every encoded frame.
                    self._light_count = (self._light_count + 1) & 0xFF
                count = self._light_count
            if tx:
                try:
                    self._send(
                        proto.CAN_ID_LIGHT_CMD,
                        proto.encode_light(enable, mode, brightness, count),
                    )
                except Exception as exc:
                    self._log("warning", "light TX failed: %s", exc)
            self._stop_event.wait(proto.LIGHT_PERIOD_S)


def clamp_linear_angular(
    linear_m_s: float,
    angular_rad_s: float,
    max_linear_m_s: float,
    max_angular_rad_s: float,
) -> Tuple[float, float]:
    max_lin = abs(max_linear_m_s)
    max_ang = abs(max_angular_rad_s)
    if max_lin > 0:
        linear_m_s = max(-max_lin, min(max_lin, linear_m_s))
    if max_ang > 0:
        angular_rad_s = max(-max_ang, min(max_ang, angular_rad_s))
    if not math.isfinite(linear_m_s):
        linear_m_s = 0.0
    if not math.isfinite(angular_rad_s):
        angular_rad_s = 0.0
    return linear_m_s, angular_rad_s
