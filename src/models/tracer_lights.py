"""Generic component for AgileX Tracer front light control via DoCommand."""

from __future__ import annotations

import asyncio
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.generic import Generic
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from .can_client import TracerCanClient, get_client, parse_can_attrs, release_client
from . import protocol as proto

_MODE_ALIASES = {
    "off": proto.LightMode.OFF,
    "on": proto.LightMode.ON,
    "always_on": proto.LightMode.ON,
    "always_off": proto.LightMode.OFF,
    "breathe": proto.LightMode.BREATHE,
    "breathing": proto.LightMode.BREATHE,
    "custom": proto.LightMode.CUSTOM,
    "brightness": proto.LightMode.CUSTOM,
}

_MODE_NAMES = {
    int(proto.LightMode.OFF): "off",
    int(proto.LightMode.ON): "on",
    int(proto.LightMode.BREATHE): "breathe",
    int(proto.LightMode.CUSTOM): "custom",
}


def _parse_mode(value: Any) -> int:
    if isinstance(value, bool):
        return proto.LightMode.ON if value else proto.LightMode.OFF
    if isinstance(value, (int, float)):
        return int(value) & 0xFF
    text = str(value).strip().lower()
    if text in _MODE_ALIASES:
        return int(_MODE_ALIASES[text])
    if text.isdigit():
        return int(text) & 0xFF
    raise Exception(
        f"unknown light mode '{value}'. Use off, on, breathe, custom, or 0-3"
    )


def _as_int(value: Any, default: int) -> int:
    if value is None:
        return default
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


class TracerLights(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("viam-labs", "agilex-tracer"), "lights")

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
        lights = cls(config.name)
        lights.reconfigure(config, dependencies)
        return lights

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
        backend, channel, bitrate = parse_can_attrs(attrs)
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
        self.logger.info(
            "Tracer lights on %s:%s@%d", self._backend, self._channel, self._bitrate
        )

    async def close(self):
        if self._client is not None:
            self._client.set_light(False, proto.LightMode.OFF, 0)
            release_client(self._client)
            self._client = None

    def _require_client(self) -> TracerCanClient:
        if self._client is None:
            raise RuntimeError("Tracer CAN client is not configured")
        return self._client

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        client = self._require_client()
        name = str(command.get("command", "")).strip().lower()

        if name in ("set_light", "set", "light"):
            mode = _parse_mode(command.get("mode", "on"))
            brightness = _as_int(command.get("brightness", 100), 100)
            enable = command.get("enable", True)
            if isinstance(enable, str):
                enable_b = enable.strip().lower() not in ("0", "false", "no", "off")
            else:
                enable_b = bool(enable) if enable is not None else True
            if mode == proto.LightMode.OFF:
                enable_b = True  # enabled control with OFF mode
            await asyncio.to_thread(client.set_light, enable_b, mode, brightness)
            return {
                "ok": True,
                "enable": enable_b,
                "mode": int(mode),
                "mode_name": _MODE_NAMES.get(int(mode), str(mode)),
                "brightness": max(0, min(100, brightness)),
                "hint": (
                    "If mode does not change, set remote SWC away from breathe "
                    "(SWC up) — RC light switch can override CAN light commands."
                ),
            }

        if name in ("off", "lights_off"):
            await asyncio.to_thread(client.set_light, True, proto.LightMode.OFF, 0)
            return {
                "ok": True,
                "mode": int(proto.LightMode.OFF),
                "mode_name": "off",
            }

        if name in ("on", "lights_on"):
            await asyncio.to_thread(client.set_light, True, proto.LightMode.ON, 100)
            return {
                "ok": True,
                "mode": int(proto.LightMode.ON),
                "mode_name": "on",
            }

        if name in ("breathe", "breathing"):
            await asyncio.to_thread(client.set_light, True, proto.LightMode.BREATHE, 0)
            return {
                "ok": True,
                "mode": int(proto.LightMode.BREATHE),
                "mode_name": "breathe",
            }

        if name in ("status", "get_status"):
            state = client.snapshot()
            mode = int(state.light.mode)
            return {
                "enabled": state.light.enabled,
                "mode": mode,
                "mode_name": _MODE_NAMES.get(mode, str(mode)),
                "brightness": state.light.brightness,
                "can_backend": self._backend,
                "can_channel": self._channel,
                "can_bitrate": self._bitrate,
            }


        if name == "release":
            # Stop keepalive so chassis can time out light control.
            await asyncio.to_thread(client.set_light, False, proto.LightMode.OFF, 0)
            return {"ok": True, "released": True}

        raise Exception(
            "unknown command. Supported: set_light, on, off, breathe, get_status, release"
        )
