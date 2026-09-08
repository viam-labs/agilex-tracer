# AgileX Tracer (Viam module)

Drivers for the [AgileX Tracer / Tracer 2.0](https://www.agilex.ai/) differential mobile base over a CAN-to-USB adapter (SocketCAN). No ROS required.

Protocol reference: [TRACER 2.0 User Manual](https://cdn.shopify.com/s/files/1/0551/0630/6141/files/TRACER_2.0_User_Manual.pdf) (CAN §3.3). ROS reference implementation: [agilexrobotics/tracer_ros2](https://github.com/agilexrobotics/tracer_ros2).

## Models

| Model | API | Purpose |
| --- | --- | --- |
| `viam-labs:agilex-tracer:base` | `rdk:component:base` | Drive the chassis (`SetPower`, `SetVelocity`, `Stop`, open-loop `MoveStraight` / `Spin`) |
| `viam-labs:agilex-tracer:odometry` | `rdk:component:movement_sensor` | Wheeled odometry from left/right tire odometers (0x311) + velocity feedback (0x221) |
| `viam-labs:agilex-tracer:lights` | `rdk:component:generic` | Front light control via `DoCommand` |

Wrap the base + odometry with Viam’s builtin [`sensor-controlled`](https://docs.viam.com/reference/components/base/sensor-controlled/) base for closed-loop `SetVelocity` / `MoveStraight` / `Spin`.

## CAN setup

**Linux only** (SocketCAN). Tracer talks CAN 2.0B @ **500 kbit/s**.

| `can_backend` | Typical hardware |
| --- | --- |
| `socketcan` (default) | AgileX USB-CAN with `gs_usb` kernel module → `can0` |
| `slcan` | Adapter in SLCAN/serial mode (`/dev/ttyUSB0`, `/dev/ttyACM0`) |
| `auto` | Serial path → `slcan`; otherwise `socketcan` |

### SocketCAN — AgileX scripts

```bash
sudo modprobe gs_usb
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
candump can0
```

### slcan

If the dongle shows up as a serial port:

```json
{
  "can_backend": "slcan",
  "can_channel": "/dev/ttyUSB0",
  "can_bitrate": 500000
}
```

Put the remote in **command / navigation** mode (SWB up on the FS transmitter), or power the transmitter off. Lights can change over CAN while the remote still owns motion — if `get_status` shows `control_mode` / `control_mode_name` as `remote` (2) or `standby` (0), the base will ignore velocity until mode is `can_command` (1). The module sends `0x421` with motion keepalives; still need SWB/command mode when the remote is on.

**Safety:** keep the E-stop and remote ready. Motion commands must be refreshed within 500 ms or the chassis stops — the module keepalive runs at 20 ms while moving.

## Local module

```bash
git clone https://github.com/viam-labs/agilex-tracer.git
cd agilex-tracer
chmod +x run.sh setup.sh build.sh
./run.sh   # or add as a local module pointing at run.sh
```

Registry packages are a Linux source tarball (`run.sh` + venv via `setup.sh`); no PyInstaller binary. macOS is not supported.

## Machine configuration

See [`example_config.json`](example_config.json). Minimal attributes:

### Tracer base

```json
{
  "name": "tracer-base",
  "api": "rdk:component:base",
  "model": "viam-labs:agilex-tracer:base",
  "attributes": {
    "can_backend": "auto",
    "can_interface": "can0",
    "width_meters": 0.5174,
    "wheel_circumference_meters": 0.518
  }
}
```

| Attribute | Default | Notes |
| --- | --- | --- |
| `can_backend` | `auto` | `socketcan`, `slcan`, or `auto` |
| `can_interface` / `can_channel` | `can0` | SocketCAN name or slcan serial path |
| `can_bitrate` | `500000` | Must match chassis (500 kbit/s) |
| `width_meters` | `0.5174` | Track width (manual wheelbase) |
| `wheel_circumference_meters` | `0.518` | Used in `GetProperties` |
| `max_linear_m_s` | `1.8` | Clamp for power/velocity |
| `max_angular_rad_s` | `1.0` | Clamp for power/velocity |

`SetVelocity`: linear **mm/s** on **Y** (forward; X accepted as fallback), angular **deg/s** on **Z**.

Base `DoCommand` helpers: `enable_can_control`, `clear_faults`, `get_status`.

### Tracer odometry

```json
{
  "name": "tracer-odom",
  "api": "rdk:component:movement_sensor",
  "model": "viam-labs:agilex-tracer:odometry",
  "attributes": {
    "can_backend": "auto",
    "can_interface": "can0",
    "width_meters": 0.5174,
    "time_interval_msec": 50
  }
}
```

Reports `LinearVelocity` (Y, m/s), `AngularVelocity` (Z, deg/s), `Orientation` (yaw), and `Position` (geo encoding compatible with sensor-controlled `MoveStraight`), matching builtin [wheeled-odometry](https://docs.viam.com/reference/components/movement-sensor/wheeled-odometry/) conventions.

`DoCommand`: `reset_odometry`, `set_origin` (`lat` / `long`).

### Sensor-controlled wrapper

```json
{
  "name": "tracer-sc",
  "api": "rdk:component:base",
  "model": "sensor-controlled",
  "attributes": {
    "base": "tracer-base",
    "movement_sensor": ["tracer-odom"],
    "control_parameters": [
      { "type": "linear_velocity", "p": 0, "i": 0, "d": 0 },
      { "type": "angular_velocity", "p": 0, "i": 0, "d": 0 }
    ]
  },
  "depends_on": ["tracer-base", "tracer-odom"]
}
```

Zero PID gains trigger auto-tune (robot will move). Copy logged gains into config when done. Prefer driving `tracer-sc` for closed-loop motion; use `tracer-base` for direct teleop.

### Tracer lights

```json
{
  "name": "tracer-lights",
  "api": "rdk:component:generic",
  "model": "viam-labs:agilex-tracer:lights",
  "attributes": {
    "can_backend": "auto",
    "can_interface": "can0"
  }
}
```

Examples:

```json
{ "command": "on" }
{ "command": "off" }
{ "command": "breathe" }
{ "command": "set_light", "mode": "custom", "brightness": 40 }
{ "command": "get_status" }
{ "command": "release" }
```

Modes: `off` (0), `on` (1), `breathe` (2), `custom` (3) with `brightness` 0–100.

**Remote note:** SWC on the FS transmitter also controls lights (up=breathe, middle=constant, down=off). If CAN light commands seem stuck in breathe or won’t turn off, set SWC to middle/down or power off the remote so chassis light control isn’t overridden.

## Shared CAN client

Base, odometry, and lights with the same `can_backend` + channel + bitrate share one bus and RX thread (reference-counted). Configure all three identically.

## License

Apache-2.0 — see [LICENSE](LICENSE).
