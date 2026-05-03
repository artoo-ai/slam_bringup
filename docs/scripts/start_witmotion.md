# `start_witmotion.sh` — WitMotion WT901 IMU

Standalone WitMotion bring-up via our custom `wt901c_imu` Python node (parses the 0x61 packet at 200 Hz).

## What it does

`ros2 launch slam_bringup witmotion.launch.py` — runs `slam_bringup/wt901c_imu_node.py`. Publishes `/imu/data` (`sensor_msgs/Imu`) at ~200 Hz.

NOT used by FAST-LIO (which uses the Mid-360 internal IMU). Reserved as a redundant attitude reference for downstream consumers.

## Dependencies

- WitMotion plugged into Jetson over USB-serial (CH340 or FT232 typically).
- User in `dialout` group — `./install.sh` adds you. Log out/in if just added.
- Workspace built (`wt901c_imu` is a console_script entry point).

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | USB-serial device. | Per-host pinning (some systems are `/dev/ttyACM0`). |
| `baudrate` | `9600` | WitMotion factory default. | `230400` only if you've reflashed the firmware. |
| `frame_id` | `imu_link` | TF frame in published `Imu` messages. | Almost never. |
| `publish_rate_hz` | `200` | Throttle target. WitMotion fires at 200 Hz natively. | Lower if you're CPU-constrained. |

## Examples

```bash
./start_witmotion.sh                                # default port
./start_witmotion.sh serial_port:=/dev/ttyUSB1     # second USB-serial device
```

## Gotchas

- **"Device or resource busy"** — orphan node holding the port. `./kill_witmotion.sh` (with kill_helpers escalation) catches it.
- **No data on `/imu/data`** — WitMotion firmware needs to be in 0x61 mode (set via WitMotion's PC config tool, one-time per device).
- **CPU contention with FAST-LIO** — disable for SLAM runs by passing `enable_witmotion:=false` to `start_sensors.sh` / `start_perception.sh`.

## See also

- `slam_bringup/wt901c_imu_node.py` — protocol implementation
- [start_sensors.md](start_sensors.md) — bundled with Mid-360 + D435
