# `start_yahboom.sh` — Yahboom mecanum drive bridge

Subscribes `/cmd_vel` (`geometry_msgs/Twist`), forwards `(vx, vy, wz)` to the Yahboom YB-ERF01-V3.0 STM32 board over USB-serial via `Rosmaster_Lib.set_car_motion()`. The STM32 firmware does the mecanum inverse kinematics internally (when `car_type=0x01` = X3 mecanum), so this is a thin pass-through node — the "Path A" approach from the Obsidian *"Mecanum UGV - GitHub - AutomaticAddison ROSMASTER X3 ROS2"* note.

## What it does

1. Kills any prior bridge instance.
2. Verifies a Yahboom serial device exists (`/dev/myserial` or `/dev/ttyUSB0`).
3. `ros2 launch slam_bringup yahboom.launch.py "$@"` — spawns `slam_bringup/yahboom_bridge_node.py`.
4. Bridge:
   - Opens the serial port via `Rosmaster_Lib`.
   - Locks `car_type=1` (X3 mecanum).
   - Subscribes `/cmd_vel`, clamps to `max_v*`, calls `set_car_motion(vx, vy, wz)`.
   - Watchdog: zeroes motors if `/cmd_vel` is silent for `cmd_timeout` seconds.
   - Shutdown: sends `set_car_motion(0,0,0)` so SIGINT doesn't leave the rover coasting.

Does **not** publish `/odom` or `/imu/data` — FAST-LIO and Mid-360 already own those topics, and a duplicate publisher would create QoS / ordering conflicts. SBUS RC stays operational independently (firmware-level).

## Dependencies

**One-time setup** (the bridge will fail loudly if any of these are missing):

1. **Hardware** — Yahboom YB-ERF01-V3.0 plugged into Jetson via USB-C. Verify:
   ```bash
   ls -l /dev/serial/by-id/      # → usb-1a86_USB_Serial-if00-port0 → ../../ttyUSB0
   ```
2. **udev pin** — so the bridge always finds `/dev/myserial` regardless of USB ordering:
   ```bash
   sudo tee /etc/udev/rules.d/99-yahboom.rules <<'EOF'
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0666"
   EOF
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
3. **`Rosmaster_Lib`** — vendored in this repo at `vendor/Rosmaster_Lib_3.3.9/`. `./install.sh` does this; manually:
   ```bash
   pip3 install --user ~/slam_ws/src/slam_bringup/vendor/Rosmaster_Lib_3.3.9
   python3 -c "from Rosmaster_Lib import Rosmaster; print('OK')"
   ```
4. **dialout group** — needed to open `/dev/ttyUSB0` without sudo. `./install.sh` adds you; verify with `groups | grep dialout`. Log out / back in if just added.

NO ROS-side dependencies — the bridge runs standalone, useful for bench-testing the drive base independently of SLAM.

## Default usage

```bash
./start_yahboom.sh
```

What you get:
- Serial port: `/dev/myserial` (the udev-pinned symlink)
- `car_type=1` (X3 mecanum)
- Velocity clamps: `max_vx=0.5`, `max_vy=0.3`, `max_wz=1.0` rad/s
- Watchdog: 0.5 s timeout
- Listens on `/cmd_vel`, drives motors immediately

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `serial_port` | `/dev/myserial` | USB-serial path. Pinned by udev; falls back manually to `/dev/ttyUSB0` if pin missing. | Pass `serial_port:=/dev/ttyUSB0` if udev rule isn't installed yet. |
| `car_type` | `1` | Yahboom firmware constant for the chassis kinematics block. `1=X3 mecanum`, `2=X3 PLUS`, `4=X1`, `5=R2`. | Only when reusing the same Jetson on a different chassis. The bridge always re-asserts car_type via `set_car_type()` so you can flip between platforms. |
| `cmd_timeout` | `0.5` s | If no `/cmd_vel` for this long, command zero. | Raise if you're publishing at <2 Hz (e.g. step-by-step CLI testing). Lower if you want a tighter dead-man. |
| `max_vx` | `0.5` m/s | Forward/back velocity cap. | `0.3` for first runs / cluttered indoor. The chassis can do >1 m/s but encoder slip confuses FAST-LIO. |
| `max_vy` | `0.3` m/s | Strafe velocity cap (mecanum-only). | `0.0` to disable strafe entirely (forces diff-drive-like behavior). |
| `max_wz` | `1.0` rad/s | Yaw rate cap. | `0.5` for cautious testing; up to ~2.0 if you need quick turns. |

These are launch args (`name:=value`); the bridge node also accepts them as ROS params (`ros2 param set /yahboom_bridge max_vx 0.3` at runtime).

## Bench-test sequence (BEFORE Nav2)

Wheels off the ground — see Obsidian *"Yahboom Mecanum Configuration - Motor Wiring and Taranis SBUS Setup"* §4 for the canonical sanity tests.

```bash
# Terminal 1
./start_yahboom.sh

# Terminal 2
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Press 'b' to enable HOLONOMIC mode (mecanum strafe). Then:
#   i = forward       , = backward
#   u/o = strafe-fwd-left / strafe-fwd-right
#   j/l = pure yaw left / pure yaw right
```

Three checks:

| Twist | Expected |
|---|---|
| `vx = +0.1, vy = 0, vz = 0` | All four wheels spin forward at the top |
| `vx = 0, vy = +0.1, vz = 0` | MA + MD forward, MB + MC backward (strafe right) |
| `vx = 0, vy = 0, vz = +0.1` | MA + MB backward, MC + MD forward (yaw left / CCW) |

If a single wheel spins backward → fix via the **firmware polarity flag** for that channel, NOT by swapping motor leads. If strafe is broken even after fixing individual wheel directions, you have a port-assignment error — re-check the MA/MB/MC/MD corner mapping.

## Examples

```bash
# Default — wheels-on-blocks bench test:
./start_yahboom.sh

# Tighter caps for first runs in a cluttered space:
./start_yahboom.sh max_vx:=0.3 max_vy:=0.2 max_wz:=0.5

# udev pin not yet installed — point at the raw device:
./start_yahboom.sh serial_port:=/dev/ttyUSB0

# Disable strafe (if a wheel is fighting the diagonals during debug):
./start_yahboom.sh max_vy:=0.0
```

## Composed via slam.launch.py

You usually don't run `start_yahboom.sh` standalone for autonomous operation — it's included via `enable_drive:=true` in the SLAM bring-up:

```bash
./start_nav.sh platform:=mecanum force_3dof:=true enable_drive:=true
```

That brings up the entire SLAM + Nav2 + drive-bridge chain in one command. Use the standalone script only for bench-testing the drive base.

## Gotchas

- **"Rosmaster_Lib not installed"** — install the vendored copy: `pip3 install --user ~/slam_ws/src/slam_bringup/vendor/Rosmaster_Lib_3.3.9`.
- **"Permission denied on /dev/ttyUSB0"** — `groups | grep dialout`; if missing, `sudo usermod -aG dialout $USER` and log out/in.
- **Two bridges running** — each writes to the same serial port and motors lurch. `./kill_yahboom.sh` (with the kill_helpers escalation) catches and reports orphans.
- **SBUS RC works but `/cmd_vel` doesn't** — bridge isn't reaching the board. Check `pgrep -af yahboom_bridge` (must be running), `ls -l /dev/myserial` (must symlink to ttyUSB0), and `ros2 topic info /cmd_vel` (Subscription count ≥ 1 with the bridge up).
- **Nav2 commands `/cmd_vel` but rover doesn't move** — bridge isn't running. Pass `enable_drive:=true` to `start_nav.sh`, or run `./start_yahboom.sh` separately.

## See also

- [start_nav.md](start_nav.md) — autonomous nav including the bridge
- Main README §[Mecanum drive (Yahboom YB-ERF01)](../../README.md#mecanum-drive-yahboom-yb-erf01)
- Obsidian: *Robotics → Robots → Mecanum UGV → "Yahboom Mecanum Configuration - Motor Wiring and Taranis SBUS Setup"*
- Obsidian: *Robotics → Robots → Mecanum UGV → "Mecanum UGV - GitHub - AutomaticAddison ROSMASTER X3 ROS2"*
