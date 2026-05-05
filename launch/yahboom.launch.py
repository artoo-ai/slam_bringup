"""Yahboom YB-ERF01 (mecanum X3) cmd_vel bridge.

Run standalone for teleop testing:

    ./start_yahboom.sh                       # auto-pick port, car_type=1 (X3 mecanum)
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # press 'b' to enable holonomic mode → u/o/m/. command strafe (vy)

Or include from slam.launch.py via enable_drive:=true. See PLATFORM_BRIDGES
in slam.launch.py for the per-platform mecanum entry. Companion vault
notes:
  - "Yahboom Mecanum Configuration - Motor Wiring and Taranis SBUS Setup"
  - "Mecanum UGV - GitHub - AutomaticAddison ROSMASTER X3 ROS2"

Serial port resolution priority (matches start_yahboom.sh):
  1. serial_port:=... explicit launch arg                (highest)
  2. $YAHBOOM_SERIAL_PORT env var
  3. ~/.yahboom_serial_port file
  4. /dev/myserial
  5. /dev/ttyUSB0                                        (last resort)

The resolution lives here (not just in start_yahboom.sh) so that
start_nav.sh / start_slam.sh — which include this launch file from
slam.launch.py without going through start_yahboom.sh — also pick the
right port. Without this, the launch arg defaulted to /dev/myserial,
which races with the WitMotion (also CH340 1a86:7523) and silently
opens the wrong device — bridge looks healthy, motors never spin.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_serial_port(context, *args, **kwargs):
    """Resolve serial_port at launch time. If the arg is at the sentinel
    default `auto`, walk the priority list. Otherwise honor the override.
    """
    requested = LaunchConfiguration('serial_port').perform(context)

    if requested != 'auto' and requested:
        resolved = requested
        source = 'serial_port launch arg'
    else:
        env_port = os.environ.get('YAHBOOM_SERIAL_PORT', '')
        cfg_file = Path.home() / '.yahboom_serial_port'
        cfg_port = ''
        if cfg_file.exists():
            cfg_port = cfg_file.read_text().strip()

        if env_port and Path(env_port).exists():
            resolved = env_port
            source = '$YAHBOOM_SERIAL_PORT'
        elif cfg_port and Path(cfg_port).exists():
            resolved = cfg_port
            source = '~/.yahboom_serial_port'
        elif Path('/dev/myserial').exists():
            resolved = '/dev/myserial'
            source = '/dev/myserial fallback'
        elif Path('/dev/ttyUSB0').exists():
            resolved = '/dev/ttyUSB0'
            source = '/dev/ttyUSB0 last-resort'
        else:
            raise RuntimeError(
                'yahboom.launch.py: no Yahboom serial device found. '
                'Plug in the YB-ERF01 and run scripts/yahboom_find_port.sh, '
                'or pass serial_port:=/dev/serial/by-path/<...> explicitly.'
            )

    print(f'yahboom.launch.py: serial_port → {resolved}  (source: {source})')

    return [Node(
        package='slam_bringup',
        executable='yahboom_bridge',
        name='yahboom_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': resolved,
            'car_type':    int(LaunchConfiguration('car_type').perform(context)),
            'cmd_timeout': float(LaunchConfiguration('cmd_timeout').perform(context)),
            'max_vx':      float(LaunchConfiguration('max_vx').perform(context)),
            'max_vy':      float(LaunchConfiguration('max_vy').perform(context)),
            'max_wz':      float(LaunchConfiguration('max_wz').perform(context)),
            'invert_vx':   LaunchConfiguration('invert_vx').perform(context).lower() in ('1', 'true', 'yes'),
            'invert_vy':   LaunchConfiguration('invert_vy').perform(context).lower() in ('1', 'true', 'yes'),
        }],
    )]


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='auto',
        description='USB-serial path to the Yahboom STM32. Default `auto` walks '
                    'the priority list ($YAHBOOM_SERIAL_PORT → ~/.yahboom_serial_port '
                    '→ /dev/myserial → /dev/ttyUSB0). Pass an explicit path '
                    '(e.g. /dev/serial/by-path/...) to override.',
    )
    car_type_arg = DeclareLaunchArgument(
        'car_type', default_value='1',
        description='Yahboom firmware car_type constant. 1=X3 mecanum (default), '
                    '2=X3 PLUS, 4=X1, 5=R2.',
    )
    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout', default_value='0.5',
        description='Watchdog: zero motors if /cmd_vel is silent for this many seconds.',
    )
    max_vx_arg = DeclareLaunchArgument('max_vx', default_value='0.5')
    max_vy_arg = DeclareLaunchArgument('max_vy', default_value='0.3')
    max_wz_arg = DeclareLaunchArgument('max_wz', default_value='1.0')
    # Default true: this rover's YB-ERF01 is mounted with firmware-+X
    # opposite to the sensor mast (see PLATFORM_BRIDGES['mecanum'] in
    # slam.launch.py for the full story). Pass invert_vx:=false /
    # invert_vy:=false on a chassis where firmware-+X already faces the
    # sensor side.
    invert_vx_arg = DeclareLaunchArgument('invert_vx', default_value='true')
    invert_vy_arg = DeclareLaunchArgument('invert_vy', default_value='true')

    return LaunchDescription([
        serial_port_arg,
        car_type_arg,
        cmd_timeout_arg,
        max_vx_arg,
        max_vy_arg,
        max_wz_arg,
        invert_vx_arg,
        invert_vy_arg,
        OpaqueFunction(function=_resolve_serial_port),
    ])
