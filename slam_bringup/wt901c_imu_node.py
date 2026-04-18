"""WitMotion WT901C / WT9011-style 0x61 packet → sensor_msgs/Imu publisher.

Why this exists rather than using witmotion_ros (ElettraSar):
  ElettraSar's witmotion-uart-qt library only registers packet IDs 0x51–0x54
  (legacy WitMotion protocol — separate frames for accel, gyro, angles, mag).
  The WT901C / WT9011 family emits a single combined 20-byte 0x61 packet
  (header + accel-XYZ + gyro-XYZ + angle-XYZ, no checksum). ElettraSar
  silently drops every byte it sees from this device. Rather than patching
  a Qt-heavy upstream we don't otherwise need, we read the port directly.

Packet layout (per WitMotion WT9011DCL / BWT901CL spec, verified against a
live capture from gizmo's WT901C at 115200 baud):

  offset  size  field           scaling                              units
  ------  ----  --------------  ----------------------------------   -----
  0       1     header (0x55)
  1       1     type   (0x61)
  2-3     i16   AccX            value/32768 * 16g                    g
  4-5     i16   AccY            value/32768 * 16g                    g
  6-7     i16   AccZ            value/32768 * 16g                    g
  8-9     i16   GyroX           value/32768 * 2000 deg/s             deg/s
  10-11   i16   GyroY           value/32768 * 2000 deg/s             deg/s
  12-13   i16   GyroZ           value/32768 * 2000 deg/s             deg/s
  14-15   i16   AngleX (Roll)   value/32768 * 180 deg                deg
  16-17   i16   AngleY (Pitch)  value/32768 * 180 deg                deg
  18-19   i16   AngleZ (Yaw)    value/32768 * 180 deg                deg

There is no trailing checksum on this packet variant — packets sit back
to back, so re-sync on every (0x55, 0x61) pair we see and discard bytes
between sync points.
"""

import math
import struct

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu

HEADER = 0x55
PACKET_TYPE = 0x61
PACKET_SIZE = 20

ACC_SCALE = 16.0 * 9.80665 / 32768.0      # int16 → m/s^2
GYRO_SCALE = math.radians(2000.0) / 32768.0  # int16 → rad/s
ANGLE_SCALE = math.radians(180.0) / 32768.0  # int16 → rad

# WitMotion config-write protocol: 0xFF 0xAA <reg> <data_low> <data_high>.
# Registers must be unlocked first; otherwise writes are silently ignored.
# We tested the volatile path (unlock + rate, no save) on this WT901C and
# the rate change reverts within ~1ms — SAVE is required for any config
# write to take effect at all on this firmware variant. So when the user
# requests an output rate change we always include SAVE, which means the
# new rate persists across power cycles and the EEPROM gets one write per
# explicit reconfiguration. The device's EEPROM is rated ~10k writes, so
# this is fine as long as we don't reconfigure every launch — see the
# output_rate_hz parameter default (0) and the comment there.
UNLOCK_BYTES = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
SAVE_BYTES = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])

# WT9011 / WT901C output-rate register values:
#   0x06=10Hz (factory default), 0x07=20Hz, 0x08=50Hz,
#   0x09=100Hz, 0x0B=200Hz
RATE_REG_VALUES = {
    10: 0x06, 20: 0x07, 50: 0x08, 100: 0x09, 200: 0x0B,
}


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class WT901CImuNode(Node):
    def __init__(self):
        super().__init__('wt901c_imu')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data')
        # Stat reporting cadence — verifies in the log that the packet stream
        # is healthy without spamming. 0 disables.
        self.declare_parameter('stats_period_sec', 5.0)
        # Output rate to set on the device at startup. Default 0 = skip
        # reconfiguration entirely (just trust whatever rate the device's
        # EEPROM holds). Set to one of RATE_REG_VALUES keys (10/20/50/100/200)
        # to push a new rate; this issues unlock+write+save which IS an
        # EEPROM write on this WT901C firmware (volatile writes silently
        # revert — see UNLOCK_BYTES comment for why). After a one-time
        # `ros2 launch ... witmotion.launch.py output_rate_hz:=200` the
        # rate persists across power cycles and you can drop back to 0.
        self.declare_parameter('output_rate_hz', 0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        self.stats_period = float(self.get_parameter('stats_period_sec').value)
        rate_hz = int(self.get_parameter('output_rate_hz').value)

        self.publisher = self.create_publisher(Imu, topic, 50)

        self.get_logger().info(
            f'Opening {port} @ {baud} baud, publishing to {topic} '
            f'(frame_id={self.frame_id})'
        )
        # timeout=0.1 keeps us responsive to Ctrl-C; the IMU at 200 Hz fills
        # the buffer well within that window so no actual data is missed.
        self.serial = serial.Serial(port, baud, timeout=0.1)

        if rate_hz > 0:
            self._configure_output_rate(rate_hz)
        self.buffer = bytearray()

        self.packet_count = 0
        self.bad_sync_count = 0
        self.last_stats_t = self.get_clock().now()
        if self.stats_period > 0:
            self.create_timer(self.stats_period, self._log_stats)

        # Spin the read loop on a ROS timer rather than a blocking thread —
        # this lets parameter callbacks and timer callbacks share the executor.
        self.create_timer(0.001, self._read_and_publish)

    def _configure_output_rate(self, rate_hz):
        if rate_hz not in RATE_REG_VALUES:
            self.get_logger().warn(
                f'output_rate_hz={rate_hz} not in {sorted(RATE_REG_VALUES)} — '
                f'skipping reconfiguration, device keeps its current rate'
            )
            return
        import time
        rate_byte = RATE_REG_VALUES[rate_hz]
        rate_cmd = bytes([0xFF, 0xAA, 0x03, rate_byte, 0x00])
        self.serial.write(UNLOCK_BYTES); self.serial.flush(); time.sleep(0.1)
        self.serial.write(rate_cmd);     self.serial.flush(); time.sleep(0.1)
        self.serial.write(SAVE_BYTES);   self.serial.flush(); time.sleep(0.3)
        # Drain the data emitted during the reconfigure window before
        # our packet sync starts. Without this the first second of
        # output gets parsed against a buffer that may straddle the
        # rate transition, producing spurious resync warnings.
        self.serial.reset_input_buffer()
        self.get_logger().warn(
            f'set device output rate to {rate_hz} Hz and SAVED to EEPROM. '
            f'Drop output_rate_hz back to 0 for normal launches.'
        )

    def _read_and_publish(self):
        chunk = self.serial.read(256)
        if not chunk:
            return
        self.buffer.extend(chunk)

        # Drain as many whole packets as the buffer holds.
        while len(self.buffer) >= PACKET_SIZE:
            # Re-sync to (0x55, 0x61). Drop bytes until we find it.
            i = 0
            while i <= len(self.buffer) - 2:
                if self.buffer[i] == HEADER and self.buffer[i + 1] == PACKET_TYPE:
                    break
                i += 1
            else:
                # No header in buffer — discard everything but possibly the
                # last byte (which could be the start of a half-arrived header).
                self.bad_sync_count += len(self.buffer) - 1
                self.buffer = self.buffer[-1:]
                return

            if i > 0:
                self.bad_sync_count += i
                del self.buffer[:i]

            if len(self.buffer) < PACKET_SIZE:
                return  # wait for the rest of the packet to arrive

            self._publish(bytes(self.buffer[:PACKET_SIZE]))
            del self.buffer[:PACKET_SIZE]
            self.packet_count += 1

    def _publish(self, packet):
        # struct: header(B), type(B), 9x int16 little-endian
        _, _, ax, ay, az, gx, gy, gz, rx, ry, rz = struct.unpack('<BBhhhhhhhhh', packet)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = ax * ACC_SCALE
        msg.linear_acceleration.y = ay * ACC_SCALE
        msg.linear_acceleration.z = az * ACC_SCALE

        msg.angular_velocity.x = gx * GYRO_SCALE
        msg.angular_velocity.y = gy * GYRO_SCALE
        msg.angular_velocity.z = gz * GYRO_SCALE

        roll, pitch, yaw = rx * ANGLE_SCALE, ry * ANGLE_SCALE, rz * ANGLE_SCALE
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Covariances: -1 in [0] of any matrix means "unknown" per REP 145.
        # We don't have factory covariances for this unit, so leave the
        # default zero matrices — RTABMap/Madgwick filters will treat the
        # data as point estimates rather than weighted observations.
        self.publisher.publish(msg)

    def _log_stats(self):
        now = self.get_clock().now()
        dt = (now - self.last_stats_t).nanoseconds * 1e-9
        rate = self.packet_count / dt if dt > 0 else 0.0
        self.get_logger().info(
            f'{rate:.1f} Hz over last {dt:.1f}s '
            f'({self.packet_count} packets, {self.bad_sync_count} resync bytes dropped)'
        )
        self.packet_count = 0
        self.bad_sync_count = 0
        self.last_stats_t = now

    def destroy_node(self):
        try:
            self.serial.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WT901CImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Under SIGTERM (e.g. ros2 launch sending the kill), rclpy's signal
        # handler calls shutdown() before we get here, so a second call
        # raises RCLError. Guard with rclpy.ok().
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
