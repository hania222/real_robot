#!/usr/bin/env python3
import json
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import serial


WHEEL_RADIUS = 0.065   # metres


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__('hardware_bridge_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baud_rate',   115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # ── Serial ───────────────────────────────────────────────────────────
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'Opened serial port {port} at {baud} baud')

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_js  = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_imu = self.create_publisher(Imu,        '/imu/data',     10)

        # ── Subscribers (wheel velocity commands from diff_drive_controller) ─
        #
        # diff_drive_controller publishes one Float64MultiArray per wheel.
        # The value is in rad/s — we convert to m/s before sending to ESP32.
        #
        self.create_subscription(
            Float64MultiArray,
            '/left_wheel_velocity_controller/commands',
            self._cb_left_cmd,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            '/right_wheel_velocity_controller/commands',
            self._cb_right_cmd,
            10,
        )

        # ── Serial reader thread ──────────────────────────────────────────────
        self._stop = False
        self._thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._thread.start()

        self.get_logger().info('Hardware bridge node ready.')

    # ── Command callbacks ────────────────────────────────────────────────────

    def _cb_left_cmd(self, msg: Float64MultiArray):
        """Receive left wheel command in rad/s, send m/s to ESP32."""
        rad_per_sec = msg.data[0]
        self._send_cmd('L', rad_per_sec * WHEEL_RADIUS)

    def _cb_right_cmd(self, msg: Float64MultiArray):
        """Receive right wheel command in rad/s, send m/s to ESP32."""
        rad_per_sec = msg.data[0]
        self._send_cmd('R', rad_per_sec * WHEEL_RADIUS)

    def _send_cmd(self, side: str, vel_ms: float):
        """Write a single-wheel velocity command (m/s) to ESP32 over serial."""
        try:
            line = json.dumps({side: round(vel_ms, 4)}) + '\n'
            self.ser.write(line.encode())
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial reader ────────────────────────────────────────────────────────

    def _serial_reader(self):
        """
        Continuously read feedback JSON from ESP32.
        Publishes /joint_states and /imu/data at the firmware's 50 Hz rate.
        """
        while not self._stop:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line or not line.startswith('{'):
                    continue

                data = json.loads(line)
                now  = self.get_clock().now().to_msg()

                # ── Joint states ─────────────────────────────────────────────
                #
                # ESP32 sends velocities in m/s and positions in m (accumulated).
                # ROS 2 joint_states for a wheel joint must be in rad/s and rad.
                #
                lv_ms = data.get('lv', 0.0)   # m/s
                rv_ms = data.get('rv', 0.0)
                lp_m  = data.get('lp', 0.0)   # m  (accumulated arc length)
                rp_m  = data.get('rp', 0.0)

                js = JointState()
                js.header.stamp = now
                js.name     = ['left_drive', 'right_drive']
                js.velocity = [lv_ms / WHEEL_RADIUS,   # → rad/s
                               rv_ms / WHEEL_RADIUS]
                js.position = [lp_m  / WHEEL_RADIUS,   # → rad
                               rp_m  / WHEEL_RADIUS]
                js.effort   = []
                self.pub_js.publish(js)

                # ── IMU ───────────────────────────────────────────────────────
                #
                # ESP32 sends raw SI values (m/s², rad/s) — no conversion needed.
                # Covariances are left as zero (unknown); tune ekf.yaml instead.
                #
                imu = Imu()
                imu.header.stamp    = now
                imu.header.frame_id = 'imu_link'

                imu.linear_acceleration.x = data.get('ax', 0.0)
                imu.linear_acceleration.y = data.get('ay', 0.0)
                imu.linear_acceleration.z = data.get('az', 0.0)

                imu.angular_velocity.x = data.get('gx', 0.0)
                imu.angular_velocity.y = data.get('gy', 0.0)
                imu.angular_velocity.z = data.get('gz', 0.0)

                # Orientation not provided by MPU6050 raw read;
                # robot_localization EKF will integrate angular_velocity itself.
                imu.orientation_covariance[0] = -1.0   # signals "no orientation"

                self.pub_imu.publish(imu)

            except json.JSONDecodeError:
                pass   # partial line on startup — ignore
            except Exception as e:
                if not self._stop:
                    self.get_logger().warn(f'Serial read error: {e}')

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop = True
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()