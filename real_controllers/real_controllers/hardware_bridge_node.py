#!/usr/bin/env python3
import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
import serial


WHEEL_RADIUS = 0.065   # metres — must match config.h WHEEL_RADIUS


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

        # ── Last commanded velocities (rad/s) — written by callbacks, read by
        #    serial-reader thread for /pid_debug target fields ────────────────
        self._cmd_lock             = threading.Lock()
        self._last_left_cmd_rads   = 0.0
        self._last_right_cmd_rads  = 0.0

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_js    = self.create_publisher(JointState,        '/joint_states', 10)
        self.pub_imu   = self.create_publisher(Imu,               '/imu/data',     10)
        self.pub_debug = self.create_publisher(Float64MultiArray,  '/pid_debug',    10)

        # ── Subscribers ──────────────────────────────────────────────────────

        # Wheel velocity commands from diff_drive_controller (rad/s → m/s to ESP32)
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

        # PID gain updates forwarded to ESP32 over serial (no reflash needed)
        self.create_subscription(
            String,
            '/pid_gains',
            self._cb_pid_gains,
            10,
        )

        # ── Serial reader thread ──────────────────────────────────────────────
        self._stop   = False
        self._thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._thread.start()

        self.get_logger().info(
            'Hardware bridge node ready.\n'
            '  Publishing:   /joint_states  /imu/data  /pid_debug\n'
            '  Subscribing:  /left_wheel_velocity_controller/commands\n'
            '                /right_wheel_velocity_controller/commands\n'
            '                /pid_gains'
        )

    # ── Command callbacks ────────────────────────────────────────────────────

    def _cb_left_cmd(self, msg: Float64MultiArray):
        """Receive left wheel command in rad/s → convert to m/s → send to ESP32."""
        rad_per_sec = msg.data[0]
        with self._cmd_lock:
            self._last_left_cmd_rads = rad_per_sec
        self._send_cmd('L', rad_per_sec * WHEEL_RADIUS)

    def _cb_right_cmd(self, msg: Float64MultiArray):
        """Receive right wheel command in rad/s → convert to m/s → send to ESP32."""
        rad_per_sec = msg.data[0]
        with self._cmd_lock:
            self._last_right_cmd_rads = rad_per_sec
        self._send_cmd('R', rad_per_sec * WHEEL_RADIUS)

    def _cb_pid_gains(self, msg: String):
        """
        Forward a PID gain update from /pid_gains to the ESP32 over serial.

        Accepted JSON in msg.data:
            {"kp": 80.0, "ki": 5.0, "kd": 1.0}        ← both wheels (ESP32 native)
            {"kp_l": 80.0, "ki_l": 5.0, "kd_l": 1.0}  ← left  (bridge strips suffix)
            {"kp_r": 80.0, "ki_r": 5.0, "kd_r": 1.0}  ← right (bridge strips suffix)

        The ESP32 firmware applies kp/ki/kd to both wheels simultaneously.
        For independent left/right tuning you would need two serial messages;
        that is left for a future firmware update.
        """
        try:
            incoming = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'/pid_gains: invalid JSON — {e}')
            return

        # Normalise: strip _l / _r suffixes so ESP32 gets bare kp/ki/kd
        out: dict = {}
        for key in ('kp', 'ki', 'kd'):
            # Priority: bare key first, then _l, then _r
            if key in incoming:
                out[key] = float(incoming[key])
            elif f'{key}_l' in incoming:
                out[key] = float(incoming[f'{key}_l'])
            elif f'{key}_r' in incoming:
                out[key] = float(incoming[f'{key}_r'])

        if not out:
            self.get_logger().warn('/pid_gains: no recognised keys (kp/ki/kd)')
            return

        self.get_logger().info(f'Sending PID gains to ESP32: {out}')
        try:
            line = json.dumps({k: round(v, 4) for k, v in out.items()}) + '\n'
            self.ser.write(line.encode())
        except Exception as e:
            self.get_logger().warn(f'Serial write error (pid_gains): {e}')

    # ── Serial helpers ────────────────────────────────────────────────────────

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
        Continuously read feedback JSON from ESP32 at 50 Hz.

        Publishes:
            /joint_states  — wheel positions (rad) and velocities (rad/s)
            /imu/data      — accelerometer (m/s²) and gyroscope (rad/s)
            /pid_debug     — Float64MultiArray[8] for PlotJuggler:
                               [0] target_left_rads
                               [1] actual_left_rads
                               [2] error_left_rads
                               [3] pwm_left        (0.0 — not reported by firmware)
                               [4] target_right_rads
                               [5] actual_right_rads
                               [6] error_right_rads
                               [7] pwm_right       (0.0 — not reported by firmware)

        PlotJuggler setup:
            • Subscribe to /pid_debug
            • Drag data[0] and data[1] onto the same plot → left wheel step response
            • Drag data[4] and data[5] onto the same plot → right wheel step response
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

                # Skip info/warn messages the firmware sends on startup
                if 'info' in data or 'warn' in data:
                    key = 'info' if 'info' in data else 'warn'
                    self.get_logger().info(f'ESP32 [{key}]: {data[key]}')
                    continue

                now = self.get_clock().now().to_msg()

                # ── Wheel data ────────────────────────────────────────────────
                lv_ms = data.get('lv', 0.0)   # actual velocity m/s
                rv_ms = data.get('rv', 0.0)
                lp_m  = data.get('lp', 0.0)   # cumulative arc-length m
                rp_m  = data.get('rp', 0.0)

                lv_rads = lv_ms / WHEEL_RADIUS   # convert to rad/s for ROS
                rv_rads = rv_ms / WHEEL_RADIUS

                # ── Joint states ──────────────────────────────────────────────
                js = JointState()
                js.header.stamp = now
                js.name         = ['left_drive', 'right_drive']
                js.velocity     = [lv_rads, rv_rads]
                js.position     = [lp_m / WHEEL_RADIUS,   # → rad
                                   rp_m / WHEEL_RADIUS]
                js.effort       = []
                self.pub_js.publish(js)

                # ── IMU ───────────────────────────────────────────────────────
                imu = Imu()
                imu.header.stamp    = now
                imu.header.frame_id = 'imu_link'

                imu.linear_acceleration.x = data.get('ax', 0.0)
                imu.linear_acceleration.y = data.get('ay', 0.0)
                imu.linear_acceleration.z = data.get('az', 0.0)

                imu.angular_velocity.x = data.get('gx', 0.0)
                imu.angular_velocity.y = data.get('gy', 0.0)
                imu.angular_velocity.z = data.get('gz', 0.0)

                # MPU6050 raw read does not provide orientation;
                # robot_localization EKF integrates angular_velocity itself.
                imu.orientation_covariance[0] = -1.0   # signals "no orientation"

                self.pub_imu.publish(imu)

                # ── PID debug (for PlotJuggler) ───────────────────────────────
                with self._cmd_lock:
                    tgt_l = self._last_left_cmd_rads
                    tgt_r = self._last_right_cmd_rads

                debug = Float64MultiArray()
                debug.data = [
                    tgt_l,              # [0] target left  (rad/s)
                    lv_rads,            # [1] actual left  (rad/s)
                    tgt_l - lv_rads,    # [2] error  left  (rad/s)
                    0.0,                # [3] pwm    left  (not available from ESP32)
                    tgt_r,              # [4] target right (rad/s)
                    rv_rads,            # [5] actual right (rad/s)
                    tgt_r - rv_rads,    # [6] error  right (rad/s)
                    0.0,                # [7] pwm    right (not available from ESP32)
                ]
                self.pub_debug.publish(debug)

            except json.JSONDecodeError:
                pass   # partial line on startup — silently ignore
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