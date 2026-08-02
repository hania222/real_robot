#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json
import threading

# Defaults — override via ROS2 params if needed
SERIAL_PORT = '/dev/esp32'   # ttyACM0 is common for Mega, ttyUSB0 for ESP32/CP2102 boards — check with `ls /dev/tty*`
BAUD_RATE   = 115200
WHEEL_BASE  = 0.642  # metres — must match config.h WHEEL_BASE exactly


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__('hardware_bridge_node')

        self.declare_parameter('serial_port', SERIAL_PORT)
        self.declare_parameter('baud_rate', BAUD_RATE)
        self.declare_parameter('wheel_base', WHEEL_BASE)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port}: {e}')
            raise

        # Give the board a moment to reset after the serial port opens (common on Mega/ESP32)
        self.create_timer(2.0, self._log_ready, )

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Background thread continuously reads feedback lines coming from the board
        self._stop_reading = False
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def _log_ready(self):
        self.get_logger().info('Bridge ready — listening on /cmd_vel')

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics: split body velocity into per-wheel targets
        left_target = v - (w * self.wheel_base / 2.0)
        right_target = v + (w * self.wheel_base / 2.0)
        command = {"left_target": round(left_target, 4), "right_target": round(right_target, 4)}
        self.get_logger().info(f'Sending: {command}')  

        command = {
            "left_target": round(left_target, 4),
            "right_target": round(right_target, 4)
        }

        try:
            line = json.dumps(command) + '\n'
            self.ser.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write failed: {e}')

    def read_serial_loop(self):
        while not self._stop_reading and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    raw_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line:
                        self.handle_feedback(raw_line)
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read failed: {e}')

    def handle_feedback(self, raw_line):
        try:
            data = json.loads(raw_line)
            self.get_logger().debug(f'Feedback: {data}')
        except json.JSONDecodeError:
            # Firmware also prints a plain info string on boot — that's expected, not an error
            self.get_logger().debug(f'Non-JSON line from board: {raw_line}')

    def destroy_node(self):
        self._stop_reading = True
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
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