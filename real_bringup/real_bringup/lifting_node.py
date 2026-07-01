#!/usr/bin/env python3
"""
lift_node.py

Simple ROS 2 bridge between the Pi and the lift Arduino Uno
(lift_main.ino — home / lift / status / stop JSON-over-serial protocol).

How it works:
  - Subscribe to /lift/command (std_msgs/String) — publish raw JSON strings
    like '{"cmd":"home"}' or '{"cmd":"lift","height_mm":150.0}' to trigger
    a sequence.
  - Every JSON line the Arduino sends back over serial is republished as-is
    on /lift/status (std_msgs/String) — ack, home_complete, lift_complete,
    status, error, boot — all of it, unmodified.
  - No blocking, no waiting for completion inside the node. If you need to
    know when a "lift" finished, subscribe to /lift/status and watch for a
    message with "type":"lift_complete".

Same pattern as hardware_bridge_node.py, joystick_ws_node.py, and
carriage_node.py — kept consistent on purpose.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class LiftNode(Node):

    def __init__(self):
        super().__init__('lift_node')

        self.declare_parameter('serial_port', '/dev/lift_arduino')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('status_poll_period_sec', 1.0)

        port        = self.get_parameter('serial_port').value
        baud_rate   = self.get_parameter('baud_rate').value
        poll_period = self.get_parameter('status_poll_period_sec').value

        self.serial_conn = serial.Serial(port, baud_rate, timeout=0.1)
        self.get_logger().info(f'Serial open: {port} @ {baud_rate} baud')

        # write lock — command callback and the status-poll timer both
        # write to the serial port, so guard against interleaved writes
        self._write_lock = threading.Lock()
        self._shutdown_requested = False

        # publisher: every JSON line from the Arduino, forwarded as-is
        self.status_pub = self.create_publisher(String, '/lift/status', 10)

        # subscriber: raw JSON commands to send to the Arduino
        self.create_subscription(String, '/lift/command', self._on_command, 10)

        # periodic status poll so consumers get regular updates even if
        # nothing is actively homing/lifting
        self.create_timer(poll_period, self._poll_status)

        threading.Thread(target=self._serial_reader_loop, daemon=True).start()

    def _send(self, data: dict):
        try:
            with self._write_lock:
                self.serial_conn.write((json.dumps(data) + '\n').encode())
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def _on_command(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'/lift/command: bad JSON: {msg.data}')
            return
        self._send(data)

    def _poll_status(self):
        self._send({'cmd': 'status'})

    def _serial_reader_loop(self):
        while not self._shutdown_requested:
            try:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
                continue

            if not line or not line.startswith('{'):
                continue

            # validate it's real JSON before republishing, but forward the
            # original raw line so nothing is lost/reformatted
            try:
                json.loads(line)
            except json.JSONDecodeError:
                continue

            out = String()
            out.data = line
            self.status_pub.publish(out)

    def destroy_node(self):
        self._shutdown_requested = True
        if self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()