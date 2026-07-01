#!/usr/bin/env python3
"""
carriage_node.py

Simple ROS 2 bridge between the Pi and the carriage Arduino Uno
(carriage_main.ino — home / pick / status / stop JSON-over-serial protocol).

How it works:
  - Subscribe to /carriage/command (std_msgs/String) — publish raw JSON
    strings like '{"cmd":"home"}' or '{"cmd":"pick"}' to trigger a sequence.
  - Every JSON line the Arduino sends back over serial is republished as-is
    on /carriage/status (std_msgs/String) — ack, home_complete, pick_complete,
    status, error, boot, arm_end_sw — all of it, unmodified.
  - No blocking, no waiting for completion inside the node. If you need to
    know when a "pick" finished, subscribe to /carriage/status and watch for
    a message with "type":"pick_complete".

This trades the request/response convenience of a service for simplicity —
it's the same pattern as hardware_bridge_node.py and joystick_ws_node.py.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class CarriageNode(Node):

    def __init__(self):
        super().__init__('carriage_node')

        self.declare_parameter('serial_port', '/dev/carriage_arduino')
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
        self.status_pub = self.create_publisher(String, '/carriage/status', 10)

        # subscriber: raw JSON commands to send to the Arduino
        self.create_subscription(String, '/carriage/command', self._on_command, 10)

        # periodic status poll so consumers get regular updates even if
        # nothing is actively homing/picking
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
            self.get_logger().warn(f'/carriage/command: bad JSON: {msg.data}')
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
    node = CarriageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()