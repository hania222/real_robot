#!/usr/bin/env python3
"""
lift_bridge_node.py
Bridges the Arduino Uno lift controller and ROS 2.

Serial protocol (115200 baud, newline-terminated JSON):
  Pi → Uno:  {"cmd":"home"} | {"cmd":"lift","height_mm":100.0} | {"cmd":"status"} | {"cmd":"stop"}
  Uno → Pi:  {"type":"boot"} | {"type":"ack"} | {"type":"home_complete"} |
              {"type":"lift_complete"} | {"type":"status"} | {"type":"error"}

ROS interface:
  Subscriptions:
    /lift/command   (std_msgs/String)  — JSON: {"cmd":"home"} or {"cmd":"lift","height_mm":N} or {"cmd":"stop"}
  Publications:
    /lift/status    (std_msgs/String)  — raw status JSON forwarded from Arduino
    /lift/state     (std_msgs/String)  — "IDLE" | "BUSY" | "ERROR" | "HOMING" | "MOVING"
    /lift/height_mm (std_msgs/Float32) — current lift height in mm (from status packets)
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import serial


class LiftBridgeNode(Node):

    def __init__(self):
        super().__init__('lift_bridge_node')

        self.declare_parameter('serial_port', '/dev/lift_arduino')
        self.declare_parameter('baud_rate', 115200)

        port      = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.serial_conn = serial.Serial(port, baud_rate, timeout=0.1)
        self.get_logger().info(f'Serial open: {port} @ {baud_rate} baud')

        # publishers
        self.status_pub    = self.create_publisher(String,  '/lift/status',    10)
        self.state_pub     = self.create_publisher(String,  '/lift/state',     10)
        self.height_pub    = self.create_publisher(Float32, '/lift/height_mm', 10)

        # subscriber — Pi/task_manager sends commands here
        self.create_subscription(String, '/lift/command', self._on_command, 10)

        # internal state mirrored from Arduino status packets
        self._is_homed    = False
        self._lift_mm     = 0.0
        self._busy        = False

        # clean shutdown flag
        self._shutdown_requested = False

        threading.Thread(target=self._serial_reader_loop, daemon=True).start()

    # ── Outgoing: ROS → Arduino ──────────────────────────────────────────────

    def _on_command(self, msg: String):
        """
        Forward a command from /lift/command to the Arduino over serial.
        Accepted payloads:
          {"cmd":"home"}
          {"cmd":"lift","height_mm":250.0}
          {"cmd":"stop"}
          {"cmd":"status"}
        Reject lift commands while not homed or while busy so the
        task_manager gets an immediate error instead of a silent queue.
        """
        raw = msg.data.strip()

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as error:
            self.get_logger().warn(f'/lift/command bad JSON: {error}')
            return

        cmd = data.get('cmd', '')

        # guard: lift requires homing first
        if cmd == 'lift' and not self._is_homed:
            self._publish_error('not homed — send home command first')
            return

        # guard: reject new commands while the Arduino is busy
        # (stop and status are always allowed through)
        if self._busy and cmd not in ('stop', 'status'):
            self._publish_error(f'busy — Arduino is still executing previous command, wait for completion')
            return

        self._send(data)
        self.get_logger().info(f'→ Arduino: {raw}')

    def _send(self, data: dict):
        try:
            self.serial_conn.write((json.dumps(data) + '\n').encode())
        except Exception as error:
            self.get_logger().warn(f'Serial write error: {error}')

    # ── Incoming: Arduino → ROS ──────────────────────────────────────────────

    def _serial_reader_loop(self):
        while not self._shutdown_requested:
            try:
                raw_line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not raw_line.startswith('{'):
                    continue
                data = json.loads(raw_line)
            except json.JSONDecodeError:
                continue
            except Exception as error:
                self.get_logger().warn(f'Serial read error: {error}')
                continue

            self._dispatch_packet(data, raw_line)

    def _dispatch_packet(self, data: dict, raw_line: str):
        """Route each incoming JSON packet to the right handler."""
        packet_type = data.get('type', '')

        if packet_type == 'boot':
            self.get_logger().info(f'Arduino boot: {data.get("msg", "")}')
            self._publish_state('IDLE')

        elif packet_type == 'ack':
            cmd = data.get('cmd', '')
            self.get_logger().info(f'Arduino ack: {cmd}')
            # mark busy as soon as Arduino acknowledges an action command
            if cmd in ('home', 'lift'):
                self._busy = True
                self._publish_state('HOMING' if cmd == 'home' else 'MOVING')

        elif packet_type == 'home_complete':
            success = data.get('success', False)
            if success:
                self._is_homed = True
                self._lift_mm  = 0.0
                self.get_logger().info('Lift homed successfully')
                self._publish_state('IDLE')
                self._publish_height(0.0)
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Homing failed: {reason}')
                self._publish_state('ERROR')
            self._busy = False

        elif packet_type == 'lift_complete':
            success = data.get('success', False)
            if success:
                self.get_logger().info(f'Lift move complete — now at {self._lift_mm:.1f} mm')
                self._publish_state('IDLE')
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Lift move failed: {reason}')
                self._publish_state('ERROR')
            self._busy = False

        elif packet_type == 'status':
            # mirror Arduino state into ROS topics
            self._is_homed = data.get('homed', self._is_homed)
            self._lift_mm  = float(data.get('lift_mm', self._lift_mm))
            self._busy     = data.get('busy', self._busy)
            self._publish_height(self._lift_mm)
            self._publish_state(data.get('state', 'IDLE'))
            # forward the full raw JSON so task_manager can inspect it if needed
            status_msg      = String()
            status_msg.data = raw_line
            self.status_pub.publish(status_msg)

        elif packet_type == 'error':
            error_text = data.get('msg', 'unknown error')
            self.get_logger().error(f'Arduino error: {error_text}')
            self._busy = False
            self._publish_state('ERROR')
            # also forward as a status message so callers subscribed to /lift/status see it
            status_msg      = String()
            status_msg.data = raw_line
            self.status_pub.publish(status_msg)

        else:
            # unknown packet — log and ignore
            self.get_logger().debug(f'Unknown packet type "{packet_type}": {raw_line}')

    # ── Publisher helpers ────────────────────────────────────────────────────

    def _publish_state(self, state: str):
        msg      = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _publish_height(self, height_mm: float):
        msg       = Float32()
        msg.data  = height_mm
        self.height_pub.publish(msg)

    def _publish_error(self, text: str):
        """Publish a synthetic error to /lift/status without sending to Arduino."""
        self.get_logger().error(f'[lift_bridge] {text}')
        error_packet = json.dumps({'type': 'error', 'msg': text})
        status_msg      = String()
        status_msg.data = error_packet
        self.status_pub.publish(status_msg)
        self._publish_state('ERROR')

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._shutdown_requested = True
        if self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()