#!/usr/bin/env python3
"""
robot_bridge_node.py
Bridges BOTH Arduino controllers — the lift (Uno) and the carriage (Uno) —
and ROS 2, in a single node with two independent serial connections.

────────────────────────────────────────────────────────────────────────────
LIFT — Serial protocol (115200 baud, newline-terminated JSON)
  Pi → Uno:  {"cmd":"home"} | {"cmd":"lift","height_mm":100.0} | {"cmd":"status"} | {"cmd":"stop"}
  Uno → Pi:  {"type":"boot"} | {"type":"ack"} | {"type":"home_complete"} |
              {"type":"lift_complete"} | {"type":"status"} | {"type":"error"}

LIFT — ROS interface:
  Subscriptions:
    /lift/command   (std_msgs/String)  — JSON: {"cmd":"home"} or {"cmd":"lift","height_mm":N} or {"cmd":"stop"}
  Publications:
    /lift/status    (std_msgs/String)  — raw status JSON forwarded from Arduino
    /lift/state     (std_msgs/String)  — "IDLE" | "BUSY" | "ERROR" | "HOMING" | "MOVING"
    /lift/height_mm (std_msgs/Float32) — current lift height in mm (from status packets)

────────────────────────────────────────────────────────────────────────────
CARRIAGE — Serial protocol (115200 baud, newline-terminated JSON)
  Pi → Arduino:  {"cmd":"home"} | {"cmd":"pick"} | {"cmd":"status"} | {"cmd":"stop"}
  Arduino → Pi:  {"type":"boot"} | {"type":"ack"} | {"type":"home_complete"} |
                  {"type":"pick_complete"} | {"type":"status"} | {"type":"error"} |
                  {"type":"arm_end_sw"}

CARRIAGE — ROS interface:
  Subscriptions:
    /carriage/command (std_msgs/String) — JSON: {"cmd":"home"} or {"cmd":"pick"} or {"cmd":"stop"}
  Publications:
    /carriage/status   (std_msgs/String) — raw status JSON forwarded from Arduino
    /carriage/state    (std_msgs/String) — "IDLE" | "BUSY" | "ERROR" | "HOMING" | "PICKING"
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import serial


class RobotBridgeNode(Node):

    def __init__(self):
        super().__init__('robot_bridge_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('lift_serial_port', '/dev/lift_arduino')
        self.declare_parameter('lift_baud_rate', 115200)
        self.declare_parameter('carriage_serial_port', '/dev/carriage_arduino')
        self.declare_parameter('carriage_baud_rate', 115200)

        lift_port      = self.get_parameter('lift_serial_port').value
        lift_baud      = self.get_parameter('lift_baud_rate').value
        carriage_port  = self.get_parameter('carriage_serial_port').value
        carriage_baud  = self.get_parameter('carriage_baud_rate').value

        # ── Serial connections ──────────────────────────────────────────────
        self.lift_serial_conn = serial.Serial(lift_port, lift_baud, timeout=0.1)
        self.get_logger().info(f'Lift serial open: {lift_port} @ {lift_baud} baud')

        self.carriage_serial_conn = serial.Serial(carriage_port, carriage_baud, timeout=0.1)
        self.get_logger().info(f'Carriage serial open: {carriage_port} @ {carriage_baud} baud')

        # ── Lift publishers ─────────────────────────────────────────────────
        self.lift_status_pub = self.create_publisher(String,  '/lift/status',    10)
        self.lift_state_pub  = self.create_publisher(String,  '/lift/state',     10)
        self.lift_height_pub = self.create_publisher(Float32, '/lift/height_mm', 10)

        # ── Carriage publishers ─────────────────────────────────────────────
        self.carriage_status_pub = self.create_publisher(String, '/carriage/status', 10)
        self.carriage_state_pub  = self.create_publisher(String, '/carriage/state',  10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, '/lift/command',     self._on_lift_command,     10)
        self.create_subscription(String, '/carriage/command', self._on_carriage_command, 10)

        # ── Internal state — lift ───────────────────────────────────────────
        self._lift_is_homed = False
        self._lift_mm        = 0.0
        self._lift_busy      = False

        # ── Internal state — carriage ───────────────────────────────────────
        self._carriage_is_homed = False
        self._carriage_busy      = False

        # clean shutdown flag (shared by both reader threads)
        self._shutdown_requested = False

        threading.Thread(target=self._lift_serial_reader_loop,     daemon=True).start()
        threading.Thread(target=self._carriage_serial_reader_loop, daemon=True).start()

    # ══════════════════════════════════════════════════════════════════════
    # LIFT
    # ══════════════════════════════════════════════════════════════════════

    # ── Outgoing: ROS → Arduino ──────────────────────────────────────────────

    def _on_lift_command(self, msg: String):
        """
        Forward a command from /lift/command to the lift Arduino over serial.
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
        if cmd == 'lift' and not self._lift_is_homed:
            self._publish_lift_error('not homed — send home command first')
            return

        # guard: reject new commands while the Arduino is busy
        # (stop and status are always allowed through)
        if self._lift_busy and cmd not in ('stop', 'status'):
            self._publish_lift_error('busy — Arduino is still executing previous command, wait for completion')
            return

        self._lift_send(data)
        self.get_logger().info(f'→ Lift Arduino: {raw}')

    def _lift_send(self, data: dict):
        try:
            self.lift_serial_conn.write((json.dumps(data) + '\n').encode())
        except Exception as error:
            self.get_logger().warn(f'Lift serial write error: {error}')

    # ── Incoming: Arduino → ROS ──────────────────────────────────────────────

    def _lift_serial_reader_loop(self):
        while not self._shutdown_requested:
            try:
                raw_line = self.lift_serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not raw_line.startswith('{'):
                    continue
                data = json.loads(raw_line)
            except json.JSONDecodeError:
                continue
            except Exception as error:
                self.get_logger().warn(f'Lift serial read error: {error}')
                continue

            self._dispatch_lift_packet(data, raw_line)

    def _dispatch_lift_packet(self, data: dict, raw_line: str):
        """Route each incoming lift JSON packet to the right handler."""
        packet_type = data.get('type', '')

        if packet_type == 'boot':
            self.get_logger().info(f'Lift Arduino boot: {data.get("msg", "")}')
            self._publish_lift_state('IDLE')

        elif packet_type == 'ack':
            cmd = data.get('cmd', '')
            self.get_logger().info(f'Lift Arduino ack: {cmd}')
            # mark busy as soon as Arduino acknowledges an action command
            if cmd in ('home', 'lift'):
                self._lift_busy = True
                self._publish_lift_state('HOMING' if cmd == 'home' else 'MOVING')

        elif packet_type == 'home_complete':
            success = data.get('success', False)
            if success:
                self._lift_is_homed = True
                self._lift_mm       = 0.0
                self.get_logger().info('Lift homed successfully')
                self._publish_lift_state('IDLE')
                self._publish_lift_height(0.0)
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Lift homing failed: {reason}')
                self._publish_lift_state('ERROR')
            self._lift_busy = False

        elif packet_type == 'lift_complete':
            success = data.get('success', False)
            if success:
                self.get_logger().info(f'Lift move complete — now at {self._lift_mm:.1f} mm')
                self._publish_lift_state('IDLE')
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Lift move failed: {reason}')
                self._publish_lift_state('ERROR')
            self._lift_busy = False

        elif packet_type == 'status':
            # mirror Arduino state into ROS topics
            self._lift_is_homed = data.get('homed', self._lift_is_homed)
            self._lift_mm        = float(data.get('lift_mm', self._lift_mm))
            self._lift_busy      = data.get('busy', self._lift_busy)
            self._publish_lift_height(self._lift_mm)
            self._publish_lift_state(data.get('state', 'IDLE'))
            # forward the full raw JSON so task_manager can inspect it if needed
            status_msg      = String()
            status_msg.data = raw_line
            self.lift_status_pub.publish(status_msg)

        elif packet_type == 'error':
            error_text = data.get('msg', 'unknown error')
            self.get_logger().error(f'Lift Arduino error: {error_text}')
            self._lift_busy = False
            self._publish_lift_state('ERROR')
            # also forward as a status message so callers subscribed to /lift/status see it
            status_msg      = String()
            status_msg.data = raw_line
            self.lift_status_pub.publish(status_msg)

        else:
            # unknown packet — log and ignore
            self.get_logger().debug(f'Unknown lift packet type "{packet_type}": {raw_line}')

    # ── Publisher helpers ────────────────────────────────────────────────────

    def _publish_lift_state(self, state: str):
        msg      = String()
        msg.data = state
        self.lift_state_pub.publish(msg)

    def _publish_lift_height(self, height_mm: float):
        msg      = Float32()
        msg.data = height_mm
        self.lift_height_pub.publish(msg)

    def _publish_lift_error(self, text: str):
        """Publish a synthetic error to /lift/status without sending to Arduino."""
        self.get_logger().error(f'[lift_bridge] {text}')
        error_packet    = json.dumps({'type': 'error', 'msg': text})
        status_msg      = String()
        status_msg.data = error_packet
        self.lift_status_pub.publish(status_msg)
        self._publish_lift_state('ERROR')

    # ══════════════════════════════════════════════════════════════════════
    # CARRIAGE
    # ══════════════════════════════════════════════════════════════════════

    # ── Outgoing: ROS → Arduino ──────────────────────────────────────────────

    def _on_carriage_command(self, msg: String):
        """
        Forward a command from /carriage/command to the carriage Arduino over serial.
        Accepted payloads:
          {"cmd":"home"}
          {"cmd":"pick"}
          {"cmd":"stop"}
          {"cmd":"status"}
        Reject pick commands while not homed or while busy so the
        task_manager gets an immediate error instead of a silent queue.
        """
        raw = msg.data.strip()

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as error:
            self.get_logger().warn(f'/carriage/command bad JSON: {error}')
            return

        cmd = data.get('cmd', '')

        # guard: pick requires homing first
        if cmd == 'pick' and not self._carriage_is_homed:
            self._publish_carriage_error('not homed — send home command first')
            return

        # guard: reject new commands while the Arduino is busy
        # (stop and status are always allowed through)
        if self._carriage_busy and cmd not in ('stop', 'status'):
            self._publish_carriage_error('busy — Arduino is still executing previous command, wait for completion')
            return

        self._carriage_send(data)
        self.get_logger().info(f'→ Carriage Arduino: {raw}')

    def _carriage_send(self, data: dict):
        try:
            self.carriage_serial_conn.write((json.dumps(data) + '\n').encode())
        except Exception as error:
            self.get_logger().warn(f'Carriage serial write error: {error}')

    # ── Incoming: Arduino → ROS ──────────────────────────────────────────────

    def _carriage_serial_reader_loop(self):
        while not self._shutdown_requested:
            try:
                raw_line = self.carriage_serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not raw_line.startswith('{'):
                    continue
                data = json.loads(raw_line)
            except json.JSONDecodeError:
                continue
            except Exception as error:
                self.get_logger().warn(f'Carriage serial read error: {error}')
                continue

            self._dispatch_carriage_packet(data, raw_line)

    def _dispatch_carriage_packet(self, data: dict, raw_line: str):
        """Route each incoming carriage JSON packet to the right handler."""
        packet_type = data.get('type', '')

        if packet_type == 'boot':
            self.get_logger().info(f'Carriage Arduino boot: {data.get("msg", "")}')
            self._publish_carriage_state('IDLE')

        elif packet_type == 'ack':
            cmd = data.get('cmd', '')
            self.get_logger().info(f'Carriage Arduino ack: {cmd}')
            # mark busy as soon as Arduino acknowledges an action command
            if cmd in ('home', 'pick'):
                self._carriage_busy = True
                self._publish_carriage_state('HOMING' if cmd == 'home' else 'PICKING')

        elif packet_type == 'home_complete':
            success = data.get('success', False)
            if success:
                self._carriage_is_homed = True
                self.get_logger().info('Carriage homed successfully')
                self._publish_carriage_state('IDLE')
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Carriage homing failed: {reason}')
                self._publish_carriage_state('ERROR')
            self._carriage_busy = False

        elif packet_type == 'pick_complete':
            success = data.get('success', False)
            if success:
                self.get_logger().info('Carriage pick sequence complete')
                self._publish_carriage_state('IDLE')
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Carriage pick failed: {reason}')
                self._publish_carriage_state('ERROR')
            self._carriage_busy = False

        elif packet_type == 'status':
            # mirror Arduino state into ROS topics
            self._carriage_is_homed = data.get('homed', self._carriage_is_homed)
            self._carriage_busy     = data.get('busy', self._carriage_busy)
            self._publish_carriage_state(data.get('state', 'IDLE'))
            # forward the full raw JSON so task_manager can inspect it if needed
            status_msg      = String()
            status_msg.data = raw_line
            self.carriage_status_pub.publish(status_msg)

        elif packet_type == 'arm_end_sw':
            # non-fatal notice from the Arduino that the extension end-of-travel
            # switch fired mid-sequence — forward it, but it is not an error state
            self.get_logger().warn(f'Carriage arm end switch: {data.get("msg", "")}')
            status_msg      = String()
            status_msg.data = raw_line
            self.carriage_status_pub.publish(status_msg)

        elif packet_type == 'error':
            error_text = data.get('msg', 'unknown error')
            self.get_logger().error(f'Carriage Arduino error: {error_text}')
            self._carriage_busy = False
            self._publish_carriage_state('ERROR')
            # also forward as a status message so callers subscribed to /carriage/status see it
            status_msg      = String()
            status_msg.data = raw_line
            self.carriage_status_pub.publish(status_msg)

        else:
            # unknown packet — log and ignore
            self.get_logger().debug(f'Unknown carriage packet type "{packet_type}": {raw_line}')

    # ── Publisher helpers ────────────────────────────────────────────────────

    def _publish_carriage_state(self, state: str):
        msg      = String()
        msg.data = state
        self.carriage_state_pub.publish(msg)

    def _publish_carriage_error(self, text: str):
        """Publish a synthetic error to /carriage/status without sending to Arduino."""
        self.get_logger().error(f'[carriage_bridge] {text}')
        error_packet    = json.dumps({'type': 'error', 'msg': text})
        status_msg      = String()
        status_msg.data = error_packet
        self.carriage_status_pub.publish(status_msg)
        self._publish_carriage_state('ERROR')

    # ══════════════════════════════════════════════════════════════════════
    # Cleanup
    # ══════════════════════════════════════════════════════════════════════

    def destroy_node(self):
        self._shutdown_requested = True
        if self.lift_serial_conn.is_open:
            self.lift_serial_conn.close()
        if self.carriage_serial_conn.is_open:
            self.carriage_serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()