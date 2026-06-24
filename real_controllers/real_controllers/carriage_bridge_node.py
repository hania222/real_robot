#!/usr/bin/env python3
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class CarriageNode(Node):

    def __init__(self):
        super().__init__('carriage_node')

        self.declare_parameter('serial_port', '/dev/carriage_esp32')
        self.declare_parameter('baud_rate',   115200)
        port      = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.serial_conn = serial.Serial(port, baud_rate, timeout=0.1)
        self.get_logger().info(f'Serial open: {port} @ {baud_rate} baud')

        # publishers
        # /carriage/status  — raw JSON packet from ESP32, forwarded as-is for full detail
        # /carriage/state   — simple human-readable state string for other nodes to branch on
        self.status_pub = self.create_publisher(String, '/carriage/status', 10)
        self.state_pub  = self.create_publisher(String, '/carriage/state',  10)

        # subscriber — Pi sends commands here as JSON strings
        # Examples:
        #   {"cmd": "home"}
        #   {"cmd": "pick", "rotation": 400, "extension": 6154}
        #   {"cmd": "status"}
        #   {"cmd": "stop"}
        self.create_subscription(String, '/carriage/command', self._on_command, 10)

        # track current known state so we can publish /carriage/state on every packet
        self._current_state = 'UNKNOWN'
        self._is_homed      = False

        # shutdown flag — set to True in destroy_node() to stop the reader thread cleanly
        self._shutdown_requested = False

        # start serial reader in background thread so it never blocks ROS callbacks
        threading.Thread(target=self._serial_reader_loop, daemon=True).start()

        self.get_logger().info('Carriage node ready.')
        self.get_logger().info('  Commands  → /carriage/command  (std_msgs/String, JSON)')
        self.get_logger().info('  Status    ← /carriage/status   (std_msgs/String, JSON)')
        self.get_logger().info('  State     ← /carriage/state    (std_msgs/String, plain text)')

    # Outgoing: ROS → ESP32
    def _on_command(self, msg: String):
        raw = msg.data.strip()

        # validate it is real JSON before forwarding, so we never corrupt the ESP32 serial stream
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as error:
            self.get_logger().warn(f'/carriage/command bad JSON — ignored: {error}')
            return

        # log what we are sending so it is easy to trace during testing
        cmd = parsed.get('cmd', '?')
        self.get_logger().info(f'Sending command to carriage ESP32: cmd={cmd}')

        self._send_raw(raw)

    def _send_raw(self, text: str):
        try:
            self.serial_conn.write((text + '\n').encode())
        except Exception as error:
            self.get_logger().warn(f'Serial write error: {error}')

    # Incoming: ESP32 → ROS
    # Runs in a background thread — reads lines, parses JSON, publishes to ROS topics
    def _serial_reader_loop(self):
        while not self._shutdown_requested:
            try:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith('{'):
                    continue
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            except Exception as error:
                self.get_logger().warn(f'Serial read error: {error}')
                continue

            self._handle_incoming_packet(data, line)

    def _handle_incoming_packet(self, data: dict, raw_line: str):
        packet_type = data.get('type', '')

        # boot message — ESP32 just started up
        if packet_type == 'boot':
            self.get_logger().info(f"Carriage ESP32 booted: {data.get('msg', '')}")
            self._publish_state('IDLE')
            self._publish_status(raw_line)
            return

        # ack — ESP32 acknowledged a command and started executing it
        if packet_type == 'ack':
            self.get_logger().info(f"Carriage ACK: cmd={data.get('cmd', '?')}")
            self._publish_status(raw_line)
            return

        # status packet — periodic debug broadcast from ESP32 (when DEBUG_VERBOSE=true)
        # or response to an explicit {"cmd":"status"} request
        if packet_type == 'status':
            state    = data.get('state', 'UNKNOWN')
            is_homed = data.get('homed', False)
            is_busy  = data.get('busy',  False)
            rot_pos  = data.get('rot_pos', 0)
            ext_pos  = data.get('ext_pos', 0)
            grip     = data.get('grip', '?')

            self._current_state = state
            self._is_homed      = is_homed

            self.get_logger().debug(
                f'Carriage status — state={state} homed={is_homed} busy={is_busy} '
                f'rot={rot_pos} ext={ext_pos} grip={grip}'
            )

            self._publish_state(state)
            self._publish_status(raw_line)
            return

        # home_complete — homing sequence finished (success or failure)
        if packet_type == 'home_complete':
            success = data.get('success', False)
            if success:
                self._is_homed = True
                self.get_logger().info('Carriage HOMED successfully.')
                self._publish_state('IDLE')
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Homing FAILED: {reason}')
                self._publish_state('ERROR')

            self._publish_status(raw_line)
            return

        # pick_complete — pick sequence finished (success or failure)
        if packet_type == 'pick_complete':
            success = data.get('success', False)
            if success:
                self.get_logger().info('Pick sequence COMPLETE.')
                self._publish_state('IDLE')
            else:
                reason = data.get('reason', 'unknown')
                self.get_logger().error(f'Pick FAILED: {reason}')
                self._publish_state('ERROR')

            self._publish_status(raw_line)
            return

        # error — ESP32 encountered an error and moved to S_ERROR
        if packet_type == 'error':
            error_message = data.get('msg', 'unknown error')
            self.get_logger().error(f'Carriage ESP32 error: {error_message}')
            self._publish_state('ERROR')
            self._publish_status(raw_line)
            return

        # unknown packet type — log it and forward it anyway so nothing is silently lost
        self.get_logger().warn(f'Unknown packet type from carriage ESP32: {raw_line}')
        self._publish_status(raw_line)

    def _publish_status(self, raw_json_string: str):
        msg      = String()
        msg.data = raw_json_string
        self.status_pub.publish(msg)

    def _publish_state(self, state_string: str):
        self._current_state = state_string
        msg      = String()
        msg.data = state_string
        self.state_pub.publish(msg)

    # Cleanup serial connection on shutdown
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