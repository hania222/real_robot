import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool


class StreamlitRosClient(Node):
    """
    ROS 2 node for the Streamlit process only. Streamlit runs as a separate
    OS process outside the ROS 2 launch stack, so it needs its own node to
    publish /lift/command and /carriage/command and read back state topics.
    It does NOT touch serial — robot_bridge_node.py still owns both serial
    ports and does all Arduino communication, unchanged. This node just
    talks to it over the same ROS topics robot_bridge_node.py already exposes.

    It also publishes /mission/busy directly (True while a pick sequence is
    running, False otherwise). This replaces mission_node.py's role in the
    safety interlock: joystick_ws_node.py subscribes to /mission/busy and
    zeroes /cmd_vel while it's True, so the base can't be driven while the
    lift/carriage are mid-sequence — without needing a separate mission node.
    """

    def __init__(self):
        super().__init__('warehouse_streamlit_client')

        self.lift_command_pub     = self.create_publisher(String, '/lift/command', 10)
        self.carriage_command_pub = self.create_publisher(String, '/carriage/command', 10)
        self.mission_busy_pub     = self.create_publisher(Bool, '/mission/busy', 10)

        self.create_subscription(String,  '/lift/state',       self._on_lift_state,      10)
        self.create_subscription(Float32, '/lift/height_mm',   self._on_lift_height,     10)
        self.create_subscription(String,  '/carriage/state',   self._on_carriage_state,  10)
        self.create_subscription(String,  '/lift/status',      self._on_lift_status,     10)
        self.create_subscription(String,  '/carriage/status',  self._on_carriage_status, 10)

        self.lift_state          = 'UNKNOWN'
        self.lift_height_mm      = 0.0
        self.carriage_state      = 'UNKNOWN'
        self.last_lift_error     = None
        self.last_carriage_error = None

    # ── incoming state ──────────────────────────────────────────────
    def _on_lift_state(self, msg: String):
        self.lift_state = msg.data

    def _on_lift_height(self, msg: Float32):
        self.lift_height_mm = msg.data

    def _on_carriage_state(self, msg: String):
        self.carriage_state = msg.data

    def _on_lift_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('type') == 'error':
            self.last_lift_error = data.get('msg', 'unknown error')

    def _on_carriage_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('type') == 'error':
            self.last_carriage_error = data.get('msg', 'unknown error')

    # ── outgoing commands ───────────────────────────────────────────
    def send_lift_home(self):
        self._publish(self.lift_command_pub, {'cmd': 'home'})

    def send_lift_to_height(self, height_mm: float):
        self._publish(self.lift_command_pub, {'cmd': 'lift', 'height_mm': height_mm})

    def send_carriage_home(self):
        self._publish(self.carriage_command_pub, {'cmd': 'home'})

    def send_carriage_pick(self):
        self._publish(self.carriage_command_pub, {'cmd': 'pick'})

    def set_mission_busy(self, busy: bool):
        """Tell joystick_ws_node.py whether it should zero /cmd_vel right now."""
        msg = Bool()
        msg.data = busy
        self.mission_busy_pub.publish(msg)

    def _publish(self, publisher, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        publisher.publish(msg)
        self.last_lift_error     = None
        self.last_carriage_error = None


_client_lock    = threading.Lock()
_client_instance = None
_spin_thread     = None


def get_ros_client() -> StreamlitRosClient:
    """
    Returns a single, process-wide StreamlitRosClient, creating it (and its
    background spin thread) on first call. Streamlit reruns the whole
    script on every interaction, so this lives in module-level globals
    rather than st.session_state, guarded by a lock against concurrent reruns.
    """
    global _client_instance, _spin_thread

    with _client_lock:
        if _client_instance is None:
            if not rclpy.ok():
                rclpy.init()
            _client_instance = StreamlitRosClient()
            _spin_thread = threading.Thread(
                target=rclpy.spin, args=(_client_instance,), daemon=True
            )
            _spin_thread.start()

    return _client_instance