#!/usr/bin/env python3
import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster
import serial

WHEEL_RADIUS = 0.065   
WHEEL_BASE   = 0.642    


class HardwareBridgeNode(Node):

    def __init__(self):
        super().__init__('hardware_bridge_node')

        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baud_rate',   115200)
        port      = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.serial_conn = serial.Serial(port, baud_rate, timeout=0.1)
        self.get_logger().info(f'Serial open: {port} @ {baud_rate} baud')

        # publishers
        self.joint_states_pub = self.create_publisher(JointState,'/joint_states', 10)
        self.imu_pub          = self.create_publisher(Imu, '/imu/data',10)
        self.pid_debug_pub    = self.create_publisher(Float64MultiArray,  '/pid_debug',   10)
        self.odom_pub         = self.create_publisher(Odometry,          '/odom',         10)

        # broadcasts the odom → CHASSIS transform so the TF tree stays connected
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscribers
        self.create_subscription(Twist,  '/cmd_vel',   self._on_cmd_vel,   10)
        self.create_subscription(String, '/pid_gains', self._on_pid_gains, 10)

        # odometry state — updated every feedback packet
        self._pos_x           = 0.0
        self._pos_y           = 0.0
        self._heading         = 0.0   # yaw (radians)
        self._last_odom_time  = time.monotonic()  # storing last time we received an odometry packet to compute dt for dead-reckoning

        # serial reader thread and shutdown flag (to cleanly stop the thread on node shutdown)
        self._shutdown_requested = False

        # start the serial reader loop in a separate thread so it doesn't block the ROS callbacks
        threading.Thread(target=self._serial_reader_loop, daemon=True).start()

    # Outgoing: ROS -> ESP32
    def _on_cmd_vel(self, msg: Twist):
        linear  = msg.linear.x
        angular = msg.angular.z
        # send both wheel targets in ONE serial message so the ESP32 applies them
        self._send({
            'left_target':  round(linear - angular * WHEEL_BASE / 2.0, 4),
            'right_target': round(linear + angular * WHEEL_BASE / 2.0, 4),
        })

    def _on_pid_gains(self, msg: String):
        try:
        # convert the incoming string JSON string message to a dict
            raw = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'/pid_gains bad JSON: {e}')
            return

        # extract PID gains from the raw data and put them the gain dict, supporting both unified keys (kp/ki/kd) and separate left/right keys (kp_l/ki_l/kd_l and kp_r/ki_r/kd_r)
        gains = {}
        for name in ('kp', 'ki', 'kd'):
            value = raw.get(name) or raw.get(f'{name}_l') or raw.get(f'{name}_r')
            if value is not None:
                gains[name] = round(float(value), 4)

        if gains:
            self._send(gains)
            self.get_logger().info(f'PID gains sent: {gains}')
        else:
            self.get_logger().warn('/pid_gains: no kp/ki/kd keys found')

    def _send(self, data: dict):
        try:
            self.serial_conn.write((json.dumps(data) + '\n').encode())
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # Incomings: ESP32 -> ROS
    # Reads lines of JSON from the serial port, parses them, and publishes to the appropriate ROS topics
    def _serial_reader_loop(self):
        while not self._shutdown_requested:
            try:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith('{'):
                    continue
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
                continue

            if 'info' in data or 'warn' in data:
                key = 'info' if 'info' in data else 'warn'
                self.get_logger().info(f'ESP32: {data[key]}')
                continue

            now = self.get_clock().now().to_msg()
            self._publish_joint_states(data, now)
            self._publish_imu(data, now)
            self._publish_pid_debug(data)
            self._publish_odom(data, now)

    # extract joint states from the coming data dict & publish a joint state msg with the left and right wheel positions (in radians) and velocities (in radians/s),
    # converting from the raw position/velocity values (in metres and metres/s) using the wheel radius.
    def _publish_joint_states(self, data: dict, timestamp):
        msg              = JointState()
        msg.header.stamp = timestamp
        msg.name         = ['left_drive', 'right_drive']
        msg.position     = [data.get('left_position',  0.0) / WHEEL_RADIUS,  # rad
                            data.get('right_position', 0.0) / WHEEL_RADIUS]
        msg.velocity     = [data.get('left_velocity',  0.0) / WHEEL_RADIUS,  # rad/s
                            data.get('right_velocity', 0.0) / WHEEL_RADIUS]
        msg.effort       = []
        self.joint_states_pub.publish(msg)

    def _publish_imu(self, data: dict, timestamp):
        msg                           = Imu()
        msg.header.stamp              = timestamp
        msg.header.frame_id           = 'imu_link'
        msg.linear_acceleration.x     = data.get('accel_x', 0.0)
        msg.linear_acceleration.y     = data.get('accel_y', 0.0)
        msg.linear_acceleration.z     = data.get('accel_z', 0.0)
        msg.angular_velocity.x        = data.get('gyro_x',  0.0)
        msg.angular_velocity.y        = data.get('gyro_y',  0.0)
        msg.angular_velocity.z        = data.get('gyro_z',  0.0)
        msg.orientation_covariance[0] = -1.0   # MPU-6050 gives no orientation
        self.imu_pub.publish(msg)

    def _publish_pid_debug(self, data: dict):
        r        = WHEEL_RADIUS
        msg      = Float64MultiArray()
        msg.data = [
            data.get('left_target',    0.0) / r,   # [0] left  target  (rad/s)
            data.get('left_velocity',  0.0) / r,   # [1] left  actual  (rad/s)
            data.get('left_error',     0.0) / r,   # [2] left  error   (rad/s)
            data.get('left_pwm',       0.0),        # [3] left  PWM
            data.get('right_target',   0.0) / r,   # [4] right target  (rad/s)
            data.get('right_velocity', 0.0) / r,   # [5] right actual  (rad/s)
            data.get('right_error',    0.0) / r,   # [6] right error   (rad/s)
            data.get('right_pwm',      0.0),        # [7] right PWM
        ]
        self.pid_debug_pub.publish(msg)

    def _publish_odom(self, data: dict, timestamp):
        # time delta
        now_mono = time.monotonic()
        dt       = now_mono - self._last_odom_time
        self._last_odom_time = now_mono

        # skip bad dt (first packet or stall > 1 s)
        if dt <= 0.0 or dt > 1.0:
            return

        # differential drive dead-reckoning
        # robot forward velocity and yaw rate from left/right wheel speeds
        left_vel  = data.get('left_velocity',  0.0)   # m/s
        right_vel = data.get('right_velocity', 0.0)   # m/s

        linear_vel  = (left_vel + right_vel) / 2.0          # m/s forward
        angular_vel = (right_vel - left_vel) / WHEEL_BASE   # rad/s yaw

        # integrate heading first, then position
        self._heading += angular_vel * dt

        # keep heading in [-π, π] to prevent floating-point drift
        self._heading  = math.atan2(math.sin(self._heading), math.cos(self._heading))

        self._pos_x += linear_vel * math.cos(self._heading) * dt  # cosine is the x component of the forward velocity
        self._pos_y += linear_vel * math.sin(self._heading) * dt

        # yaw angle -> quaternion (z-rotation only, 2D robot)
        qz = math.sin(self._heading / 2.0)
        qw = math.cos(self._heading / 2.0)

        # publish /odom
        odom                         = Odometry()
        odom.header.stamp            = timestamp
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'CHASSIS'

        odom.pose.pose.position.x    = self._pos_x
        odom.pose.pose.position.y    = self._pos_y
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x    = linear_vel
        odom.twist.twist.angular.z   = angular_vel

        # simple diagonal covariance assumptions
        odom.pose.covariance[0]      = 0.001   # x variance
        odom.pose.covariance[7]      = 0.001   # y variance
        odom.pose.covariance[35]     = 0.05    # yaw variance
        odom.twist.covariance[0]     = 0.001   # vx variance
        odom.twist.covariance[35]    = 0.05    # vyaw variance

        self.odom_pub.publish(odom)

        # broadcast odom → CHASSIS TF
        # needed so robot_state_publisher can complete the full TF tree
        tf_msg                         = TransformStamped()
        tf_msg.header.stamp            = timestamp
        tf_msg.header.frame_id         = 'odom'
        tf_msg.child_frame_id          = 'CHASSIS'
        tf_msg.transform.translation.x = self._pos_x
        tf_msg.transform.translation.y = self._pos_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x    = 0.0
        tf_msg.transform.rotation.y    = 0.0
        tf_msg.transform.rotation.z    = qz
        tf_msg.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    # Cleanup serial connection on shutdown
    def destroy_node(self):
        self._shutdown_requested = True
        if self.serial_conn.is_open:
            self.serial_conn.close()
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