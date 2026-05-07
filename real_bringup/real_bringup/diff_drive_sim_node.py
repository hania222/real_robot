#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf2_ros
import math

class DiffDriveSimNode(Node):
    def __init__(self):
        super().__init__('diff_drive_sim')

        self.wheel_radius     = 0.085
        self.wheel_separation = 0.642

        # Wheel angles (for joint_states)
        self.left_pos  = 0.0
        self.right_pos = 0.0

        # Robot pose in odom frame
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.linear_x  = 0.0
        self.angular_z = 0.0

        self.last_time = self.get_clock().now()

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub  = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster — this is what moves the robot in RViz
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info('diff_drive_sim started')

    def cmd_cb(self, msg: Twist):
        self.linear_x  = msg.linear.x
        self.angular_z = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Wheel velocities
        v_left  = self.linear_x - self.angular_z * self.wheel_separation / 2.0
        v_right = self.linear_x + self.angular_z * self.wheel_separation / 2.0

        # Integrate wheel angles
        self.left_pos  += (v_left  / self.wheel_radius) * dt
        self.right_pos += (v_right / self.wheel_radius) * dt

        # Integrate robot pose
        v     = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_separation

        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        stamp = now.to_msg()

        # 1. Publish joint states (wheel rotation)
        js = JointState()
        js.header.stamp = stamp
        js.name     = ['left_wheel', 'right_wheel',
                       'front_swivel', 'front_rolling',
                       'rear_swiveling', 'rear_rolling']
        js.position = [self.left_pos, self.right_pos, 0.0, 0.0, 0.0, 0.0]
        js.velocity = [v_left / self.wheel_radius, v_right / self.wheel_radius,
                       0.0, 0.0, 0.0, 0.0]
        self.joint_pub.publish(js)

        # 2. Broadcast odom → base_link TF (this moves the robot in RViz)
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'CHASSIS'        # must match your URDF root link name
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # Convert heading angle to quaternion (rotation around Z only)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # 3. Publish /odom message (optional but good practice)
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'CHASSIS'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    rclpy.spin(DiffDriveSimNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()