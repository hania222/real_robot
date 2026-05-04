#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twiststamped')

        # ESP32 subscribes to /cmd_vel_stamped as TwistStamped
        # joystick_ws_node publishes Twist on /cmd_vel
        # this node bridges the two
        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_stamped',
            10
        )
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )
        self.get_logger().info('twist_to_twiststamped node started')

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'CHASSIS'
        stamped.twist = msg
        self.pub.publish(stamped)


def main():
    rclpy.init()
    node = TwistToTwistStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()