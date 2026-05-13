#!/usr/bin/env python3
"""
Step tester — tests linear, angular, or combined motion
Works with fake_robot.py oR the real ESP32 (same /cmd_vel_stamped topic)

Usage:
    python3 step_tester.py                  # default: linear only
    python3 step_tester.py linear           # straight line
    python3 step_tester.py angular          # spin in place
    python3 step_tester.py combined         # forward + turning
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

#Test parameters 
LINEAR_SPEED  = 0.3    # m/s
ANGULAR_SPEED = 0.5    # rad/s

STEP_DURATION = 3.0    # seconds ON
PERIOD        = 6.0    # total cycle (3s ON, 3s OFF)
DT            = 0.05   # 20 Hz


class StepTester(Node):
    def __init__(self, mode: str):
        super().__init__('step_tester')
        self.mode = mode
        self.pub  = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.create_timer(DT, self.update)
        self.t = 0.0
        self.get_logger().info(f'Step tester mode: [{self.mode}]')
        self.get_logger().info(f'Step ON for {STEP_DURATION}s every {PERIOD}s')
        self.get_logger().info('Watch /pid_debug in PlotJuggler')

    def update(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        on = (self.t % PERIOD) < STEP_DURATION

        if self.mode == 'linear':
            msg.twist.linear.x  = LINEAR_SPEED if on else 0.0
            msg.twist.angular.z = 0.0

        elif self.mode == 'angular':
            msg.twist.linear.x  = 0.0
            msg.twist.angular.z = ANGULAR_SPEED if on else 0.0

        elif self.mode == 'combined':
            msg.twist.linear.x  = LINEAR_SPEED  if on else 0.0
            msg.twist.angular.z = ANGULAR_SPEED if on else 0.0

        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')
            return

        self.pub.publish(msg)
        self.t += DT


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'linear'
    rclpy.init()
    node = StepTester(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()