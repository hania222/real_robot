import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

# ── Test parameters ───────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.3    # m/s  — forward speed during ON phase
ANGULAR_SPEED = 0.5    # rad/s — yaw rate during ON phase

STEP_DURATION = 3.0    # seconds the command is ON
PERIOD        = 6.0    # total cycle length (3 s ON + 3 s OFF)
DT            = 0.05   # timer period → 20 Hz publish rate


class StepTester(Node):
    def __init__(self, mode: str, once: bool):
        super().__init__('step_tester')
        self.mode = mode
        self.once = once
        self._step_sent = False   # used with --once flag

        # /cmd_vel  → diff_drive_controller → wheel velocity controllers → bridge
        self.pub_twist = self.create_publisher(Twist,        '/cmd_vel',         10)
        # /cmd_vel_stamped → direct path (joystick / legacy) — kept for compatibility
        self.pub_stamped = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)

        self.create_timer(DT, self._update)
        self.t = 0.0

        self.get_logger().info(
            f'\n'
            f'  Step tester mode : [{self.mode}]{"  (single shot)" if self.once else ""}\n'
            f'  Step ON          : {STEP_DURATION} s every {PERIOD} s\n'
            f'  Linear speed     : {LINEAR_SPEED} m/s\n'
            f'  Angular speed    : {ANGULAR_SPEED} rad/s\n'
            f'\n'
            f'  PlotJuggler: subscribe to /pid_debug\n'
            f'    data[0] = target left  (rad/s)   data[1] = actual left  (rad/s)\n'
            f'    data[4] = target right (rad/s)   data[5] = actual right (rad/s)\n'
            f'\n'
            f'  Runtime gain update (no reflash):\n'
            f'    ros2 topic pub --once /pid_gains std_msgs/msg/String \\\n'
            f'      \'data: "{{\\\"kp\\\": 80.0, \\\"ki\\\": 5.0, \\\"kd\\\": 1.0}}"\'\n'
        )

    # ── Timer callback ────────────────────────────────────────────────────────

    def _update(self):
        on = (self.t % PERIOD) < STEP_DURATION

        # --once: publish one ON phase then stop and shutdown
        if self.once:
            if self._step_sent and not on:
                # Step is over — send a stop command and exit
                self._publish(0.0, 0.0)
                self.get_logger().info('Single step complete — shutting down.')
                raise SystemExit
            if on:
                self._step_sent = True

        lx = az = 0.0

        if self.mode == 'linear':
            lx = LINEAR_SPEED if on else 0.0
            az = 0.0

        elif self.mode == 'angular':
            lx = 0.0
            az = ANGULAR_SPEED if on else 0.0

        elif self.mode == 'combined':
            lx = LINEAR_SPEED  if on else 0.0
            az = ANGULAR_SPEED if on else 0.0

        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')
            return

        self._publish(lx, az)
        self.t += DT

    def _publish(self, linear_x: float, angular_z: float):
        now = self.get_clock().now().to_msg()

        # Plain Twist on /cmd_vel — consumed by diff_drive_controller
        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        self.pub_twist.publish(twist)

        # Stamped version on /cmd_vel_stamped — consumed by legacy joystick path
        stamped = TwistStamped()
        stamped.header.stamp    = now
        stamped.header.frame_id = 'CHASSIS'
        stamped.twist.linear.x  = linear_x
        stamped.twist.angular.z = angular_z
        self.pub_stamped.publish(stamped)


def main():
    args   = sys.argv[1:]
    once   = '--once' in args
    modes  = [a for a in args if not a.startswith('--')]
    mode   = modes[0] if modes else 'linear'

    rclpy.init()
    node = StepTester(mode, once)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()