#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Test parameters
LINEAR_SPEED_MS   = 0.2    # m/s   — forward speed during the ON phase
ANGULAR_SPEED_RS  = 0.3    # rad/s — yaw rate  during the ON phase
STEP_ON_DURATION  = 3.0    # seconds the command is active (ON)
CYCLE_PERIOD      = 6.0    # total cycle: 3 s ON then 3 s OFF
PUBLISH_PERIOD    = 0.05   # timer period → 20 Hz publish rate


class StepTester(Node):

    def __init__(self, mode: str, single_shot: bool):
        super().__init__('step_tester')
        self.mode        = mode
        self.single_shot = single_shot
        self._step_was_on = False   # tracks when the ON phase has fired (--once)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(PUBLISH_PERIOD, self._timer_callback)
        self.elapsed_time = 0.0

        self.get_logger().info(
            f'\n'
            f'  Step tester mode : [{self.mode}]'
            f'{"  (single shot)" if self.single_shot else ""}\n'
            f'  Step ON for      : {STEP_ON_DURATION} s  every  {CYCLE_PERIOD} s\n'
            f'  Linear speed     : {LINEAR_SPEED_MS} m/s\n'
            f'  Angular speed    : {ANGULAR_SPEED_RS} rad/s\n'
            f'\n'
            f'  PlotJuggler  →  subscribe to /pid_debug\n'
            f'    left  step response : data[0] (target)  vs  data[1] (actual)\n'
            f'    right step response : data[4] (target)  vs  data[5] (actual)\n'
            f'    PWM saturation      : data[3] (left)        data[7] (right)\n'
            f'\n'
            f'  Runtime gain update (no reflash):\n'
            f'    ros2 topic pub --once /pid_gains std_msgs/msg/String \\\n'
            f'      \'data: "{{\\\"kp\\\": 80.0, \\\"ki\\\": 5.0, \\\"kd\\\": 1.0}}"\'\n'
        )

    # Timer callback
    def _timer_callback(self):
        step_is_on = (self.elapsed_time % CYCLE_PERIOD) < STEP_ON_DURATION

        # --once: send one ON phase then stop
        if self.single_shot:
            if self._step_was_on and not step_is_on:
                self._publish_velocity(0.0, 0.0)
                self.get_logger().info('Single step complete — shutting down.')
                raise SystemExit
            if step_is_on:
                self._step_was_on = True

        linear_ms  = 0.0
        angular_rs = 0.0

        if self.mode == 'linear':
            linear_ms  = LINEAR_SPEED_MS if step_is_on else 0.0
        elif self.mode == 'angular':
            angular_rs = ANGULAR_SPEED_RS if step_is_on else 0.0
        elif self.mode == 'combined':
            linear_ms  = LINEAR_SPEED_MS  if step_is_on else 0.0
            angular_rs = ANGULAR_SPEED_RS if step_is_on else 0.0
        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')
            return

        self._publish_velocity(linear_ms, angular_rs)
        self.elapsed_time += PUBLISH_PERIOD

    def _publish_velocity(self, linear_ms: float, angular_rs: float):
        twist_msg             = Twist()
        twist_msg.linear.x    = linear_ms
        twist_msg.angular.z   = angular_rs
        self.cmd_vel_pub.publish(twist_msg)


def main():
    cli_args    = sys.argv[1:]
    single_shot = '--once' in cli_args
    mode_args   = [arg for arg in cli_args if not arg.startswith('--')]
    mode        = mode_args[0] if mode_args else 'linear'

    rclpy.init()
    node = StepTester(mode, single_shot)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()