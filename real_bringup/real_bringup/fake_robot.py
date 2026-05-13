#!/usr/bin/env python3
"""
Fake Robot Node — practice PlotJuggler + rqt PID tuning without hardware.

Publishes:  /pid_debug        (std_msgs/Float32MultiArray)
            /odom             (nav_msgs/Odometry)
Subscribes: /cmd_vel_stamped  (geometry_msgs/TwistStamped)
            /pid_gains        (std_msgs/String)

The "robot" is a first-order system: actual speed chases target speed
with time constant TAU (simulates motor + gearbox inertia).
PID drives it just like the real firmware.

Run:
    python3 fake_robot.py
Then open PlotJuggler, subscribe to /pid_debug.
Use rqt → Topic Publisher to send new gains to /pid_gains.
Use rqt → Topic Publisher OR teleop_twist_keyboard for /cmd_vel_stamped.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math

# Simulated plant parameters 
WHEEL_BASE   = 0.30          # metres  
WHEEL_RADIUS = 0.065         # metres
TAU          = 0.25          # seconds — motor+gearbox inertia time constant
                             # lower = snappier motor, higher = more sluggish
MAX_SPEED    = 0.5           # m/s — physical speed limit of the fake wheels
DT           = 0.05          # seconds — 20 Hz loop, matches ODOM_PUBLISH_MS

# Default PID gains (tuned live via /pid_gains topic)
DEFAULT_KP = 1.0
DEFAULT_KI = 0.0
DEFAULT_KD = 0.0

# ── PID output limits ─────────────────────────────────────────────────────────
PID_MIN = -255.0
PID_MAX =  255.0
# Map PWM → m/s for the fake plant (255 PWM → MAX_SPEED m/s)
PWM_TO_MPS = MAX_SPEED / PID_MAX


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.debug_pub = self.create_publisher(Float32MultiArray, '/pid_debug', 10) #for PlotJuggler
        self.odom_pub  = self.create_publisher(Odometry,          '/odom',      10) #like encoders 
        self.create_subscription(TwistStamped, '/cmd_vel_stamped',self.cmd_callback,   10)
        self.create_subscription(String,'/pid_gains',self.gains_callback, 10)

        self.timer = self.create_timer(DT, self.update)

        #Simulated state variables
        self.target_left  = 0.0    # m/s : what the controller wants
        self.target_right = 0.0
        self.actual_left  = 0.0    # m/s — what the fake motor is doing
        self.actual_right = 0.0

        # PID state
        self.integral_left  = 0.0
        self.integral_right = 0.0
        self.prev_err_left  = 0.0
        self.prev_err_right = 0.0

        # gains(live-tunable via /pid_gains topic)
        self.kp_l = DEFAULT_KP;  
        self.ki_l = DEFAULT_KI; 
        self.kd_l = DEFAULT_KD
        self.kp_r = DEFAULT_KP; 
        self.ki_r = DEFAULT_KI; 
        self.kd_r = DEFAULT_KD

        # Odometry position
        self.pos_x   = 0.0
        self.pos_y   = 0.0
        self.heading = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.CMD_TIMEOUT   = 0.5   # seconds

        self.get_logger().info('Fake robot ready.')
        self.get_logger().info('PlotJuggler: subscribe to /pid_debug')
        self.get_logger().info('rqt Topic Publisher: publish String to /pid_gains')
        self.get_logger().info('  e.g.  "kp_l:2.0 ki_l:0.1 kd_l:0.0 kp_r:2.0 ki_r:0.1 kd_r:0.0"')

    # cmd_vel callback 
    def cmd_callback(self, msg: TwistStamped):
        linear  = msg.twist.linear.x
        angular = msg.twist.angular.z
        self.target_left  = linear - (angular * WHEEL_BASE / 2.0)
        self.target_right = linear + (angular * WHEEL_BASE / 2.0)
        self.last_cmd_time = self.get_clock().now()

    # gains callback 
    def gains_callback(self, msg: String):
        """
        Parse: "kp_l:1.5 ki_l:0.1 kd_l:0.0 kp_r:1.5 ki_r:0.1 kd_r:0.0"
        Any missing token keeps its current value (safe partial updates)
        """
        tokens = {}
        for part in msg.data.strip().split():
            if ':' in part:
                key, val = part.split(':', 1)
                try:
                    tokens[key] = float(val)
                except ValueError:
                    pass

        self.kp_l = tokens.get('kp_l', self.kp_l)
        self.ki_l = tokens.get('ki_l', self.ki_l)
        self.kd_l = tokens.get('kd_l', self.kd_l)
        self.kp_r = tokens.get('kp_r', self.kp_r)
        self.ki_r = tokens.get('ki_r', self.ki_r)
        self.kd_r = tokens.get('kd_r', self.kd_r)

        # Reset integrals when gains change (same as firmware)
        self.integral_left  = 0.0
        self.integral_right = 0.0
        self.get_logger().info(f'Gains updated → kp_l={self.kp_l} ki_l={self.ki_l} kd_l={self.kd_l} ' f'kp_r={self.kp_r} ki_r={self.ki_r} kd_r={self.kd_r}')

    # PID (identical logic to firmware) 
    def compute_pid(self, target, actual, kp, ki, kd, integral, prev_error):
        error      = target - actual
        integral  += error * DT
        derivative = (error - prev_error) / DT
        prev_error = error

        # integral Anti-windup
        if ki > 1e-6:
            integral = max(PID_MIN / ki, min(PID_MAX / ki, integral))
        else:
            integral = 0.0

        output = kp * error + ki * integral + kd * derivative
        output = max(PID_MIN, min(PID_MAX, output))
        return output, integral, prev_error

    # First-order plant simulation
    def simulate_motor(self, actual_speed, pwm_command):
        """
        Motor + gearbox modelled as a first-order low-pass filter
        Desired speed from PWM:  v_desired = pwm * PWM_TO_MPS
        actual chases v_desired with time constant TAU
        Euler step:  actual += (v_desired - actual) * (DT / TAU)
        """
        v_desired = pwm_command * PWM_TO_MPS
        v_desired = max(-MAX_SPEED, min(MAX_SPEED, v_desired))
        actual_speed += (v_desired - actual_speed) * (DT / TAU)
        return actual_speed

    #  Main 20 Hz update loop
    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9
        # Timeout, stop if no command for CMD_TIMEOUT seconds
        if elapsed > self.CMD_TIMEOUT:
            self.target_left  = 0.0
            self.target_right = 0.0
            self.integral_left  = 0.0
            self.integral_right = 0.0

        # PID calculations for left and right wheels (identical to firmware logic)
        pwm_left, self.integral_left, self.prev_err_left = self.compute_pid(
            self.target_left,  self.actual_left,
            self.kp_l, self.ki_l, self.kd_l,
            self.integral_left,  self.prev_err_left)

        pwm_right, self.integral_right, self.prev_err_right = self.compute_pid(
            self.target_right, self.actual_right,
            self.kp_r, self.ki_r, self.kd_r,
            self.integral_right, self.prev_err_right)

        # simulate motor response 
        self.actual_left  = self.simulate_motor(self.actual_left,  pwm_left)
        self.actual_right = self.simulate_motor(self.actual_right, pwm_right)

        #Odometry
        dl = self.actual_left  * DT
        dr = self.actual_right * DT
        d_centre  = (dl + dr) / 2.0
        d_heading = (dr - dl) / WHEEL_BASE
        self.heading += d_heading
        self.pos_x   += d_centre * math.cos(self.heading)
        self.pos_y   += d_centre * math.sin(self.heading)

        # ── Publish /pid_debug ────────────────────────────────────────────────
        # Layout mirrors your firmware exactly so PlotJuggler setup is identical:
        #   data[0]=target_L  data[1]=actual_L  data[2]=error_L  data[3]=pwm_L
        #   data[4]=target_R  data[5]=actual_R  data[6]=error_R  data[7]=pwm_R
        dbg = Float32MultiArray()
        dbg.data = [
            float(self.target_left),
            float(self.actual_left),
            float(self.target_left  - self.actual_left),
            float(pwm_left),
            float(self.target_right),
            float(self.actual_right),
            float(self.target_right - self.actual_right),
            float(pwm_right),
        ]
        self.debug_pub.publish(dbg)

        #  Publish /odom
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        odom.pose.pose.orientation.z = math.sin(self.heading / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.heading / 2.0)
        odom.twist.twist.linear.x  = d_centre / DT
        odom.twist.twist.angular.z = d_heading / DT
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()