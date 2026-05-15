#pragma once

//  Robot geometry
#define WHEEL_RADIUS      0.065f        // 130 mm diameter -> 65 mm radius
#define WHEEL_BASE        0.642f        // metres, from URDF joints
#define GEAR_RATIO        30.0f
#define ENCODER_PPR       600.0f        // pulses per motor shaft rotation
#define COUNTS_PER_REV    (ENCODER_PPR * 4.0f * GEAR_RATIO)  // 600 × 4 × 30 = 72 000

//  BTS7960 motor driver pins
//  RPWM = forward PWM, LPWM = reverse PWM, R_EN + L_EN = enable (must be HIGH)
// Left motor
#define L_RPWM  25
#define L_LPWM  26
#define L_R_EN  27
#define L_L_EN  14
// Right motor
#define R_RPWM  32
#define R_LPWM  33
#define R_R_EN  15
#define R_L_EN   4

//  Encoder pins 
//  X4 mode: BOTH channel A and B must be interrupt-capable pins
#define LEFT_ENC_A   18
#define LEFT_ENC_B   16
#define RIGHT_ENC_A  19
#define RIGHT_ENC_B  17

//  IMU I2C pins
#define IMU_SDA  21
#define IMU_SCL  22

//  micro-ROS serial
#define MICROROS_SERIAL_BAUD  921600

//  PID gains 
// noteee:  Motor max rated speed: 176 RPM -> 1.20 m/s at wheel rim
//  With slew limiting (MAX_ACCEL_PER_TICK = 0.05 m/s), the instantaneous
//  speed error during a ramp is small (~0.05 m/s), so Kp only needs to be
//  large enough to produce useful PWM from that small error
//
//  TUNING PROCEDURE:
//    1. kd/Ki=0 ,Raise Kp until wheels track target speed without oscillating
//    2. Add Ki slowly (e.g. 1, 2, 5) until the steady-state offset disappears
//       If wheels start oscillating, halve Ki
//    3. Add Kd (e.g. 0.5, 1, 2) only if the wheel velocity oscillates around
//       the target even with good Kp/Ki.  Kd is often not needed

#define KP_LEFT    80.0f
#define KI_LEFT     5.0f
#define KD_LEFT     1.0f
#define KP_RIGHT   80.0f
#define KI_RIGHT    5.0f
#define KD_RIGHT    1.0f

#define PID_MAX_OUTPUT   255.0f
#define PID_MIN_OUTPUT  -255.0f


//  Timing
#define ODOM_PUBLISH_MS    50     // 20 Hz  —> odometry + PID loop rate
#define IMU_PUBLISH_MS     20     // 50 Hz  —> IMU publish rate
#define CMD_TIMEOUT_MS    500     // stop motors if no cmd_vel for 500 ms
#define PWM_FREQ          1000    // Hz
#define PWM_RESOLUTION       8    // bits: 0–255


//  Stall detection
#define STALL_PWM_THRESHOLD    220.0f
#define STALL_SPEED_THRESHOLD    0.02f   // m/s — below this = "wheel stopped"
#define STALL_TIME_MS           800     // ms before motors are cut on stall

//  Slew rate limiter
//  Start at 0.05,  (If gearbox jolts on start, lower to 0.03. If robot won't reach top speed, raise to 0.1/0.2)
#define MAX_ACCEL_PER_TICK  0.05f   // m/s per 50 ms tick  (= 1.0 m/s^2 acceleration)


//  PWM dead-band
// if wheels twitch at zero command, will raise to 20
//  if the robot won't start moving at slow Nav2 commands, lower to 10
#define PWM_DEADBAND     15     // PWM counts below this → output 0


//  Minimum wheel speed for stall detection
// match STALL_SPEED_THRESHOLD so both checks agree on "wheel stopped" 
#define MIN_WHEEL_SPEED  0.02f  // m/s
#define MAX_PHYSICAL_SPEED  1.4f   // m/s — above rated 1.2 m/s + margin
