#pragma once

// ═══════════════════════════════════════════════════════
//  Robot geometry
// ═══════════════════════════════════════════════════════
#define WHEEL_RADIUS      0.065f   // metres — measure to confirm
#define WHEEL_BASE        0.642f   // metres — from URDF joints
#define GEAR_RATIO        34.0f    // JGB37-545 200RPM
#define ENCODER_PPR       7.0f     // MY-37 pulses per motor shaft rotation
// Counts per wheel revolution (single edge, one channel)
#define COUNTS_PER_REV    (ENCODER_PPR * GEAR_RATIO)   // = 238

// ═══════════════════════════════════════════════════════
//  BTS7960 motor driver pins
//  One BTS7960 board per motor
//  RPWM = forward PWM, LPWM = reverse PWM
//  R_EN + L_EN = enable (must be HIGH to drive)
// ═══════════════════════════════════════════════════════
// Left motor
#define L_RPWM  25   // forward
#define L_LPWM  26   // reverse
#define L_R_EN  27   // enable forward half-bridge
#define L_L_EN  14   // enable reverse half-bridge

// Right motor
#define R_RPWM  32   // forward
#define R_LPWM  33   // reverse
#define R_R_EN  15   // enable forward half-bridge
#define R_L_EN   4   // enable reverse half-bridge

// ═══════════════════════════════════════════════════════
//  Encoder pins (interrupt-capable GPIO on ESP32)
// ═══════════════════════════════════════════════════════
#define LEFT_ENC_A   18   // left wheel encoder signal
#define RIGHT_ENC_A  19   // right wheel encoder signal

// ═══════════════════════════════════════════════════════
//  MPU-6050 I2C
// ═══════════════════════════════════════════════════════
#define IMU_SDA  21
#define IMU_SCL  22

// ═══════════════════════════════════════════════════════
//  micro-ROS serial
// ═══════════════════════════════════════════════════════
#define MICROROS_SERIAL_BAUD  921600

// ═══════════════════════════════════════════════════════
//  PID gains — tune on real hardware
// ═══════════════════════════════════════════════════════
#define PID_KP   8.0f
#define PID_KI   2.0f
#define PID_KD   0.1f

// ═══════════════════════════════════════════════════════
//  Timing
// ═══════════════════════════════════════════════════════
#define ODOM_PUBLISH_MS    50    // 20 Hz
#define IMU_PUBLISH_MS     20    // 50 Hz
#define CMD_TIMEOUT_MS    500    // stop if no cmd_vel for 500ms
#define PWM_FREQ          1000   // Hz
#define PWM_RESOLUTION       8   // bits → 0-255