#pragma once

// ═══════════════════════════════════════════════════════
//  Robot geometry
// ═══════════════════════════════════════════════════════
#define WHEEL_RADIUS      0.08f  // 160mm diameter → 80mm radius
#define WHEEL_BASE        0.642f   // metres — from URDF joints
#define GEAR_RATIO        99.5f    // TQ42-775 60RPM — actual datasheet gear ratio
#define ENCODER_PPR       7.0f     // MY-37 pulses per motor shaft rotation (7 PPR, 2-channel)
// Counts per wheel revolution.
// ISR triggers on CHANGE (rising + falling) of channel A only → 2× counting.
// 7 PPR × 2 edges × 99.5 gear ratio = 1393 counts/rev.
// If you later switch to X4 (CHANGE on both channels), multiply by 4 instead of 2 → 2786.
#define COUNTS_PER_REV    (ENCODER_PPR * 2.0f * GEAR_RATIO)   // 7 × 2 × 99.5 = 1393

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
#define LEFT_ENC_A   18   // left wheel encoder channel A — interrupt pin
#define RIGHT_ENC_A  19   // right wheel encoder channel A — interrupt pin
#define LEFT_ENC_B   16   // left wheel encoder channel B — direction detection
#define RIGHT_ENC_B  17   // right wheel encoder channel B — direction detection

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
//  PID gains — starting values for real hardware tuning
//
//  YOUR ENCODER: TQ42-775 = 7 PPR × 2 edges (CHANGE mode) × 99.5 gear ratio
//                         = 1393 counts/rev
//  That gives ~0.36mm per tick at the wheel (160mm diameter) — good resolution.
//
//  TUNING ORDER (do not skip steps):
//
//  Step 1 — Kp only, Ki=0, Kd=0
//    Command 0.2 m/s forward. Raise Kp until robot nearly reaches target.
//    If wheels oscillate (speed bounces up and down) → lower Kp.
//    Good Kp: robot reaches ~90% of target speed, no oscillation.
//    Typical starting range for this motor+encoder: 80–150
//    (Yes, these gains are high because input is m/s but output is 0-255 PWM)
//
//  Step 2 — Add Ki to fix steady-state error (robot settles below target)
//    Start at Ki = 0.5, raise slowly.
//    Watch /pid_debug data[2] (left error) in PlotJuggler — it should go to 0.
//    If robot overshoots and keeps oscillating → reduce Ki.
//
//  Step 3 — Add Kd only if overshoot is still visible after Ki is set
//    Start at Kd = 0.5. The gearbox backlash and inter-tick noise on this
//    motor can make large Kd values cause jitter, so keep it small (0.5–2.0).
//    If jitter appears → set Kd back to 0.
//
//  These defaults are safe starting values (P-only, no drive output).
//  Change them via /pid_gains topic from rqt during testing, then
//  write the final working values back here once found.
// ═══════════════════════════════════════════════════════
#define KP_LEFT    80.0f   // start here, raise until motor responds
#define KI_LEFT     0.0f   // add after Kp is stable
#define KD_LEFT     0.0f   // add last, keep small

#define KP_RIGHT   80.0f
#define KI_RIGHT    0.0f
#define KD_RIGHT    0.0f

#define PID_MAX_OUTPUT   255.0f   // max PWM forward
#define PID_MIN_OUTPUT  -255.0f   // max PWM reverse

// ═══════════════════════════════════════════════════════
//  Timing
// ═══════════════════════════════════════════════════════
#define ODOM_PUBLISH_MS    50    // 20 Hz
#define IMU_PUBLISH_MS     20    // 50 Hz
#define CMD_TIMEOUT_MS    500    // stop if no cmd_vel for 500ms
#define PWM_FREQ          1000   // Hz
#define PWM_RESOLUTION       8   // bits → 0-255
