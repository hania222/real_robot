#pragma once

// Robot geometry (kept only because WHEEL_BASE is used to split
// linear/angular velocity into left/right wheel targets in hardware_bridge_node.py)
#define WHEEL_BASE        0.642f        // metres — must match URDF joint origins

// BTS7960 motor driver pins
// RPWM = forward PWM, LPWM = reverse PWM, R_EN + L_EN = enable (must be HIGH)
#define L_RPWM  25
#define L_LPWM  26
#define L_R_EN  27
#define L_L_EN  14

#define R_RPWM  32
#define R_LPWM  33
#define R_R_EN  15
#define R_L_EN   4

// Command timeout — stop motors if no serial command arrives within this window
#define CMD_TIMEOUT_MS   500

// Slew-rate limiter — limits how fast the target velocity can change per tick
// Lower → smoother start, slower response
// Higher → faster response, more current spike on startup
#define MAX_ACCEL_PER_TICK  0.01f   // m/s per tick (= 0.5 m/s² at 50 Hz)

// PWM dead-band — PWM values below this are forced to zero
// Raise to 20 if wheels twitch at zero command
// Lower to 10 if robot won't start moving on slow joystick commands
#define PWM_DEADBAND     15

#define PWM_MAX_OUTPUT   255.0f
#define PWM_MIN_OUTPUT  -255.0f

// Open-loop scaling — maps target velocity (m/s) directly to PWM counts.
// No encoders, no PID — this is the only "tuning knob" for how the robot moves.
// Tune it live: drive full joystick stick, if the robot feels sluggish raise this,
// if it lurches too hard lower it.
#define OPEN_LOOP_PWM_PER_MS   250.0f   // PWM counts per 1.0 m/s of target