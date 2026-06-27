#pragma once

// Robot geometry
#define WHEEL_RADIUS      0.065f        // 130 mm diameter -> 65 mm radius
#define WHEEL_BASE        0.642f        // metres — must match URDF joint origins
#define GEAR_RATIO        99.0f
#define ENCODER_PPR       600.0f        // pulses per motor shaft revolution
#define COUNTS_PER_REV    (ENCODER_PPR * 4.0f * GEAR_RATIO)  // 600 × 4 × 99.5 = 238 800

// BTS7960 motor driver pins
// RPWM = forward PWM, LPWM = reverse PWM, R_EN + L_EN = enable HIGH
#define L_RPWM  25
#define L_LPWM  26
#define L_R_EN  27
#define L_L_EN  14

#define R_RPWM  32
#define R_LPWM  33
#define R_R_EN  15
#define R_L_EN   4

// Encoder pins — X4 mode: both A and B must be interrupt-capable
#define LEFT_ENC_A   18
#define LEFT_ENC_B   16
#define RIGHT_ENC_A  19
#define RIGHT_ENC_B  17

// IMU I2C pins
#define IMU_SDA  21
#define IMU_SCL  22

// PID gains
// Tuning procedure:
//   1. Ki=Kd=0 — raise Kp until wheels track without oscillating
//   2. Add Ki slowly (1 → 2 → 5) until steady-state offset disappears
//   3. Add Kd only if velocity still oscillates with good Kp+Ki
#define KP_LEFT    80.0f
#define KI_LEFT     1.0f
#define KD_LEFT     0.0f
#define KP_RIGHT   80.0f
#define KI_RIGHT    1.0f
#define KD_RIGHT    0.0f

#define PID_MAX_OUTPUT   255.0f
#define PID_MIN_OUTPUT  -255.0f

// Command timeout — stop motors if no serial command arrives within this window
#define CMD_TIMEOUT_MS   10000

// Stall detection
#define STALL_PWM_THRESHOLD    220.0f   // PWM counts — above this = "near full power"
#define STALL_SPEED_THRESHOLD    0.02f  // m/s        — below this = "wheel stopped"
#define STALL_TIME_MS            800    // ms of stall before motor is cut

// Slew-rate limiter — limits how fast the target velocity can change per PID tick
// Lower -> smoother start, slower response
// Higher -> faster response, more current spike on startup
#define MAX_ACCEL_PER_TICK  0.01f   // m/s per tick (= 0.5 m/s² at 50 Hz)

// PWM dead-band — PWM values below this are forced to zero
// Raise to 20 if wheels twitch at zero command
// Lower to 10 if robot won't start moving on slow Nav2 commands
#define PWM_DEADBAND     15

// Minimum wheel speed on the command side
// Must match STALL_SPEED_THRESHOLD so both checks agree on "wheel stopped"
#define MIN_WHEEL_SPEED  0.02f   // m/s
// Encoder spike filter — discard reads above this speed as corrupted (max motor speed= 60RPM= 0.408 m/s)
// impossible value so genuine noise spikes are caught before reaching the PID
#define MAX_BELIEVABLE_SPEED  0.6f   // m/s