#pragma once

// Rotation axis: NEMA23 + DM542 — direct drive 
#define ROT_STEP_PIN        18
#define ROT_DIR_PIN         19
#define ROT_ENA_PIN         21

//Extension axis: 2x NEMA17, each on its own DM542 driver 
// Both drivers receive identical STEP and DIR signals from the same ESP32 pins (wired in parallel on signal side)
// Each driver powers one motor independently at 1.7A 
#define EXT_STEP_PIN        22    // connected to BOTH DM542 PUL-
#define EXT_DIR_PIN         23    // connected to BOTH DM542 DIR-
#define EXT1_ENA_PIN        25    // DM542 driver 1 ENA- (motor 1)
#define EXT2_ENA_PIN        26    // DM542 driver 2 ENA- (motor 2)

//  SG90 servo for tabs that hold box onto platform during rotation
#define SERVO_LEFT_PIN      27
#define SERVO_RIGHT_PIN     14

// Limit switches (INPUT_PULLUP: LOW=triggered)
// ARM_HOME: triggered when arms are fully retracted (start pos)
// ARM_END:  triggered when arms are fully extended (shelf end)
#define ARM_HOME_SWITCH_PIN  32
#define ARM_END_SWITCH_PIN   35
#define ROT_HOME_SWITCH_PIN  33

// we set ALL 3x DM542s to 8 microstepping for smoother motion and less noise
// NEMA23 rated 2.5A --> set rotation driver  current to 2.5A
// NEMA17 rated 1.7A -> set extension driver1 current to 1.7A
//                  -->set extension driver2 current to 1.7A
// All motors: 200 full steps x 8 microsteps = 1600 steps/rev
#define STEPS_PER_REV       1600

//Rotation — NEMA23 direct drive 
// 1600 steps = 360 degrees → 400 steps = 90 degrees (will be Calibrated ROT_90_STEPS physically after flashing:
#define ROT_90_STEPS        400
#define ROT_MAX_STEPS       450
#define ROT_HOMING_SPEED    200
#define ROT_SPEED           800

// Extension — NEMA17 + GT2 belt + 26 tooth pulley 
// GT2 pitch = 2mm, 26 teeth -> 52mm travel per revolution
// 1600 steps per revolution -> 30.77 steps per mm
//( will be MeasuredARM_TRAVEL_MM physically distance from arm fully retracted to arm at shelf edge
#define STEPS_PER_MM_EXT    30.77
#define ARM_TRAVEL_MM       200
#define EXT_MAX_STEPS       (int)(STEPS_PER_MM_EXT * ARM_TRAVEL_MM)
#define EXT_HOMING_SPEED    300
#define EXT_SPEED           1500

//DM542 pulse timing 
// DM542 minimum pulse width = 2.5 microseconds
// We'll use 5 microseconds to be safeeee
#define STEP_PULSE_US       5

//  DM542 DIR setup time 
// DM542 datasheet: DIR must be stable before PUL rises
#define DIR_SETUP_US        5

// Servo tab positions 
// OPEN   = tabs folded DOWN — > safe to extend, nothing blocked
// CLOSED = tabs flipped UP  — > box locked onto platform
// SG90 range: 1000 microseconds (-90deg) to 2000 microseconds (+90deg)
// will be Tuned physically (swap values if direction is wrong)
#define SERVO_OPEN_US       1000
#define SERVO_CLOSED_US     2000
#define SERVO_SETTLE_MS     600

//Serial communication
#define SERIAL_BAUD         115200

//Debug verbose status 
// true  = publish full status every STATUS_REPORT_MS (development)
// false = silent, Pi only sees completion replies (production)
#define DEBUG_VERBOSE       true
#define STATUS_REPORT_MS    200