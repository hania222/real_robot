#pragma once

// ── Rotation axis: NEMA23 + TB6600 ───────────────────────────────────────────
#define ROT_STEP_PIN        18
#define ROT_DIR_PIN         19
// No ROT_ENA_PIN — ENA hardwired to GND, always enabled

// ── Extension axis: 2x NEMA17, both TB6600s share same STEP and DIR ──────────
// Wire both PUL+ together to GPIO 22, both DIR+ together to GPIO 23
// Both ENA+ and ENA− tied to GND on each driver
#define EXT_STEP_PIN        22
#define EXT_DIR_PIN         23
// No EXT_ENA pins — ENA hardwired to GND, always enabled

// ── SG90 servos ──────────────────────────────────────────────────────────────
#define SERVO_LEFT_PIN      27
#define SERVO_RIGHT_PIN     14

// ── Limit switches (INPUT_PULLUP: LOW = triggered) ───────────────────────────
// ARM_HOME: arms fully retracted (start position)
// ARM_END:  arms fully extended (safety cutoff, like top switch on lift)
// ROT_HOME: rotation centered (home position)
#define ARM_HOME_SWITCH_PIN  32
#define ARM_END_SWITCH_PIN   13    // GPIO 13 — supports INPUT_PULLUP, unlike GPIO 35
#define ROT_HOME_SWITCH_PIN  33

// ── Motor and driver constants ────────────────────────────────────────────────
// All 3 TB6600s set to 8 microsteps on DIP switches
// NEMA23 (rotation): set driver current to 2.5A
// NEMA17 (extension x2): set each driver current to 1.7A
// All motors: 200 full steps x 8 microsteps = 1600 steps/rev
#define STEPS_PER_REV       1600

// ── Rotation constants ────────────────────────────────────────────────────────
// Direct drive — 1600 steps = 360 degrees, 400 steps = 90 degrees
#define ROT_MAX_STEPS       550    // safety limit
#define ROT_HOMING_SPEED    200    // steps/sec — slow for safe homing
#define ROT_SPEED           800    // steps/sec — normal operation

// ── Extension constants ───────────────────────────────────────────────────────
// GT2 belt, 26-tooth pulley: 52mm travel per revolution
// 1600 steps/rev / 52mm/rev = 30.77 steps/mm
// Set ARM_TRAVEL_MM after physically measuring max safe travel
#define STEPS_PER_MM_EXT    30.77f
#define ARM_TRAVEL_MM       200
#define EXT_MAX_STEPS       ((int)(STEPS_PER_MM_EXT * ARM_TRAVEL_MM))  // 6154 steps
#define EXT_HOMING_SPEED    300    // steps/sec
#define EXT_SPEED           1500   // steps/sec

// ── TB6600 pulse timing ───────────────────────────────────────────────────────
// TB6600 minimum PUL high/low time: 1.5us — using 5us to be safe
#define STEP_PULSE_US       5
// TB6600 DIR must be stable before PUL rises: 5us minimum
#define DIR_SETUP_US        5

// ── Servo tab positions ───────────────────────────────────────────────────────
// OPEN   = tabs folded DOWN  — safe to extend under box
// CLOSED = tabs flipped UP   — box locked onto platform during retraction
// SG90: write(0) = one extreme, write(90) = center, write(180) = other extreme
// Swap OPEN and CLOSED values if your servos move the wrong direction
#define SERVO_OPEN_DEG      0
#define SERVO_CLOSED_DEG    180
#define SERVO_SETTLE_MS     600    // ms to wait after servo command before moving arm

// ── Serial ───────────────────────────────────────────────────────────────────
#define SERIAL_BAUD         115200

// ── Debug ────────────────────────────────────────────────────────────────────
// true  = status JSON printed every STATUS_REPORT_MS (use during testing)
// false = silent, only replies to commands (use in production)
#define DEBUG_VERBOSE       true
#define STATUS_REPORT_MS    200