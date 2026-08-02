// carriage_main_Final.ino
//
// ═══════════════════════════════════════════════════════════════════════════════
// HARDWARE OVERVIEW
// ═══════════════════════════════════════════════════════════════════════════════
//
//  ① 2× SG90 servo (gripper fingers — mirrored)
//       Left  servo: 0°  = open,  180° = closed   → pin 10
//       Right servo: 180°= open,    0° = closed    → pin 11
//       Both move together via open_gripper() / close_gripper()
//
//  ② 2× NEMA 17 extension-arm steppers (shared step pin, mirrored dirs)
//       Shared STEP pin  → pin 3
//       Left  DIR pin    → pin 2
//       Right DIR pin    → pin 4   (opposite logic to left — mirrored)
//       Home limit switch (arms retracted)  → pin 8   (INPUT_PULLUP, LOW = hit)
//       End-of-travel switch (arms extended) → pin 9   (INPUT_PULLUP, LOW = hit)
//       Driver: same TB6600 / A4988 at 8 microsteps → 1600 steps/rev
//
//  ③ 1× NEMA 23 rotation stepper (turntable)
//       STEP pin → pin 5
//       DIR  pin → pin 6
//       Home limit switch (0° position) → pin 7   (INPUT_PULLUP, LOW = hit)
//       Pick limit switch (90° pick position) → pin 8 (INPUT_PULLUP, LOW = hit)
//       Driver: TB6600 at 8 microsteps → 1600 steps/rev
//       Pick angle: 90° from home, detected by ROT_PICK_SW, not by step count
//
// ═══════════════════════════════════════════════════════════════════════════════
// HOMING SEQUENCE  (triggered by {"cmd":"home"})
// ═══════════════════════════════════════════════════════════════════════════════
//   Step 1  Open both servos  (move to open angle)
//   Step 2  Retract extension arms until HOME limit switch clicks
//   Step 3  Rotate NEMA 23 until ROTATION home switch clicks → zero set
//   → sends {"type":"home_complete","success":true}
//
// ═══════════════════════════════════════════════════════════════════════════════
// PICK SEQUENCE  (triggered by {"cmd":"pick"})
// ═══════════════════════════════════════════════════════════════════════════════
//   Step 1  Rotate NEMA 23 until ROT_PICK_SW clicks (90° pick position,
//           detected by the limit switch, not by counting steps —
//           ROT_PICK_STEPS is kept only as a safety timeout ceiling)
//   Step 2  Extend arms EXTEND_STEPS steps (or until end-of-travel switch)
//   Step 3  Close servos
//   Step 4  Retract arms back until ARM_HOME_SW clicks
//   Step 5  Rotate NEMA 23 back until ROT_HOME_SW clicks (return to zero)
//   → sends {"type":"pick_complete","success":true}
//
// ═══════════════════════════════════════════════════════════════════════════════
// SERIAL PROTOCOL — 115200 baud, newline-terminated JSON
// ═══════════════════════════════════════════════════════════════════════════════
//   Pi → Arduino:
//     {"cmd":"home"}
//     {"cmd":"pick"}
//     {"cmd":"status"}
//     {"cmd":"stop"}
//
//   Arduino → Pi:
//     {"type":"boot",          "msg":"carriage ready"}
//     {"type":"ack",           "cmd":"home"}
//     {"type":"home_complete", "success":true}
//     {"type":"ack",           "cmd":"pick"}
//     {"type":"pick_complete", "success":true}
//     {"type":"pick_complete", "success":false,"reason":"..."}
//     {"type":"status",        "state":"IDLE","homed":true,...}
//     {"type":"error",         "msg":"..."}

#include <ArduinoJson.h>
#include <Servo.h>

// ─── Pin definitions ───────────────────────────────────────────────────────────
// Gripper servos
#define SERVO_LEFT_PIN        9
#define SERVO_RIGHT_PIN       10

// Extension arm steppers (shared STEP, mirrored DIR)
#define ARM_STEP_PIN          3
#define ARM_DIR_LEFT_PIN      2
#define ARM_DIR_RIGHT_PIN     4
#define ARM_HOME_SW           6   // retracted / home position
#define ARM_END_SW            7


// Rotation NEMA 23
#define ROT_STEP_PIN          12
#define ROT_DIR_PIN           11
#define ROT_HOME_SW           13   // 0° reference position
#define ROT_PICK_SW           8    // 90° pick position (LOW = hit)

// ─── Servo angles ──────────────────────────────────────────────────────────────
// Left  servo: 0° = open,  180° = closed
// Right servo: 180° = open,  0° = closed  (mirrored)
#define SERVO_LEFT_OPEN       0
#define SERVO_LEFT_CLOSED     90
#define SERVO_RIGHT_OPEN      180
#define SERVO_RIGHT_CLOSED    90

// ─── Stepper constants ─────────────────────────────────────────────────────────
#define FULL_STEPS_PER_REV    200
#define MICROSTEPS            4
#define STEPS_PER_REV         (FULL_STEPS_PER_REV * MICROSTEPS)   // 1600

// Extension arms — configurable travel distance
// Tune EXTEND_STEPS to match your physical arm travel
#define EXTEND_STEPS          3200UL    // 2 full revolutions — adjust as needed

// Rotation NEMA 23 — 90° pick angle
// The pick position is now detected by ROT_PICK_SW (limit switch), not by
// counting steps. ROT_PICK_STEPS is retained only as a safety ceiling in case
// the switch is never reached (e.g. wiring fault) so the motor does not spin
// forever — it should be set comfortably higher than the real step count for
// 90° so it never trips before the switch under normal operation.
#define ROT_PICK_STEPS         3000UL    // safety timeout ceiling — adjust as needed

// ─── Speed constants ───────────────────────────────────────────────────────────
// Arms homing: slow retract so switch is not hit hard
#define ARM_HOME_SPEED        450UL    // steps/sec  (~0.375 rev/sec)

// Arms extend: normal working speed
#define ARM_SPEED             1200UL   // steps/sec  //was 1200

// Rotation homing: slow so home switch is caught cleanly
#define ROT_HOME_SPEED        600UL    // steps/sec

// Rotation pick move: normal working speed
#define ROT_SPEED             1200UL   // steps/sec

// ─── Pulse timing ──────────────────────────────────────────────────────────────
#define STEP_PULSE_US         5        // HIGH and LOW pulse width (µs)
#define DIR_SETUP_US          5        // DIR must be stable before STEP rises

// ─── Debug ─────────────────────────────────────────────────────────────────────
#define DEBUG_VERBOSE         false
#define STATUS_REPORT_MS      500
#define SERIAL_BAUD           115200

// ─── State machine ─────────────────────────────────────────────────────────────
enum State
{
    S_IDLE,

    // — Homing substates —
    S_HOME_SERVO_OPEN,       // open both servos
    S_HOME_ARM_RETRACT,      // retract arms until ARM_HOME_SW clicks
    S_HOME_ROT_FIND,         // spin rotation until ROT_HOME_SW clicks
    S_HOME_DONE,             // set homed flag, send result

    // — Pick substates —
    S_PICK_ROT_MOVE,         // rotate until ROT_PICK_SW clicks (90° pick position)
    S_PICK_ARM_EXTEND,       // extend arms EXTEND_STEPS steps
    S_PICK_SERVO_CLOSE,      // close gripper
    S_PICK_ARM_RETRACT,      // retract arms back until ARM_HOME_SW clicks
    S_PICK_ROT_RETURN,       // rotate back until ROT_HOME_SW clicks (return to zero)
    S_PICK_DONE,             // send pick_complete

    S_ERROR                  // fault — stop+home required to recover
};

static State         current_state   = S_IDLE;
static bool          is_homed        = false;

// Extension arm step counter (used during extend move)
static unsigned long arm_steps_done  = 0;

// Rotation step counter (used during pick rotation — safety ceiling only)
static unsigned long rot_steps_done  = 0;

// Timestamps for non-blocking step pulses
static unsigned long last_arm_step_us = 0;
static unsigned long last_rot_step_us = 0;

// Debug timer
static unsigned long status_timer     = 0;

// Servo objects
static Servo servo_left;
static Servo servo_right;

// ─── JSON helpers ──────────────────────────────────────────────────────────────
static void send_json(JsonDocument &doc)
{
    serializeJson(doc, Serial);
    Serial.println();
}

static void send_status()
{
    StaticJsonDocument<256> doc;
    doc["type"]        = "status";
    doc["state"]       = (current_state == S_IDLE)  ? "IDLE"
                       : (current_state == S_ERROR) ? "ERROR"
                       :                              "BUSY";
    doc["homed"]       = is_homed;
    doc["arm_home_sw"] = (digitalRead(ARM_HOME_SW) == LOW);
    doc["arm_end_sw"]  = (digitalRead(ARM_END_SW)  == LOW);
    doc["rot_home_sw"] = (digitalRead(ROT_HOME_SW) == LOW);
    doc["rot_pick_sw"] = (digitalRead(ROT_PICK_SW) == LOW);
    doc["busy"]        = (current_state != S_IDLE && current_state != S_ERROR);
    send_json(doc);
}

static void send_ack(const char *cmd)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["cmd"]  = cmd;
    send_json(doc);
}

static void send_error(const char *msg)
{
    StaticJsonDocument<128> doc;
    doc["type"] = "error";
    doc["msg"]  = msg;
    send_json(doc);
    current_state = S_ERROR;
}

static void send_home_result(bool success, const char *reason = "")
{
    StaticJsonDocument<128> doc;
    doc["type"]    = "home_complete";
    doc["success"] = success;
    if (!success) doc["reason"] = reason;
    send_json(doc);
}

static void send_pick_result(bool success, const char *reason = "")
{
    StaticJsonDocument<128> doc;
    doc["type"]    = "pick_complete";
    doc["success"] = success;
    if (!success) doc["reason"] = reason;
    send_json(doc);
}

// ─── Servo helpers ─────────────────────────────────────────────────────────────
static void open_gripper()
{
    servo_left.write(SERVO_LEFT_OPEN);
    servo_right.write(SERVO_RIGHT_OPEN);
}

static void close_gripper()
{
    servo_left.write(SERVO_LEFT_CLOSED);
    servo_right.write(SERVO_RIGHT_CLOSED);
}

// ─── Extension arm stepper helpers ────────────────────────────────────────────
// Set direction: extend = arms go out, retract = arms come back home
static void arm_set_dir(bool extend)
{
    // Left  motor: HIGH = extend, LOW = retract
    // Right motor: mirrored — LOW = extend, HIGH = retract
    digitalWrite(ARM_DIR_LEFT_PIN,  extend ? LOW : HIGH);
    digitalWrite(ARM_DIR_RIGHT_PIN, extend ? HIGH  : LOW);
    delayMicroseconds(DIR_SETUP_US);
}

// Pulse both arm motors together (shared STEP pin)
static void arm_pulse()
{
    digitalWrite(ARM_STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(ARM_STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
}

// Non-blocking: retract one step toward ARM_HOME_SW.
// Returns true when switch clicks (home reached).
static bool arm_step_to_home(unsigned long speed)
{
    if (digitalRead(ARM_HOME_SW) == LOW) return true;   // already home

    unsigned long now      = micros();
    unsigned long interval = 1000000UL / speed;
    if (now - last_arm_step_us < interval) return false;
    last_arm_step_us = now;

    arm_set_dir(false);   // retract
    arm_pulse();

    return (digitalRead(ARM_HOME_SW) == LOW);
}

// Non-blocking: extend one step. Watches end-of-travel switch.
// Returns true when target_steps reached.
// If end switch fires early, reports JSON and returns true (sequence continues).
static bool arm_step_extend(unsigned long target_steps, unsigned long speed)
{
    // End-of-travel switch hit — stop and report
    if (digitalRead(ARM_END_SW) == LOW)
    {
        StaticJsonDocument<128> doc;
        doc["type"]  = "arm_end_sw";
        doc["msg"]   = "extension end-of-travel switch triggered — arms stopped";
        send_json(doc);
        return true;   // treat as done, sequence continues
    }

    if (arm_steps_done >= target_steps) return true;

    unsigned long now      = micros();
    unsigned long interval = 1000000UL / speed;
    if (now - last_arm_step_us < interval) return false;
    last_arm_step_us = now;

    arm_set_dir(true);   // extend
    arm_pulse();
    arm_steps_done++;

    // Re-check switch immediately after the step
    if (digitalRead(ARM_END_SW) == LOW)
    {
        StaticJsonDocument<128> doc;
        doc["type"]  = "arm_end_sw";
        doc["msg"]   = "extension end-of-travel switch triggered — arms stopped";
        send_json(doc);
        return true;
    }

    return (arm_steps_done >= target_steps);
}

// ─── Rotation stepper helpers ──────────────────────────────────────────────────
// Set rotation direction
static void rot_set_dir(bool clockwise)
{
    digitalWrite(ROT_DIR_PIN, clockwise ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);
}

// Pulse the rotation motor
static void rot_pulse()
{
    digitalWrite(ROT_STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(ROT_STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
}

// Non-blocking: spin rotation motor until ROT_HOME_SW clicks.
// Returns true when switch is hit.
static bool rot_step_to_home(unsigned long speed)
{
    if (digitalRead(ROT_HOME_SW) == LOW) return true;

    unsigned long now      = micros();
    unsigned long interval = 1000000UL / speed;
    if (now - last_rot_step_us < interval) return false;
    last_rot_step_us = now;

    rot_set_dir(false);   // counter-clockwise toward home — flip if needed
    rot_pulse();

    return (digitalRead(ROT_HOME_SW) == LOW);
}

// Non-blocking: rotate toward the pick position. The 90° pick position is
// detected by ROT_PICK_SW (limit switch) rather than by counting steps.
// target_steps is kept only as a safety timeout ceiling in case the switch (360 degree)
// is never reached, so the motor cannot spin indefinitely.
// Returns true when the pick switch is hit (or the safety ceiling is hit).
static bool rot_step_pick(unsigned long target_steps, unsigned long speed)
{
    if (digitalRead(ROT_PICK_SW) == LOW) return true;   // pick position reached

   // if (rot_steps_done >= target_steps) return true;   // safetyafety ceiling reached

    unsigned long now      = micros();
    unsigned long interval = 1000000UL / speed;
    if (now - last_rot_step_us < interval) return false;
    last_rot_step_us = now;

    rot_set_dir(true);   // clockwise toward pick position — flip if needed
    rot_pulse();
    rot_steps_done++;

    // Re-check switch immediately after the step
    if (digitalRead(ROT_PICK_SW) == LOW) return true;

    return (rot_steps_done >= target_steps);
}

// ─── Command handler ───────────────────────────────────────────────────────────
static void handle_command(const String &line)
{
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok)
    {
        send_error("JSON parse error");
        return;
    }

    const char *cmd = doc["cmd"] | "";

    // status and stop always accepted
    if (strcmp(cmd, "status") == 0)
    {
        send_status();
        return;
    }

    if (strcmp(cmd, "stop") == 0)
    {
        current_state = S_IDLE;
        is_homed      = false;
        send_ack("stop");
        return;
    }

    // all other commands require IDLE or ERROR state
    if (current_state != S_IDLE && current_state != S_ERROR)
    {
        send_error("busy — wait for current sequence to complete");
        return;
    }

    if (strcmp(cmd, "home") == 0)
    {
        is_homed      = false;
        current_state = S_HOME_SERVO_OPEN;
        send_ack("home");
        return;
    }

    if (strcmp(cmd, "pick") == 0)
    {
        if (!is_homed)
        {
            send_error("not homed — send home command first");
            return;
        }
        rot_steps_done = 0;
        arm_steps_done = 0;
        current_state  = S_PICK_ROT_MOVE;
        send_ack("pick");
        return;
    }

    send_error("unknown command");
}

// ─── State machine ─────────────────────────────────────────────────────────────
static void run_state_machine()
{
    switch (current_state)
    {
        // ── Quiescent ────────────────────────────────────────────────────────
        case S_IDLE:
        case S_ERROR:
            break;

        // ── Homing: step 1 — open servos ─────────────────────────────────────
        case S_HOME_SERVO_OPEN:
            open_gripper();
            // Servos are slew-rate limited internally; give them time to move
            delay(500);
            arm_steps_done   = 0;
            last_arm_step_us = micros();
            current_state    = S_HOME_ARM_RETRACT;
            break;

        // ── Homing: step 2 — retract arms until ARM_HOME_SW ──────────────────
        case S_HOME_ARM_RETRACT:
            if (arm_step_to_home(ARM_HOME_SPEED))
            {
                last_rot_step_us = micros();
                current_state    = S_HOME_ROT_FIND;
            }
            break;

        // ── Homing: step 3 — spin rotation until ROT_HOME_SW ─────────────────
        case S_HOME_ROT_FIND:
            if (rot_step_to_home(ROT_HOME_SPEED))
            {
                current_state = S_HOME_DONE;
            }
            break;

        // ── Homing: complete ──────────────────────────────────────────────────
        case S_HOME_DONE:
            is_homed      = true;
            current_state = S_IDLE;
            send_home_result(true);
            break;

        // ── Pick: step 1 — rotate to pick angle (stopped by ROT_PICK_SW) ─────
        case S_PICK_ROT_MOVE:
            if (rot_step_pick(ROT_PICK_STEPS, ROT_SPEED))
            {
                arm_steps_done   = 0;
                last_arm_step_us = micros();
                current_state    = S_PICK_ARM_EXTEND;
            }
            break;

        // ── Pick: step 2 — extend arms ────────────────────────────────────────
        case S_PICK_ARM_EXTEND:
            if (arm_step_extend(EXTEND_STEPS, ARM_SPEED))
            {
                current_state = S_PICK_SERVO_CLOSE;
            }
            break;

        // ── Pick: step 3 — close gripper ─────────────────────────────────────
        case S_PICK_SERVO_CLOSE:
            close_gripper();
            delay(500);   // allow SG90 to reach closed position
            last_arm_step_us = micros();
            current_state    = S_PICK_ARM_RETRACT;
            break;

        // ── Pick: step 4 — retract arms back until ARM_HOME_SW ───────────────
        case S_PICK_ARM_RETRACT:
            if (arm_step_to_home(ARM_HOME_SPEED))
            {
                last_rot_step_us = micros();
                current_state    = S_PICK_ROT_RETURN;
            }
            break;

        // ── Pick: step 5 — rotate back to zero until ROT_HOME_SW ─────────────
        case S_PICK_ROT_RETURN:
            if (rot_step_to_home(ROT_HOME_SPEED))
            {
                current_state = S_PICK_DONE;
            }
            break;

        // ── Pick: complete ────────────────────────────────────────────────────
        case S_PICK_DONE:
            current_state = S_IDLE;
            send_pick_result(true);
            break;
    }
}

// ─── setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(SERIAL_BAUD);

    // Servo attach
    servo_left.attach(SERVO_LEFT_PIN);
    servo_right.attach(SERVO_RIGHT_PIN);
    open_gripper();   // start open

    // Extension arm stepper pins
    pinMode(ARM_STEP_PIN,       OUTPUT);
    pinMode(ARM_DIR_LEFT_PIN,   OUTPUT);
    pinMode(ARM_DIR_RIGHT_PIN,  OUTPUT);
    digitalWrite(ARM_STEP_PIN,      LOW);
    digitalWrite(ARM_DIR_LEFT_PIN,  LOW);
    digitalWrite(ARM_DIR_RIGHT_PIN, HIGH);

    // Rotation stepper pins
    pinMode(ROT_STEP_PIN, OUTPUT);
    pinMode(ROT_DIR_PIN,  OUTPUT);
    digitalWrite(ROT_STEP_PIN, LOW);
    digitalWrite(ROT_DIR_PIN,  LOW);

    // Limit switches
    pinMode(ARM_HOME_SW, INPUT_PULLUP);
    pinMode(ARM_END_SW,  INPUT_PULLUP);
    pinMode(ROT_HOME_SW, INPUT_PULLUP);
    pinMode(ROT_PICK_SW, INPUT_PULLUP);

    StaticJsonDocument<64> boot;
    boot["type"] = "boot";
    boot["msg"]  = "carriage ready";
    send_json(boot);
}

// ─── loop ─────────────────────────────────────────────────────────────────────
void loop()
{
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
            handle_command(line);
    }

    run_state_machine();

    if (DEBUG_VERBOSE)
    {
        unsigned long now = millis();
        if (now - status_timer >= STATUS_REPORT_MS)
        {
            status_timer = now;
            send_status();
        }
    }
}