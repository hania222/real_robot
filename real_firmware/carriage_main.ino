// Carriage subsystem — ESP32
//
// Hardware
//   1x NEMA23 stepper  — rotation axis, TB6600 driver at 2.5A, 8 microsteps
//   2x NEMA17 steppers — extension axis, one TB6600 each at 1.7A, 8 microsteps
//                        both share the same STEP and DIR signals (wired in parallel)
//   2x SG90 servos     — tab locks, hold box on platform during retraction
//   3x limit switches  — ARM_HOME (retracted), ARM_END (extended), ROT_HOME (centered)
//
// Wiring — identical logic to lifting firmware
//   All TB6600: ENA+ and ENA- both to GND (always enabled, no software control)
//               PUL- and DIR- both to GND
//               PUL+ to ESP32 step GPIO
//               DIR+ to ESP32 dir GPIO
//   Switches: one leg to GPIO, other leg to GND — INPUT_PULLUP, LOW = triggered
//
// Serial protocol — 115200 baud, newline-terminated JSON
//
//   Pi -> ESP32 commands:
//     {"cmd":"home"}
//     {"cmd":"rotate","steps":200}        move rotation to step position from home
//     {"cmd":"extend","steps":500}        extend arms N steps from home
//     {"cmd":"retract"}                   retract arms back to home
//     {"cmd":"tabs","pos":"open"}         open tabs (arms free to move)
//     {"cmd":"tabs","pos":"close"}        close tabs (lock box)
//     {"cmd":"pick","rotation":200,"extension":500}   full autonomous pick sequence
//     {"cmd":"status"}
//     {"cmd":"stop"}
//
//   ESP32 -> Pi replies:
//     {"type":"boot",         "msg":"carriage ready"}
//     {"type":"ack",          "cmd":"home"}
//     {"type":"home_complete","success":true}
//     {"type":"ack",          "cmd":"pick"}
//     {"type":"pick_complete","success":true}
//     {"type":"pick_complete","success":false,"reason":"..."}
//     {"type":"status", ...}
//     {"type":"error",  "msg":"..."}
//
// State machine flow
//   home  -> open tabs -> retract arm to ARM_HOME -> rotate to ROT_HOME -> done
//   pick  -> open tabs -> rotate to target -> extend to target -> close tabs
//         -> retract -> rotate back to 0 -> done

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "carriage_config.h"

enum State
{
    S_IDLE,

    S_HOME_OPEN_TABS,
    S_HOME_WAIT_TABS,
    S_HOME_RETRACT_ARM,
    S_HOME_ROTATE,
    S_HOME_DONE,

    S_PICK_OPEN_TABS,
    S_PICK_WAIT_TABS_OPEN,
    S_PICK_ROTATE,
    S_PICK_EXTEND,
    S_PICK_CLOSE_TABS,
    S_PICK_WAIT_TABS_CLOSE,
    S_PICK_RETRACT,
    S_PICK_RETURN_ROTATE,
    S_PICK_DONE,

    S_ERROR
};

static const char* state_label(State s)
{
    if (s == S_IDLE)  return "IDLE";
    if (s == S_ERROR) return "ERROR";
    if (s == S_HOME_OPEN_TABS || s == S_HOME_WAIT_TABS ||
        s == S_HOME_RETRACT_ARM || s == S_HOME_ROTATE || s == S_HOME_DONE)
        return "HOMING";
    return "PICKING";
}

static State         current_state    = S_IDLE;
static bool          is_homed         = false;
static long          rot_pos          = 0;   // steps from home (0 = centered)
static long          ext_pos          = 0;   // steps from home (0 = retracted)
static long          target_rot       = 0;
static long          target_ext       = 0;
static String        grip_state       = "open";
static unsigned long servo_timer      = 0;
static unsigned long status_timer     = 0;
static unsigned long last_rot_step_us = 0;
static unsigned long last_ext_step_us = 0;

static Servo servo_left;
static Servo servo_right;

// ── Servo helpers ─────────────────────────────────────────────────────────────
static void open_tabs()
{
    servo_left.write(SERVO_OPEN_DEG);
    servo_right.write(SERVO_OPEN_DEG);
    grip_state  = "open";
    servo_timer = millis();
}

static void close_tabs()
{
    servo_left.write(SERVO_CLOSED_DEG);
    servo_right.write(SERVO_CLOSED_DEG);
    grip_state  = "close";
    servo_timer = millis();
}

// Returns true once SERVO_SETTLE_MS has elapsed since the last servo command
static bool tabs_settled()
{
    return (millis() - servo_timer >= SERVO_SETTLE_MS);
}

// ── Stepper helpers ───────────────────────────────────────────────────────────
// Non-blocking single step toward target position.
// Returns true when pos == target.
// No enable/disable — drivers are always on (ENA hardwired to GND).
static bool step_toward(uint8_t step_pin, uint8_t dir_pin,
                         long target, long &pos, long pos_min, long pos_max,
                         unsigned long speed, unsigned long &last_step_us)
{
    if (pos == target) return true;

    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;
    if (now_us - last_step_us < interval) return false;
    last_step_us = now_us;

    bool going_positive = (target > pos);
    digitalWrite(dir_pin, going_positive ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    digitalWrite(step_pin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(STEP_PULSE_US);

    pos += going_positive ? 1 : -1;
    pos  = constrain(pos, pos_min, pos_max);

    return (pos == target);
}

// Non-blocking step toward a limit switch (used during homing).
// Returns true when switch_pin reads LOW (triggered).
static bool step_to_switch(uint8_t step_pin, uint8_t dir_pin,
                             uint8_t switch_pin, bool positive_dir,
                             unsigned long speed, unsigned long &last_step_us)
{
    if (digitalRead(switch_pin) == LOW) return true;

    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;
    if (now_us - last_step_us < interval) return false;
    last_step_us = now_us;

    digitalWrite(dir_pin, positive_dir ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    digitalWrite(step_pin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(STEP_PULSE_US);

    return (digitalRead(switch_pin) == LOW);
}

// Blocking backoff — called once right after a switch triggers.
// Small step count so the brief block is acceptable.
static void backoff_from_switch(uint8_t step_pin, uint8_t dir_pin,
                                 bool positive_dir, int steps, unsigned long speed)
{
    digitalWrite(dir_pin, positive_dir ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    unsigned long interval = 1000000UL / speed;
    for (int i = 0; i < steps; i++)
    {
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(step_pin, LOW);
        delayMicroseconds(STEP_PULSE_US);
        delayMicroseconds(interval);
    }
}

// ── JSON helpers ──────────────────────────────────────────────────────────────
static void send_json(JsonDocument &doc)
{
    serializeJson(doc, Serial);
    Serial.println();
}

static void send_status()
{
    StaticJsonDocument<256> doc;
    doc["type"]    = "status";
    doc["state"]   = state_label(current_state);
    doc["homed"]   = is_homed;
    doc["rot_pos"] = rot_pos;
    doc["ext_pos"] = ext_pos;
    doc["grip"]    = grip_state;
    doc["arm_sw"]  = (digitalRead(ARM_HOME_SWITCH_PIN) == LOW);
    doc["arm_end"] = (digitalRead(ARM_END_SWITCH_PIN)  == LOW);
    doc["rot_sw"]  = (digitalRead(ROT_HOME_SWITCH_PIN) == LOW);
    doc["busy"]    = (current_state != S_IDLE && current_state != S_ERROR);
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

// ── Command handler ───────────────────────────────────────────────────────────
static void handle_command(const String &line)
{
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok)
    {
        send_error("JSON parse error");
        return;
    }

    const char *cmd = doc["cmd"] | "";

    // always accepted regardless of state
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

    // manual single-axis commands for testing — only allowed when IDLE
    if (current_state != S_IDLE && current_state != S_ERROR)
    {
        send_error("busy — wait for sequence to complete");
        return;
    }

    if (strcmp(cmd, "tabs") == 0)
    {
        const char *pos = doc["pos"] | "open";
        if (strcmp(pos, "open") == 0)
            open_tabs();
        else
            close_tabs();
        send_ack("tabs");
        return;
    }

    if (strcmp(cmd, "home") == 0)
    {
        is_homed      = false;
        current_state = S_HOME_OPEN_TABS;
        send_ack("home");
        return;
    }

    if (strcmp(cmd, "pick") == 0)
    {
        if (!is_homed)
        {
            send_error("not homed — send home first");
            return;
        }

        target_rot = doc["rotation"]  | 0L;
        target_ext = doc["extension"] | 0L;

        if (target_rot >  ROT_MAX_STEPS) target_rot =  ROT_MAX_STEPS;
        if (target_rot < -ROT_MAX_STEPS) target_rot = -ROT_MAX_STEPS;
        if (target_ext < 0)              target_ext = 0;
        if (target_ext > EXT_MAX_STEPS)  target_ext = EXT_MAX_STEPS;

        current_state = S_PICK_OPEN_TABS;
        send_ack("pick");
        return;
    }

    send_error("unknown command");
}

// ── State machine ─────────────────────────────────────────────────────────────
static void run_state_machine()
{
    switch (current_state)
    {
        case S_IDLE:
        case S_ERROR:
            break;

        // ── HOMING SEQUENCE ──────────────────────────────────────────────────
        case S_HOME_OPEN_TABS:
            open_tabs();
            current_state = S_HOME_WAIT_TABS;
            break;

        case S_HOME_WAIT_TABS:
            if (tabs_settled()) current_state = S_HOME_RETRACT_ARM;
            break;

        case S_HOME_RETRACT_ARM:
            // Drive arm toward ARM_HOME switch in the negative (retract) direction
            if (step_to_switch(EXT_STEP_PIN, EXT_DIR_PIN,
                                ARM_HOME_SWITCH_PIN, false,
                                EXT_HOMING_SPEED, last_ext_step_us))
            {
                backoff_from_switch(EXT_STEP_PIN, EXT_DIR_PIN, true, 40, EXT_HOMING_SPEED / 2);
                ext_pos       = 0;
                current_state = S_HOME_ROTATE;
            }
            break;

        case S_HOME_ROTATE:
            // Drive rotation toward ROT_HOME switch in the negative direction
            if (step_to_switch(ROT_STEP_PIN, ROT_DIR_PIN,
                                ROT_HOME_SWITCH_PIN, false,
                                ROT_HOMING_SPEED, last_rot_step_us))
            {
                backoff_from_switch(ROT_STEP_PIN, ROT_DIR_PIN, true, 30, ROT_HOMING_SPEED / 2);
                rot_pos       = 0;
                current_state = S_HOME_DONE;
            }
            break;

        case S_HOME_DONE:
            is_homed      = true;
            current_state = S_IDLE;
            send_home_result(true);
            break;

        // ── PICKING SEQUENCE ─────────────────────────────────────────────────
        case S_PICK_OPEN_TABS:
            open_tabs();
            current_state = S_PICK_WAIT_TABS_OPEN;
            break;

        case S_PICK_WAIT_TABS_OPEN:
            if (tabs_settled()) current_state = S_PICK_ROTATE;
            break;

        case S_PICK_ROTATE:
            if (step_toward(ROT_STEP_PIN, ROT_DIR_PIN,
                             target_rot, rot_pos, -ROT_MAX_STEPS, ROT_MAX_STEPS,
                             ROT_SPEED, last_rot_step_us))
            {
                current_state = S_PICK_EXTEND;
            }
            break;

        case S_PICK_EXTEND:
            // Abort if end switch triggers — prevents mechanical damage
            if (digitalRead(ARM_END_SWITCH_PIN) == LOW)
            {
                ext_pos = EXT_MAX_STEPS;
                send_pick_result(false, "arm end switch triggered during extension");
                current_state = S_ERROR;
                break;
            }
            if (step_toward(EXT_STEP_PIN, EXT_DIR_PIN,
                             target_ext, ext_pos, 0, EXT_MAX_STEPS,
                             EXT_SPEED, last_ext_step_us))
            {
                current_state = S_PICK_CLOSE_TABS;
            }
            break;

        case S_PICK_CLOSE_TABS:
            close_tabs();
            current_state = S_PICK_WAIT_TABS_CLOSE;
            break;

        case S_PICK_WAIT_TABS_CLOSE:
            // If box slips during retraction, increase SERVO_SETTLE_MS
            if (tabs_settled()) current_state = S_PICK_RETRACT;
            break;

        case S_PICK_RETRACT:
            if (step_toward(EXT_STEP_PIN, EXT_DIR_PIN,
                             0, ext_pos, 0, EXT_MAX_STEPS,
                             EXT_SPEED, last_ext_step_us))
            {
                current_state = S_PICK_RETURN_ROTATE;
            }
            break;

        case S_PICK_RETURN_ROTATE:
            if (step_toward(ROT_STEP_PIN, ROT_DIR_PIN,
                             0, rot_pos, -ROT_MAX_STEPS, ROT_MAX_STEPS,
                             ROT_SPEED, last_rot_step_us))
            {
                current_state = S_PICK_DONE;
            }
            break;

        case S_PICK_DONE:
            current_state = S_IDLE;
            send_pick_result(true);
            break;
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(SERIAL_BAUD);

    // Stepper pins — ENA is hardwired to GND, no ENA pin needed in software
    pinMode(ROT_STEP_PIN, OUTPUT);
    pinMode(ROT_DIR_PIN,  OUTPUT);
    digitalWrite(ROT_STEP_PIN, LOW);
    digitalWrite(ROT_DIR_PIN,  LOW);

    pinMode(EXT_STEP_PIN, OUTPUT);
    pinMode(EXT_DIR_PIN,  OUTPUT);
    digitalWrite(EXT_STEP_PIN, LOW);
    digitalWrite(EXT_DIR_PIN,  LOW);

    // Limit switches
    pinMode(ARM_HOME_SWITCH_PIN, INPUT_PULLUP);
    pinMode(ARM_END_SWITCH_PIN,  INPUT_PULLUP);
    pinMode(ROT_HOME_SWITCH_PIN, INPUT_PULLUP);

    // Servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servo_left.setPeriodHertz(50);
    servo_right.setPeriodHertz(50);
    servo_left.attach(SERVO_LEFT_PIN,   1000, 2000);
    servo_right.attach(SERVO_RIGHT_PIN, 1000, 2000);

    open_tabs();
    delay(700);

    StaticJsonDocument<64> boot;
    boot["type"] = "boot";
    boot["msg"]  = "carriage ready";
    send_json(boot);
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop()
{
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) handle_command(line);
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