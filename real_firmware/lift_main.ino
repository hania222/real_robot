// lift_main.ino
//
// Hardware
//   2x NEMA23 stepper motors    — 1.8 deg/step, 2.5 A rated
//   2x TB6600 stepper drivers   — set to 2.5 A, 8 microsteps (1600 steps/rev)
//   TR8x8 leadscrew             — 8 mm lead per revolution (4 starts × 2 mm pitch)
//   2x limit switches           — bottom (home) and top (safety cutoff)
//
// Wiring
//   ENA-, ENA+  →  GND  (drivers always enabled — no software enable/disable)
//   DIR-, PUL-  →  GND
//   DIR+        →  Arduino pin 3
//   PUL+        →  Arduino pin 2
//   Bottom switch  →  Arduino pin 8 + GND  (INPUT_PULLUP, LOW = triggered)
//   Top switch     →  Arduino pin 9 + GND  (INPUT_PULLUP, LOW = triggered)
//
// NOTE: Because ENA is hardwired to GND, the motors are always energised
// whenever the TB6600 has power. This means they hold position at all times
// (good for a lift) but also draw idle current continuously. Power off the
// driver supply when the system is not in use.
//
// Math summary
//   1.8 deg/step  →  200 full steps/rev
//   8 microsteps  →  200 × 8 = 1600 steps/rev
//   TR8x8 lead    →  8 mm/rev
//   Resolution    →  1600 / 8 = 200 steps/mm
//
// Serial protocol — 115200 baud, newline-terminated JSON
//
//   Pi → Arduino commands:
//     {"cmd":"home"}
//     {"cmd":"lift","height_mm":150.0}
//     {"cmd":"status"}
//     {"cmd":"stop"}
//
//   Arduino → Pi replies:
//     {"type":"boot",         "msg":"lift ready"}
//     {"type":"ack",          "cmd":"home"}
//     {"type":"home_complete","success":true}
//     {"type":"ack",          "cmd":"lift"}
//     {"type":"lift_complete","success":true}
//     {"type":"lift_complete","success":false,"reason":"..."}
//     {"type":"status","state":"IDLE","homed":true,"lift_mm":150.0,...}
//     {"type":"error",        "msg":"..."}
//
// State machine flow
//   home command  → S_HOME_DESCEND (slow descent) → bottom switch triggers
//                → backoff 0.5 mm upward → zero position → S_IDLE + home_complete
//   lift command  → S_LIFT_MOVING (run to target steps) → S_LIFT_DONE → S_IDLE + lift_complete
//   top switch    → S_ERROR (requires stop + home to recover)

#include <ArduinoJson.h>

// ─── Pin definitions ──────────────────────────────────────────────────────────
// Both TB6600 PUL+ inputs wired together to one Arduino pin
#define LIFT_STEP_PIN       3

// Both TB6600 DIR+ inputs wired together to one Arduino pin
#define LIFT_DIR_PIN        2

// Limit switches — INPUT_PULLUP, LOW = triggered
#define LIFT_BOTTOM_SWITCH  8   // home / zero reference
#define LIFT_TOP_SWITCH     9   // safety cutoff

// ─── Leadscrew & driver constants ─────────────────────────────────────────────
#define FULL_STEPS_PER_REV  200
#define MICROSTEPS          8
#define STEPS_PER_REV       (FULL_STEPS_PER_REV * MICROSTEPS)   // 1600 steps/rev
#define MM_PER_REV          8.0f                                 // TR8x8 leadscrew
#define STEPS_PER_MM        (STEPS_PER_REV / MM_PER_REV)        // 200.0 steps/mm

// Maximum usable travel — reduce if top switch triggers before this height
#define LIFT_MAX_MM         3000.0f
#define LIFT_MAX_STEPS      ((long)(STEPS_PER_MM * LIFT_MAX_MM)) // 200 000 steps

// ─── Speed constants ──────────────────────────────────────────────────────────
// Homing: slow descent so the bottom switch is not hit hard
//   400 steps/sec ÷ 200 steps/mm = 2 mm/sec
#define LIFT_HOMING_SPEED   4000UL

// Normal travel speed
//   3000 steps/sec ÷ 200 steps/mm = 15 mm/sec
//   If you hear buzzing/stuttering, reduce in steps of 300 until smooth
#define LIFT_SPEED          4000UL

// ─── Timing constants ─────────────────────────────────────────────────────────
// TB6600 minimum PUL high/low time: 1.5 µs — 2 µs is safe
#define STEP_PULSE_US       5

// TB6600 DIR setup time before PUL rising edge: 5 µs minimum
#define DIR_SETUP_US        5

// ─── Homing backoff ───────────────────────────────────────────────────────────
// Steps to move UP after the bottom switch triggers so it just releases cleanly
//   100 steps ÷ 200 steps/mm = 0.5 mm clearance
#define HOME_BACKOFF_STEPS  100

// ─── Debug output ─────────────────────────────────────────────────────────────
// true  = send a status JSON every STATUS_REPORT_MS (useful during calibration)
// false = silent; Pi only sees replies to commands (use in production)
#define DEBUG_VERBOSE       false
#define STATUS_REPORT_MS    500

#define SERIAL_BAUD         115200

// ─── State machine ─────────────────────────────────────────────────────────────
enum State
{
    S_IDLE,          // waiting for commands
    S_HOME_DESCEND,  // descending toward bottom limit switch
    S_HOME_DONE,     // switch triggered and backoff complete, zero set
    S_LIFT_MOVING,   // moving toward target_pos
    S_LIFT_DONE,     // target reached
    S_ERROR          // fault — requires stop + home to recover
};

static State         current_state = S_IDLE;
static bool          is_homed      = false;
static long          lift_pos      = 0;    // current position in steps from home
static long          target_pos    = 0;    // destination in steps, set by lift command
static unsigned long last_step_us  = 0;    // timestamp of last step pulse (µs)
static unsigned long status_timer  = 0;    // timestamp of last debug status report

// ─── JSON helpers ──────────────────────────────────────────────────────────────
static void send_json(JsonDocument &doc)
{
    serializeJson(doc, Serial);
    Serial.println();
}

static void send_status()
{
    StaticJsonDocument<256> doc;
    doc["type"]       = "status";
    doc["state"]      = (current_state == S_IDLE)  ? "IDLE"
                      : (current_state == S_ERROR) ? "ERROR"
                      :                              "BUSY";
    doc["homed"]      = is_homed;
    doc["lift_steps"] = lift_pos;
    doc["lift_mm"]    = (float)lift_pos / STEPS_PER_MM;
    doc["target_mm"]  = (float)target_pos / STEPS_PER_MM;
    doc["bottom_sw"]  = (digitalRead(LIFT_BOTTOM_SWITCH) == LOW);
    doc["top_sw"]     = (digitalRead(LIFT_TOP_SWITCH)    == LOW);
    doc["busy"]       = (current_state != S_IDLE && current_state != S_ERROR);
    send_json(doc);
}

static void send_ack(const char *cmd)
{
    StaticJsonDocument<64> doc;
    doc["type"] = "ack";
    doc["cmd"]  = cmd;
    send_json(doc);
}

void send_error(const char *msg)
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

static void send_lift_result(bool success, const char *reason = "")
{
    StaticJsonDocument<128> doc;
    doc["type"]    = "lift_complete";
    doc["success"] = success;
    if (!success) doc["reason"] = reason;
    send_json(doc);
}

// ─── Stepper helpers ───────────────────────────────────────────────────────────

// Issue a single step pulse toward `target`.
// Non-blocking: returns false if it is not yet time for the next pulse.
// Returns true when lift_pos == target (move complete).
// Sets S_ERROR and returns false if the top switch fires while going up.
static bool step_toward_target(long target, unsigned long speed)
{
    if (lift_pos == target ) return true;

    // Safety: abort immediately if top switch triggers while going up
    if (lift_pos < target && digitalRead(LIFT_TOP_SWITCH) == LOW)
    {
        lift_pos = LIFT_MAX_STEPS;
        send_error("top limit switch triggered — reduce LIFT_MAX_MM and rehome");
        return false;
    }

    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;
    if (now_us - last_step_us < interval) return false;  // not yet time

    last_step_us = now_us;

    bool going_up = (target > lift_pos);
    digitalWrite(LIFT_DIR_PIN, going_up ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    digitalWrite(LIFT_STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(LIFT_STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);

    lift_pos += going_up ? 1 : -1;
    lift_pos  = constrain(lift_pos, 0L, LIFT_MAX_STEPS);

    return (lift_pos == target);
}

// Step toward a limit switch during homing only.
// Non-blocking: returns true only when switch_pin reads LOW (triggered).
static bool step_to_switch(uint8_t switch_pin, bool going_up, unsigned long speed)
{
    //Serial.print("in step_to_switch");
    if (digitalRead(switch_pin) == LOW) return true;  // already triggered

    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;
    if (now_us - last_step_us < interval) return false;

    last_step_us = now_us;

    digitalWrite(LIFT_DIR_PIN, going_up ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    digitalWrite(LIFT_STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(LIFT_STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
    //Serial.println("......moved one step");

    return (digitalRead(switch_pin) == LOW);
}

// Blocking backoff — only called once after the home switch triggers.
// `steps` is small (100 = 0.5 mm), so the brief block is acceptable.
static void backoff_from_switch(int steps, bool going_up, unsigned long speed)
{
    digitalWrite(LIFT_DIR_PIN, going_up ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    unsigned long interval = 1000000UL / speed;

    for (int i = 0; i < steps; i++)
    {
        digitalWrite(LIFT_STEP_PIN, HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(LIFT_STEP_PIN, LOW);
        delayMicroseconds(STEP_PULSE_US);
        delayMicroseconds(interval);
    }
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

    // status and stop are always accepted regardless of state
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

    // all other commands require the machine to be idle
    if (current_state != S_IDLE && current_state != S_ERROR)
    {
        send_error("busy — wait for current sequence to complete");
        return;
    }

    if (strcmp(cmd, "home") == 0)
    {
        is_homed      = false;
        current_state = S_HOME_DESCEND;
        send_ack("home");
        return;
    }

    if (strcmp(cmd, "lift") == 0)
    {
        if (!is_homed)
        {
            send_error("not homed — send home command first");
            return;
        }

        float height_mm = doc["height_mm"] | 0.0f;
        height_mm       = constrain(height_mm, 0.0f, LIFT_MAX_MM);
        target_pos      = (long)(height_mm * STEPS_PER_MM);
        current_state   = S_LIFT_MOVING;
        send_ack("lift");
        return;
    }

    send_error("unknown command");
}

// ─── State machine ─────────────────────────────────────────────────────────────
static void run_state_machine()
{
    switch (current_state)
    {
        case S_IDLE:
        case S_ERROR:
            // Nothing to do — waiting for a command from the Pi
            break;

        case S_HOME_DESCEND:
            // Drive DOWN (going_up = false) until the bottom switch triggers
            if (step_to_switch(LIFT_BOTTOM_SWITCH, false, LIFT_HOMING_SPEED))
            {
                // Switch triggered — back off upward so it just releases
                backoff_from_switch(HOME_BACKOFF_STEPS, true, LIFT_HOMING_SPEED / 2);
                lift_pos      = 0;
                current_state = S_HOME_DONE;
            }
            break;

        case S_HOME_DONE:
            is_homed      = true;
            current_state = S_IDLE;
            send_home_result(true);
            break;

        case S_LIFT_MOVING:
            // step_toward_target calls send_error and sets S_ERROR itself if top switch fires
            if (step_toward_target(target_pos, LIFT_SPEED))
            {
                
                current_state = S_LIFT_DONE;
            }
            break;

        case S_LIFT_DONE:
            current_state = S_IDLE;
            send_lift_result(true);
            break;
    }
}

// ─── setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(SERIAL_BAUD);

    pinMode(LIFT_STEP_PIN, OUTPUT);
    pinMode(LIFT_DIR_PIN,  OUTPUT);

    digitalWrite(LIFT_STEP_PIN, LOW);
    digitalWrite(LIFT_DIR_PIN,  LOW);

    pinMode(LIFT_BOTTOM_SWITCH, INPUT_PULLUP);
    pinMode(LIFT_TOP_SWITCH,    INPUT_PULLUP);

    StaticJsonDocument<64> boot;
    boot["type"] = "boot";
    boot["msg"]  = "lift ready";
    send_json(boot);
}

// ─── loop ─────────────────────────────────────────────────────────────────────
void loop()
{
    // Check for incoming commands from the Pi
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
        {
            handle_command(line);
        }
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