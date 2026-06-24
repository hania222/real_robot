// lift_main.ino
// Hardware:
//   2x NEMA23            — 1.8 degree step angle, 2.5A rated
//   2x TB6600 drivers     — set to 2.5A, 8 microsteps (1600 steps/rev)
//   TR8x8 leadscrew      — 8mm lead per revolution (4 starts x 2mm pitch)
//   2x limit switches    — bottom (home) and top (safety)
//
// NoteBox:
//   1.8 deg/step -> 200 full steps/rev
//   8 microsteps -> 200 x 8 = 1600 steps/rev
//   TR8x8 lead   -> 8mm per revolution
//   Resolution   -> 1600 / 8 = 200 steps/mm
//
// Serial protocol (115200 baud, newline terminated JSON):
//
// How it works??
//Pi sends home -> Arduino descends slowly until bottom switch triggers -> backs off 0.5mm -> sets position to zero -> sends home_complete -> Pi now knows it's safe to send lift commands
//Pi sends lift with a height -> Arduino drives both motors up/down until it reaches that exact step count -> "sends lift_complete" ->Pi moves on to the next task
//If the top switch triggers at any point during upward movement -> Arduino immediately stops, disables drivers, sends an error -> Pi knows something is wrong
//   Pi → Uno commands:
//     {"cmd":"home"}
//     {"cmd":"lift","height_mm":100.0}
//     {"cmd":"status"}
//     {"cmd":"stop"}
//
//   Uno → Pi replies:
//     {"type":"boot", "msg":"lift ready"}
//     {"type":"ack","cmd":"home"}
//     {"type":"home_complete","success":true}
//     {"type":"ack", "cmd":"lift"}
//     {"type":"lift_complete","success":true}
//     {"type":"lift_complete","success":false,"reason":"..."}
//     {"type":"status", "state":"IDLE","homed":true,"lift_mm":150.0,...}
//     {"type":"error","msg":"..."}

#include <ArduinoJson.h>

// Pin definition
// Both DM542 PUL- inputs wired together to this single pin
#define LIFT_STEP_PIN        4

// Both DM542 DIR- inputs wired together to this single pin
#define LIFT_DIR_PIN         5

// DM542 driver 1 ENA- pin  (controls motor 1 only)
#define LIFT_ENA1_PIN        6

// DM542 driver 2 ENA- pin  (controls motor 2 only)
#define LIFT_ENA2_PIN        7

// Bottom limit switch — triggers when lift is fully descended (home position)
// Wired: one leg to pin, other leg to GND — INPUT_PULLUP, LOW = triggered
#define LIFT_BOTTOM_SWITCH   8

// Top limit switch — safety cutoff at maximum height
// Wired: one leg to pin, other leg to GND — INPUT_PULLUP, LOW = triggered
#define LIFT_TOP_SWITCH      9



#define FULL_STEPS_PER_REV   200
#define MICROSTEPS   8
#define STEPS_PER_REV        (FULL_STEPS_PER_REV * MICROSTEPS)   // 1600
#define MM_PER_REV           8.0                                 //linear travel per revolution
#define STEPS_PER_MM         (STEPS_PER_REV / MM_PER_REV)        // 200.0
// Usable travel — start at 1000mm( 1m ),will be reduced if top switch triggers before this (Full screw is 1.2m but coupler can use some length)
#define LIFT_MAX_MM          1000.0
#define LIFT_MAX_STEPS       ((long)(STEPS_PER_MM * LIFT_MAX_MM)) // 200,000

// Speed constants
// 400 steps/sec = 400/200= 2 mm/sec ( Homing speed:slow descent so bottom switch is not hit hard)
#define LIFT_HOMING_SPEED    400UL

// Normal travel speed: 3000 steps/sec = 15 mm/sec,(note: if we heared buzzing, reduce in 300 step/sec increments until smooth)
#define LIFT_SPEED           3000UL

// DM542 pulse width
#define STEP_PULSE_US        5

// DM542 datasheet DIR setup time before PUL rising edge: 5 microseconds
#define DIR_SETUP_US         5


// Steps to move UP after bottom switch triggers so it just releases cleanly
#define HOME_BACKOFF_STEPS   100 // 100 steps = 100 / 200 steps/mm = 0.5mm clearance

#define SERIAL_BAUD          115200

//Debug verbose 
// true = send status every STATUS_REPORT_MS (use during calibration)
// false = silent, Pi only sees replies to commands (use in production)
#define DEBUG_VERBOSE        true
#define STATUS_REPORT_MS     500



enum State
{
    S_IDLE,          // waiting for commands, drivers disabled
    S_HOME_DESCEND,  // descending toward bottom limit switch
    S_HOME_DONE,     // switch triggered and backoff complete, zero set
    S_LIFT_MOVING,   // moving toward target_pos
    S_LIFT_DONE,     // target reached, drivers stay enabled to hold position
    S_ERROR          // fault state, requires stop + home to recover
};

static State         current_state = S_IDLE;
static bool          is_homed      = false;
static long          lift_pos      = 0;     // current position in steps from home
static long          target_pos    = 0;     // destination in steps, set by lift command
static unsigned long last_step_us  = 0;     // timestamp of last step pulse (microseconds)
static unsigned long status_timer  = 0;     // timestamp of last debug status report

static void enable_drivers()
{
    digitalWrite(LIFT_ENA1_PIN, LOW);
    digitalWrite(LIFT_ENA2_PIN, LOW);
}
static void disable_drivers()
{
    digitalWrite(LIFT_ENA1_PIN, HIGH);
    digitalWrite(LIFT_ENA2_PIN, HIGH);
}

static bool step_toward_target(long target, unsigned long speed)
{
    if (lift_pos == target)
    {
        return true;
    }

    //  abort if top switch triggers while going up
    if (digitalRead(LIFT_TOP_SWITCH) == LOW)
    {
        disable_drivers();
        lift_pos = LIFT_MAX_STEPS;
        send_error("top limit switch triggered — reduce LIFT_MAX_MM and rehome");
        return false;
    }

    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;

    if (now_us - last_step_us < interval)
    {
        return false;
    }
    last_step_us = now_us;

    enable_drivers();

    bool going_up = (target > lift_pos);
    digitalWrite(LIFT_ DIR_PIN, going_up ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    digitalWrite(LIFT_STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(LIFT_STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);

    if (going_up)
    {
        lift_pos += 1;
    }
    else
    {
        lift_pos -= 1;
    }

    lift_pos = constrain(lift_pos, 0L, LIFT_MAX_STEPS);

    return (lift_pos == target);
}


// step toward a limit switch during homing only
// Returns true when the switch reads LOW (triggered)
static bool step_to_switch(uint8_t switch_pin, bool going_up, unsigned long speed)
{
    if (digitalRead(switch_pin) == LOW)
    {
        return true;
    }
    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;
    if (now_us - last_step_us < interval)
    {
        return false;
    }
    last_step_us = now_us;

    enable_drivers();
    digitalWrite(LIFT_DIR_PIN, going_up ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);

    digitalWrite(LIFT_STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(LIFT_STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);

    return (digitalRead(switch_pin) == LOW);
}


//  Blocking backoff after homing switch triggers
static void backoff_from_switch( int steps, bool going_up, unsigned long speed)
{
    enable_drivers();
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

static void send_json(JsonDocument &doc)
{
    serializeJson(doc, Serial);
    Serial.println();
}

static void send_status()
{
    StaticJsonDocument<256> doc;
    doc["type"]      = "status";
    doc["state"]     = (current_state == S_IDLE)  ? "IDLE"  : (current_state == S_ERROR) ? "ERROR" : "BUSY";
    doc["homed"]     = is_homed;
    doc["lift_steps"]= lift_pos;
    doc["lift_mm"]   = (float)lift_pos / STEPS_PER_MM;
    doc["target_mm"] = (float)target_pos / STEPS_PER_MM;
    doc["bottom_sw"] = (digitalRead(LIFT_BOTTOM_SWITCH) == LOW);
    doc["top_sw"]    = (digitalRead(LIFT_TOP_SWITCH)    == LOW);
    doc["busy"]      = (current_state != S_IDLE && current_state != S_ERROR);
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
    if (!success)
    {
        doc["reason"] = reason;
    }
    send_json(doc);
}

static void send_lift_result(bool success, const char *reason = "")
{
    StaticJsonDocument<128> doc;
    doc["type"]    = "lift_complete";
    doc["success"] = success;
    if (!success)
    {
        doc["reason"] = reason;
    }
    send_json(doc);
}


// Command handler 
static void handle_command(const String &line)
{
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, line);

    if (error)
    {
        send_error("JSON parse error");
        return;
    }

    const char *cmd = doc["cmd"] | ""; // default to empty string if "cmd" is missing 

    if (strcmp(cmd, "status") == 0)
    {
        send_status();
        return;
    }

    if (strcmp(cmd, "stop") == 0)
    {
        disable_drivers();
        current_state = S_IDLE;
        is_homed      = false;
        send_ack("stop");
        return;
    }

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

        if (height_mm < 0.0f)
        {
            height_mm = 0.0f;
        }

        if (height_mm > LIFT_MAX_MM)
        {
            height_mm = LIFT_MAX_MM;
        }

        target_pos    = (long)(height_mm * STEPS_PER_MM);
        current_state = S_LIFT_MOVING;
        send_ack("lift");
        return;
    }

    send_error("unknown command");
}


// Finite state machine 
static void run_state_machine()
{
    switch (current_state)
    {
        case S_IDLE:
        case S_ERROR:
            // Nothing to do, waiting for a command from Pi
            break;

        case S_HOME_DESCEND:
            // Drive DOWN until bottom switch triggers
            // going_up = false because we are descending toward home
            if (step_to_switch(LIFT_BOTTOM_SWITCH, false, LIFT_HOMING_SPEED))
            {
                // Switch triggered — back off upward so switch just releases
                backoff_from_switch( HOME_BACKOFF_STEPS, true, LIFT_HOMING_SPEED / 2);
                lift_pos      = 0;
                current_state = S_HOME_DONE;
            }
            break;

        case S_HOME_DONE:
            disable_drivers();
            is_homed      = true;
            current_state = S_IDLE;
            send_home_result(true);
            break;

        case S_LIFT_MOVING:
            // If top switch fires it calls send_error and sets S_ERROR itself
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


void setup()
{
    Serial.begin(SERIAL_BAUD);

    pinMode(LIFT_STEP_PIN,      OUTPUT);
    pinMode(LIFT_DIR_PIN,       OUTPUT);
    pinMode(LIFT_ENA1_PIN,      OUTPUT);
    pinMode(LIFT_ENA2_PIN,      OUTPUT);

    digitalWrite(LIFT_STEP_PIN, LOW);
    digitalWrite(LIFT_DIR_PIN,  LOW);

    // Start with drivers disabled — lift is not homed yet
    disable_drivers();

    pinMode(LIFT_BOTTOM_SWITCH, INPUT_PULLUP);
    pinMode(LIFT_TOP_SWITCH,    INPUT_PULLUP);

    StaticJsonDocument<64> boot;
    boot["type"] = "boot";
    boot["msg"]  = "lift ready";
    send_json(boot);
}


void loop()
{
    // Check for incoming command from Pi
    // readStringUntil() completes instantly because Pi sends complete lines
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