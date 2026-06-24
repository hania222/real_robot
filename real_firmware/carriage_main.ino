// OUR Architecture:
//   Pi 5 sends ONE command --> ESP32 runs full sequence alone
//   Pi receives ONE reply when sequence is complete
//   Pi never knows what step the arm is on

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

// function to return readable names
static const char* state_label(State s)
{
    if (s == S_IDLE)  return "IDLE";
    if (s == S_ERROR) return "ERROR";

    if (s == S_HOME_OPEN_TABS || s == S_HOME_WAIT_TABS || s == S_HOME_RETRACT_ARM || s == S_HOME_ROTATE || s == S_HOME_DONE)
    {
        return "HOMING";
    }

    return "PICKING";
}

static State   current_state      = S_IDLE;
static bool    is_homed           = false;
static long    rot_pos            = 0;
static long    ext_pos            = 0;
static long    target_rot         = 0;
static long    target_ext         = 0;
static String  grip_state         = "open";
static unsigned long servo_timer  = 0;
static unsigned long status_timer = 0;
static unsigned long last_rot_step_us = 0;
static unsigned long last_ext_step_us = 0;

static Servo servo_left;
static Servo servo_right;

// drivers (TB6600) control
static void enable_driver(uint8_t ena_pin)
{
    digitalWrite(ena_pin, LOW);
}

static void disable_driver(uint8_t ena_pin)
{
    digitalWrite(ena_pin, HIGH);
}

static void disable_all_drivers()
{
    disable_driver(ROT_ENA_PIN);
    disable_driver(EXT1_ENA_PIN);
    disable_driver(EXT2_ENA_PIN);
}

// Enable both extension drivers together
static void enable_ext_drivers()
{
    enable_driver(EXT1_ENA_PIN);
    enable_driver(EXT2_ENA_PIN);
}

// Disable both extension drivers together
static void disable_ext_drivers()
{
    disable_driver(EXT1_ENA_PIN);
    disable_driver(EXT2_ENA_PIN);
}


// Single step toward target, non-blocking
// interval = 1000000 / speed(1500steps/sec) = 667 microsecs/step, so a pulse is generated every 667 microseconds until the target is reached
// Each loop() call checks if the 667 microseconds interval has passed
// If yes: send one pulse and update position
// If no: return false and do nothing
// Returns true when position equals target
static bool step_toward(uint8_t step_pin, uint8_t dir_pin, uint8_t ena_pin, long target, long &pos, long pos_min, long pos_max, unsigned long speed, unsigned long &last_step_us)
{
    if (pos == target) return true;
    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;
    if (now_us - last_step_us < interval) return false;
    last_step_us = now_us;
    enable_driver(ena_pin);
    bool going_positive = (target > pos);
    if (going_positive)
    {
        digitalWrite(dir_pin, HIGH); // Set direction to positive
    }
    else
    {
        digitalWrite(dir_pin, LOW);
    }
    delayMicroseconds(DIR_SETUP_US); // 5 microseconds safe delay to ensure DIR is registered before STEP pulse

    // Send STEP pulse: HIGH for STEP_PULSE_US, then LOW for STEP_PULSE_US
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(STEP_PULSE_US);

    // Update position based on direction
    if (going_positive)
    {
        pos += 1;
    }
    else
    {
        pos -= 1;
    }
    pos = constrain(pos, pos_min, pos_max);
    if (pos == target)
    {
        return true;
    }
    return false;
}


// Step toward limit switch(search for home position=homing), non-blocking (Returns true when switch reads LOW (triggered))
static bool step_to_switch(uint8_t step_pin, uint8_t dir_pin, uint8_t ena_pin, uint8_t switch_pin, bool positive_dir, unsigned long speed, unsigned long &last_step_us)
{
    if (digitalRead(switch_pin) == LOW) return true; // if Already triggered, no need to step

    unsigned long now_us   = micros();
    unsigned long interval = 1000000UL / speed;
    if (now_us - last_step_us < interval) return false;
    last_step_us = now_us;
    enable_driver(ena_pin);
    if (positive_dir)
    {
        digitalWrite(dir_pin, HIGH);
    }
    else
    {
        digitalWrite(dir_pin, LOW);
    }
    delayMicroseconds(DIR_SETUP_US);
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(STEP_PULSE_US);
    if (digitalRead(switch_pin) == LOW)
    {
        return true;
    }
    return false;
}

// Backoff after switch triggers, Moves away from switch so it just releases
// the only blocking function --> runs once per homing
static void backoff_from_switch(uint8_t step_pin, uint8_t dir_pin, bool positive_dir, int steps, unsigned long speed)
{
    // Move away from switch for a few steps to ensure it releases
    if (positive_dir)
    {
        digitalWrite(dir_pin, HIGH);
    }
    else
    {
        digitalWrite(dir_pin, LOW);
    }
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


// Servo helpers
static void open_tabs()
{
    servo_left.writeMicroseconds(SERVO_OPEN_US); // 1000 microseconds = -90 degree tabs folded DOWN
    servo_right.writeMicroseconds(SERVO_OPEN_US);
    grip_state  = "open";
    servo_timer = millis();
}


static void close_tabs()
{
    servo_left.writeMicroseconds(SERVO_CLOSED_US); // 2000 microseconds = +90 degree tabs flipped UP
    servo_right.writeMicroseconds(SERVO_CLOSED_US);
    grip_state  = "close";
    servo_timer = millis();
}

// Check if enough time has passed since commanding the servo to allow tabs to settle before next step in the sequence
// BC if we start retracting before tabs fully closed, box can slip and ruin the pick
static bool tabs_settled()
{
    if (millis() - servo_timer >= SERVO_SETTLE_MS)
    {
        return true;
    }
    return false;
}


// JSON output helpers to send structured messages back to Pi for debugging and status updates
static void send_json(JsonDocument &doc) // pass by reference to avoid copying and large memory usage
{
    serializeJson(doc, Serial);
    Serial.println();
}


// JSON message helpers to send specific types of messages back to Pi
static void send_status()
{
    StaticJsonDocument<200> doc;
    doc["type"]    = "status";
    doc["state"]   = state_label(current_state);
    doc["homed"]   = is_homed;
    doc["rot_pos"] = rot_pos;
    doc["ext_pos"] = ext_pos;
    doc["grip"]    = grip_state;
    doc["arm_sw"]  = (digitalRead(ARM_HOME_SWITCH_PIN) == LOW);
    doc["arm_end"] = (digitalRead(ARM_END_SWITCH_PIN)  == LOW);
    doc["rot_sw"]  = (digitalRead(ROT_HOME_SWITCH_PIN) == LOW);

    if (current_state != S_IDLE && current_state != S_ERROR)
    {
        doc["busy"] = true;
    }
    else
    {
        doc["busy"] = false;
    }

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
    StaticJsonDocument<100> doc;
    doc["type"] = "error";
    doc["msg"]  = msg;
    send_json(doc);
    current_state = S_ERROR;
}

static void send_home_result(bool success, const char *reason = "")
{
    StaticJsonDocument<100> doc;
    doc["type"]    = "home_complete";
    doc["success"] = success;
    if (!success)
    {
        doc["reason"] = reason;
    }
    send_json(doc);
}

static void send_pick_result(bool success, const char *reason = "")
{
    StaticJsonDocument<100> doc;
    doc["type"]    = "pick_complete";
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

    const char *cmd = doc["cmd"] | "";

    // Always allowed regardless of state
    if (strcmp(cmd, "status") == 0) // compare cmd with "status", if equal return 0
    {
        send_status();
        return;
    }

    if (strcmp(cmd, "stop") == 0)
    {
        disable_all_drivers();
        current_state = S_IDLE;
        is_homed      = false;
        send_ack("stop");
        return;
    }

    // All other commands require IDLE state
    if (current_state != S_IDLE && current_state != S_ERROR)
    {
        send_error("busy — wait for sequence to complete");
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

        if (target_rot > ROT_MAX_STEPS)
        {
            target_rot = ROT_MAX_STEPS;
        }
        if (target_rot < -ROT_MAX_STEPS)
        {
            target_rot = -ROT_MAX_STEPS;
        }
        if (target_ext < 0)
        {
            target_ext = 0;
        }
        if (target_ext > EXT_MAX_STEPS)
        {
            target_ext = EXT_MAX_STEPS;
        }

        current_state = S_PICK_OPEN_TABS;
        send_ack("pick");
        return;
    }

    send_error("unknown command");
}

// Finite State machine
//   Note (for centrifugal force): Rotation states where EXT driver must stay ON:
//   S_PICK_ROTATE        (rotating to face shelf)
//   S_PICK_RETURN_ROTATE (rotating back to center)
//   S_HOME_ROTATE        (rotating to home switch)
static void run_state_machine()
{
    switch (current_state)
    {
        case S_IDLE:
        case S_ERROR:
            break;

        // HOMING SEQUENCE
        case S_HOME_OPEN_TABS:
            // Open tabs before any movement Even if tabs are already open from a previous cycle (safety)
            open_tabs();
            current_state = S_HOME_WAIT_TABS;
            break;

        case S_HOME_WAIT_TABS:
            if (tabs_settled())
            {
                current_state = S_HOME_RETRACT_ARM;
            }
            break;

        case S_HOME_RETRACT_ARM:
            // Drive arm toward home switch (retract direction)
            // false = negative direction = toward retracted end
            enable_ext_drivers();
            if (step_to_switch(EXT_STEP_PIN, EXT_DIR_PIN, EXT1_ENA_PIN, ARM_HOME_SWITCH_PIN, false, EXT_HOMING_SPEED, last_ext_step_us))
            {
                backoff_from_switch(EXT_STEP_PIN, EXT_DIR_PIN, true, 40, EXT_HOMING_SPEED / 2);
                ext_pos       = 0;
                current_state = S_HOME_ROTATE;
            }
            break;

        case S_HOME_ROTATE:
            // keep extension drivers ON during rotation so motors hold arms during rotation homing
            enable_ext_drivers();

            // Drive rotation toward center home switch
            if (step_to_switch(ROT_STEP_PIN, ROT_DIR_PIN, ROT_ENA_PIN, ROT_HOME_SWITCH_PIN, false, ROT_HOMING_SPEED, last_rot_step_us))
            {
                backoff_from_switch(ROT_STEP_PIN, ROT_DIR_PIN, true, 30, ROT_HOMING_SPEED / 2);
                rot_pos       = 0;
                current_state = S_HOME_DONE;
            }
            break;

        case S_HOME_DONE:
            disable_all_drivers();
            is_homed      = true;
            current_state = S_IDLE;
            send_home_result(true);
            break;

        // PICKING SEQUENCE (when the Pi says "PICK" after it received "Homed")
        case S_PICK_OPEN_TABS:
            // Open tabs at start of every pick (safety)
            open_tabs();
            current_state = S_PICK_WAIT_TABS_OPEN;
            break;

        case S_PICK_WAIT_TABS_OPEN:
            if (tabs_settled())
            {
                current_state = S_PICK_ROTATE;
            }
            break;

        case S_PICK_ROTATE:
            // keep extension drivers ON during rotation so motors hold arms during rotation
            enable_ext_drivers();

            if (step_toward(ROT_STEP_PIN, ROT_DIR_PIN, ROT_ENA_PIN, target_rot, rot_pos, -ROT_MAX_STEPS, ROT_MAX_STEPS, ROT_SPEED, last_rot_step_us))
            {
                current_state = S_PICK_EXTEND;
            }
            break;

        case S_PICK_EXTEND:
            // Check end limit switch on every loop call during extension
            // If arms hit the end switch before reaching target_ext, stop immediately and abort pick to prevent mechanical damage
            if (digitalRead(ARM_END_SWITCH_PIN) == LOW)
            {
                ext_pos = EXT_MAX_STEPS;
                disable_ext_drivers();
                send_pick_result(false, "arm end switch triggered during extension");
                current_state = S_ERROR;
                break;
            }
            // EXT1_ENA_PIN used as the step_toward ena_pin argument because both drivers
            // share the same STEP and DIR pins — enabling EXT1 through step_toward is enough to trigger stepping on both motors since STEP/DIR are wiredin parallel
            enable_ext_drivers();
            if (step_toward(EXT_STEP_PIN, EXT_DIR_PIN, EXT1_ENA_PIN, target_ext, ext_pos, 0, EXT_MAX_STEPS, EXT_SPEED, last_ext_step_us))
            {
                current_state = S_PICK_CLOSE_TABS;
            }
            break;

        case S_PICK_CLOSE_TABS: // servo closes
            close_tabs();
            current_state = S_PICK_WAIT_TABS_CLOSE;
            break;

        case S_PICK_WAIT_TABS_CLOSE:
            // to allow tabs to fully close before retraction
            // note: If box slipped during retraction: increase SERVO_SETTLE_MS
            if (tabs_settled())
            {
                current_state = S_PICK_RETRACT;
            }
            break;

        case S_PICK_RETRACT:
            // Retract arms — box locked by tabs, comes with them
            enable_ext_drivers();
            if (step_toward(EXT_STEP_PIN, EXT_DIR_PIN, EXT1_ENA_PIN, 0, ext_pos, 0, EXT_MAX_STEPS, EXT_SPEED, last_ext_step_us))
            {
                current_state = S_PICK_RETURN_ROTATE;
            }
            break;

        case S_PICK_RETURN_ROTATE:
            // keep extension drivers ON during return rotation so motors hold arms during rotation
            enable_ext_drivers();

            if (step_toward(ROT_STEP_PIN, ROT_DIR_PIN, ROT_ENA_PIN, 0, rot_pos, -ROT_MAX_STEPS, ROT_MAX_STEPS, ROT_SPEED, last_rot_step_us))
            {
                current_state = S_PICK_DONE;
            }
            break;

        case S_PICK_DONE:
            disable_all_drivers();
            current_state = S_IDLE;
            send_pick_result(true);
            break;
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD);

    pinMode(ROT_STEP_PIN, OUTPUT);
    pinMode(ROT_DIR_PIN,  OUTPUT);
    pinMode(ROT_ENA_PIN,  OUTPUT);
    digitalWrite(ROT_STEP_PIN, LOW);
    digitalWrite(ROT_DIR_PIN,  LOW);
    disable_driver(ROT_ENA_PIN);

    pinMode(EXT_STEP_PIN,  OUTPUT);
    pinMode(EXT_DIR_PIN,   OUTPUT);
    pinMode(EXT1_ENA_PIN,  OUTPUT);
    pinMode(EXT2_ENA_PIN,  OUTPUT);
    digitalWrite(EXT_STEP_PIN, LOW);
    digitalWrite(EXT_DIR_PIN,  LOW);
    disable_driver(EXT1_ENA_PIN);
    disable_driver(EXT2_ENA_PIN);

    pinMode(ARM_HOME_SWITCH_PIN, INPUT_PULLUP);
    pinMode(ARM_END_SWITCH_PIN,  INPUT_PULLUP);
    pinMode(ROT_HOME_SWITCH_PIN, INPUT_PULLUP);

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


void loop()
{
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