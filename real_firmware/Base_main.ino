#include <ArduinoJson.h>

// ===================== BTS7960 motor driver pins =====================
#define L_RPWM  15
#define L_LPWM   4
#define L_R_EN  27
#define L_L_EN  14

#define R_RPWM  32
#define R_LPWM  33
#define R_R_EN  25
#define R_L_EN  26

// ===================== LEDC PWM config =====================
// 20kHz keeps switching noise above the audible range and gives BTS7960
// cleaner switching than the ~1kHz default analogWrite() emulation on ESP32.
#define PWM_FREQ_HZ   20000
#define PWM_RES_BITS  8      // 0-255 duty range, matches rest of the code

// ===================== Open-loop tuning (46 RPM motor) =====================
// A 46 RPM gear motor has a high reduction ratio -> real gearbox stiction.
// Below MIN_MOVING_PWM the motor just buzzes/stalls instead of turning.
// MEASURE THIS: slowly ramp PWM from 0 with wheels off the ground / under
// light load and note the duty at which the wheel first starts turning
// smoothly. Start with this guess and adjust.
#define MIN_MOVING_PWM   70     // tune per-motor, ~27% duty as a starting point
#define MAX_PWM          255
#define OPEN_LOOP_MAX_MS 0.313f  // must match MAX_LINEAR on the ROS side

// Slew limiting: max PWM change per control step, prevents instant jumps
// from feeling like "cuts" in motion.
#define SLEW_STEP_PER_TICK   8      // PWM counts per control tick
#define CONTROL_PERIOD_MS    20     // slew + motor update period

// Command watchdog: if no valid JSON arrives in this window, stop motors.
#define CMD_TIMEOUT_MS  300

float left_wheel_target_ms  = 0.0f;
float right_wheel_target_ms = 0.0f;

int left_pwm_current  = 0;   // signed, current slewed PWM actually applied
int right_pwm_current = 0;

unsigned long last_cmd_millis   = 0;
unsigned long last_control_tick = 0;

// ===================== Motor output =====================
void setLeftMotor(int signed_pwm) {
    int mag = constrain(abs(signed_pwm), 0, MAX_PWM);
    if (signed_pwm >= 0) {
        ledcWrite(L_RPWM, mag);
        ledcWrite(L_LPWM, 0);
    } else {
        ledcWrite(L_RPWM, 0);
        ledcWrite(L_LPWM, mag);
    }
}

void setRightMotor(int signed_pwm) {
    int mag = constrain(abs(signed_pwm), 0, MAX_PWM);
    if (signed_pwm >= 0) {
        ledcWrite(R_RPWM, mag);
        ledcWrite(R_LPWM, 0);
    } else {
        ledcWrite(R_RPWM, 0);
        ledcWrite(R_LPWM, mag);
    }
}

// Map target velocity (m/s, signed) -> signed target PWM with a minimum
// moving floor, so any nonzero command is guaranteed to actually turn the
// wheel instead of sitting below stiction.
int velocityToTargetPwm(float v_ms) {
    if (fabsf(v_ms) < 1e-3f) return 0;
    float mag_ms = fminf(fabsf(v_ms), OPEN_LOOP_MAX_MS);
    int mag_pwm = MIN_MOVING_PWM +
                  (int)(mag_ms * (MAX_PWM - MIN_MOVING_PWM) / OPEN_LOOP_MAX_MS);
    mag_pwm = constrain(mag_pwm, 0, MAX_PWM);
    return (v_ms < 0.0f) ? -mag_pwm : mag_pwm;
}

int applySlew(int current, int target, int max_step) {
    if (target > current) return min(target, current + max_step);
    if (target < current) return max(target, current - max_step);
    return current;
}

// ===================== Non-blocking serial parse =====================
// Accumulates characters as they arrive instead of blocking on
// readBytesUntil(), which could stall the loop for up to Serial's timeout
// if a line arrives split across reads.
void parseSerialCommand() {
    static char buf[128];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            buf[idx] = '\0';
            idx = 0;

            if (strlen(buf) == 0) continue;

            StaticJsonDocument<128> json_doc;
            if (deserializeJson(json_doc, buf) != DeserializationError::Ok) continue;

            if (json_doc.containsKey("left_target"))
                left_wheel_target_ms = json_doc["left_target"].as<float>();
            if (json_doc.containsKey("right_target"))
                right_wheel_target_ms = json_doc["right_target"].as<float>();

            last_cmd_millis = millis();

            // Echo back so you can confirm the board received it
            Serial.print("{\"left_target\":");
            Serial.print(left_wheel_target_ms, 4);
            Serial.print(",\"right_target\":");
            Serial.print(right_wheel_target_ms, 4);
            Serial.println("}");

        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
        // if the line is longer than the buffer, extra chars are dropped
        // until '\n' resets idx — prevents buffer overrun on garbage input
    }
}

// ===================== Control loop: watchdog + slew + output =====================
void updateMotors() {
    unsigned long now = millis();
    if (now - last_control_tick < CONTROL_PERIOD_MS) return;
    last_control_tick = now;

    // Watchdog: no fresh command recently -> force stop targets
    float left_cmd_ms  = left_wheel_target_ms;
    float right_cmd_ms = right_wheel_target_ms;
    if (now - last_cmd_millis > CMD_TIMEOUT_MS) {
        left_cmd_ms  = 0.0f;
        right_cmd_ms = 0.0f;
    }

    int left_target_pwm  = velocityToTargetPwm(left_cmd_ms);
    int right_target_pwm = velocityToTargetPwm(right_cmd_ms);

    left_pwm_current  = applySlew(left_pwm_current,  left_target_pwm,  SLEW_STEP_PER_TICK);
    right_pwm_current = applySlew(right_pwm_current, right_target_pwm, SLEW_STEP_PER_TICK);

    setLeftMotor(left_pwm_current);
    setRightMotor(right_pwm_current);
}

void setup() {
    Serial.begin(115200);

    pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
    pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
    pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
    pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

    ledcAttach(L_RPWM, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(L_LPWM, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(R_RPWM, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(R_LPWM, PWM_FREQ_HZ, PWM_RES_BITS);

    last_cmd_millis   = millis();
    last_control_tick = millis();

    Serial.println("{\"info\":\"firmware ready - open loop, deadband+slew\"}");
}

void loop() {
    parseSerialCommand();
    updateMotors();
}