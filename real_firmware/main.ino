#include <ArduinoJson.h>
#include "config.h"

// Timing parameters
#define FEEDBACK_HZ  50 //20 ms
#define CONTROL_HZ   50
const unsigned long FEEDBACK_PERIOD_US = 1000000UL / FEEDBACK_HZ;
const unsigned long CONTROL_PERIOD_US  = 1000000UL / CONTROL_HZ;

// Wheel target state (set from serial commands, sent by hardware_bridge_node.py)
float left_wheel_target_ms   = 0.0f;
float right_wheel_target_ms  = 0.0f;
float left_slewed_target_ms  = 0.0f;
float right_slewed_target_ms = 0.0f;

// PWM currently being sent to each motor (sent in feedback JSON for debugging)
float left_motor_pwm  = 0.0f;
float right_motor_pwm = 0.0f;

// Loop timestamps
unsigned long last_control_time_us  = 0;
unsigned long last_feedback_time_us = 0;
unsigned long last_command_time_ms  = 0;

// BTS7960 motor driver control functions
void setLeftMotor(float signed_pwm) {
    if (fabsf(signed_pwm) < PWM_DEADBAND) {
        analogWrite(L_RPWM, 0);
        analogWrite(L_LPWM, 0);
        return;
    }
    int pwm_magnitude = (int)constrain(fabsf(signed_pwm), 0, 255);
    if (signed_pwm >= 0.0f) {
        analogWrite(L_RPWM, pwm_magnitude); analogWrite(L_LPWM, 0); } // Forward
    else {
        analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwm_magnitude); } // Reverse
}

void setRightMotor(float signed_pwm) {
    if (fabsf(signed_pwm) < PWM_DEADBAND) {
        analogWrite(R_RPWM, 0);
        analogWrite(R_LPWM, 0);
        return;
    }
    int pwm_magnitude = (int)constrain(fabsf(signed_pwm), 0, 255);
    if (signed_pwm >= 0.0f) { analogWrite(R_RPWM, pwm_magnitude); analogWrite(R_LPWM, 0); } // Forward
    else                    { analogWrite(R_RPWM, 0); analogWrite(R_LPWM, pwm_magnitude); } // Reverse
}

// Slew-rate limiter — smooths sudden joystick jumps into gradual PWM changes
float slewToward(float current_velocity, float desired_velocity, float max_step_per_tick) {
    return current_velocity + constrain(desired_velocity - current_velocity, -max_step_per_tick, max_step_per_tick);
}

// Serial command parser — reads JSON commands sent by hardware_bridge_node.py
void parseSerialCommand() {
    if (!Serial.available()) return;

    static char serial_buffer[128];
    int bytes_read = Serial.readBytesUntil('\n', serial_buffer, sizeof(serial_buffer) - 1);
    if (bytes_read <= 0) return;
    serial_buffer[bytes_read] = '\0';

    StaticJsonDocument<128> json_doc;
    if (deserializeJson(json_doc, serial_buffer) != DeserializationError::Ok) return;

    if (json_doc.containsKey("left_target"))  left_wheel_target_ms  = json_doc["left_target"].as<float>();
    if (json_doc.containsKey("right_target")) right_wheel_target_ms = json_doc["right_target"].as<float>();

    last_command_time_ms = millis();
}

// Feedback sender — reports current target/PWM state back to the Pi for debugging.
// No encoder or IMU data since neither is used.
void sendFeedback() {
    char output_line[160];
    snprintf(output_line, sizeof(output_line),
        "{"
        "\"left_target\":%.4f,\"right_target\":%.4f,"
        "\"left_pwm\":%.1f,\"right_pwm\":%.1f"
        "}\n",
        left_slewed_target_ms, right_slewed_target_ms,
        left_motor_pwm,        right_motor_pwm
    );
    Serial.print(output_line);
}

void setup() {
    Serial.begin(115200);

    pinMode(L_RPWM, OUTPUT); analogWrite(L_RPWM, 0);
    pinMode(L_LPWM, OUTPUT); analogWrite(L_LPWM, 0);
    pinMode(R_RPWM, OUTPUT); analogWrite(R_RPWM, 0);
    pinMode(R_LPWM, OUTPUT); analogWrite(R_LPWM, 0);

    pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
    pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
    pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
    pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

    last_control_time_us  = micros();
    last_feedback_time_us = micros();
    last_command_time_ms  = millis();

    Serial.println("{\"info\":\"firmware ready - open loop, no encoders, no IMU\"}");
}

void loop() {
    unsigned long current_time_us = micros();
    unsigned long current_time_ms = millis();

    parseSerialCommand();

    // Command timeout: if no new command arrives within the timeout window,
    // zero the targets so the robot stops if the Pi disconnects or crashes.
    if ((current_time_ms - last_command_time_ms) > CMD_TIMEOUT_MS) {
        left_wheel_target_ms  = 0.0f;
        right_wheel_target_ms = 0.0f;
    }

    // Control loop at CONTROL_HZ — pure open loop, target velocity maps straight to PWM
    if ((current_time_us - last_control_time_us) >= CONTROL_PERIOD_US) {
        last_control_time_us = current_time_us;

        left_slewed_target_ms  = slewToward(left_slewed_target_ms,  left_wheel_target_ms,  MAX_ACCEL_PER_TICK);
        right_slewed_target_ms = slewToward(right_slewed_target_ms, right_wheel_target_ms, MAX_ACCEL_PER_TICK);

        left_motor_pwm  = constrain(left_slewed_target_ms  * OPEN_LOOP_PWM_PER_MS, PWM_MIN_OUTPUT, PWM_MAX_OUTPUT);
        right_motor_pwm = constrain(right_slewed_target_ms * OPEN_LOOP_PWM_PER_MS, PWM_MIN_OUTPUT, PWM_MAX_OUTPUT);

        setLeftMotor(left_motor_pwm);
        setRightMotor(right_motor_pwm);
    }

    if ((current_time_us - last_feedback_time_us) >= FEEDBACK_PERIOD_US) {
        last_feedback_time_us = current_time_us;
        sendFeedback();
    }
}