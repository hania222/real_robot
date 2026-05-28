#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>
#include "config.h"

// Timing parameters
#define FEEDBACK_HZ  50 //20 ms
#define PID_HZ       50
//in microseconds (to use micros() for timing)
const unsigned long FEEDBACK_PERIOD_US = 1000000UL / FEEDBACK_HZ;
const unsigned long PID_PERIOD_US      = 1000000UL / PID_HZ;

// Encoder tick counters (modified in ISRs )
volatile long left_encoder_ticks  = 0;
volatile long right_encoder_ticks = 0;

void IRAM_ATTR leftEncA_ISR() {
    if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) left_encoder_ticks++;
    else                                                      left_encoder_ticks--;
}
void IRAM_ATTR leftEncB_ISR() {
    if (digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B)) left_encoder_ticks++;
    else                                                      left_encoder_ticks--;
}
void IRAM_ATTR rightEncA_ISR() {
    if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) right_encoder_ticks++;
    else                                                        right_encoder_ticks--;
}
void IRAM_ATTR rightEncB_ISR() {
    if (digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B)) right_encoder_ticks++;
    else                                                        right_encoder_ticks--;
}

// Wheel state variables (updated in loop, sent in feedback JSON)
float left_wheel_velocity_ms      = 0.0f;
float right_wheel_velocity_ms     = 0.0f;
float left_wheel_position_metres  = 0.0f;
float right_wheel_position_metres = 0.0f;
float left_wheel_target_ms        = 0.0f;
float right_wheel_target_ms       = 0.0f;
float left_slewed_target_ms       = 0.0f;
float right_slewed_target_ms      = 0.0f;

//PID output and error (sent in feedback JSON, used for stall detection)
float left_motor_pwm          = 0.0f;
float right_motor_pwm         = 0.0f;
float left_velocity_error_ms  = 0.0f;
float right_velocity_error_ms = 0.0f;

// PID state 
float left_integral_sum  = 0.0f,  left_previous_error  = 0.0f;
float right_integral_sum = 0.0f,  right_previous_error = 0.0f;

//Live-tunable PID gains (initialised from config.h) 
float left_kp  = KP_LEFT,  left_ki  = KI_LEFT,  left_kd  = KD_LEFT;
float right_kp = KP_RIGHT, right_ki = KI_RIGHT, right_kd = KD_RIGHT;

// Stall detection state 
unsigned long left_stall_timer_ms  = 0;
unsigned long right_stall_timer_ms = 0;
bool left_motor_stalled  = false;
bool right_motor_stalled = false;

//IMU 
MPU6050 mpu;

//Loop timestamps 
unsigned long last_pid_time_us      = 0;
unsigned long last_feedback_time_us = 0;
unsigned long last_command_time_ms  = 0;

//BTS7960 motor driver control functions
void setLeftMotor(float signed_pwm) {
    if (fabsf(signed_pwm) < PWM_DEADBAND) {
        analogWrite(L_RPWM, 0);
        analogWrite(L_LPWM, 0);
        return;
    }
    int pwm_magnitude = (int)constrain(fabsf(signed_pwm), 0, 255);
    if (signed_pwm >= 0.0f) { analogWrite(L_RPWM, pwm_magnitude); analogWrite(L_LPWM, 0); } // Forward
    else                    { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, pwm_magnitude); } // Reverse
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

//PID step 
float pidStep(float target_velocity_ms, float actual_velocity_ms,
              float &integral_sum, float &previous_error,
              float kp, float ki, float kd,
              float delta_time_sec,
              float &velocity_error_out)
{
    float velocity_error  = target_velocity_ms - actual_velocity_ms;
    velocity_error_out    = velocity_error;
    integral_sum         += velocity_error * delta_time_sec;
    integral_sum          = constrain(integral_sum, -50.0f, 50.0f);  // Integral windup guard 
    float error_rate      = (velocity_error - previous_error) / delta_time_sec;
    previous_error        = velocity_error;
    return kp * velocity_error + ki * integral_sum + kd * error_rate;
}

// Slew-rate limiter
float slewToward(float current_velocity, float desired_velocity, float max_step_per_tick) {
    return current_velocity + constrain(desired_velocity - current_velocity,
                                        -max_step_per_tick, max_step_per_tick);
}

//  Serial command parser 
void parseSerialCommand() {
    if (!Serial.available()) return;

    static char serial_buffer[256];   // 256 — longer keys need more space than 128
    int bytes_read = Serial.readBytesUntil('\n', serial_buffer, sizeof(serial_buffer) - 1); // Leave space for null terminator
    if (bytes_read <= 0) return; // No valid data read
    serial_buffer[bytes_read] = '\0'; //

    StaticJsonDocument<256> json_doc;
    if (deserializeJson(json_doc, serial_buffer) != DeserializationError::Ok) return;

    if (json_doc.containsKey("left_target"))  left_wheel_target_ms  = json_doc["left_target"].as<float>();
    if (json_doc.containsKey("right_target")) right_wheel_target_ms = json_doc["right_target"].as<float>();

    if (json_doc.containsKey("kp")) { left_kp = right_kp = json_doc["kp"].as<float>(); }
    if (json_doc.containsKey("ki")) { left_ki = right_ki = json_doc["ki"].as<float>(); }
    if (json_doc.containsKey("kd")) { left_kd = right_kd = json_doc["kd"].as<float>(); }

    last_command_time_ms = millis();
}

//  Stall detection ( if motor is commanded near max PWM but wheel speed is near zero for too long, cut power to prevent damage)
void checkStall(unsigned long current_time_ms) {
    bool left_pwm_near_max     = fabsf(left_motor_pwm)          > STALL_PWM_THRESHOLD;
    bool right_pwm_near_max    = fabsf(right_motor_pwm)         > STALL_PWM_THRESHOLD;
    bool left_speed_near_zero  = fabsf(left_wheel_velocity_ms)  < STALL_SPEED_THRESHOLD;
    bool right_speed_near_zero = fabsf(right_wheel_velocity_ms) < STALL_SPEED_THRESHOLD;

    if (left_pwm_near_max && left_speed_near_zero) {
        if (left_stall_timer_ms == 0) left_stall_timer_ms = current_time_ms;
        if ((current_time_ms - left_stall_timer_ms) > STALL_TIME_MS) {
            left_motor_stalled = true;
            setLeftMotor(0);
            Serial.println("{\"warn\":\"left wheel stall - motor cut\"}");
        }
    } else {
        left_stall_timer_ms = 0;
        left_motor_stalled  = false;
    }

    if (right_pwm_near_max && right_speed_near_zero) {
        if (right_stall_timer_ms == 0) right_stall_timer_ms = current_time_ms;
        if ((current_time_ms - right_stall_timer_ms) > STALL_TIME_MS) {
            right_motor_stalled = true;
            setRightMotor(0);
            Serial.println("{\"warn\":\"right wheel stall - motor cut\"}");
        }
    } else {
        right_stall_timer_ms = 0;
        right_motor_stalled  = false;
    }
}

// Feedback sender 
void sendFeedback() {
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_gyro_x,  raw_gyro_y,  raw_gyro_z;
    mpu.getMotion6(&raw_accel_x, &raw_accel_y, &raw_accel_z,
                   &raw_gyro_x,  &raw_gyro_y,  &raw_gyro_z);

    float accel_x_ms2 = raw_accel_x / 16384.0f * 9.81f;
    float accel_y_ms2 = raw_accel_y / 16384.0f * 9.81f;
    float accel_z_ms2 = raw_accel_z / 16384.0f * 9.81f;

    float gyro_x_rads = raw_gyro_x / 131.0f * (M_PI / 180.0f);
    float gyro_y_rads = raw_gyro_y / 131.0f * (M_PI / 180.0f);
    float gyro_z_rads = raw_gyro_z / 131.0f * (M_PI / 180.0f);

    char output_line[320];
    snprintf(output_line, sizeof(output_line),
        "{"
        "\"left_velocity\":%.4f,\"right_velocity\":%.4f,"
        "\"left_position\":%.4f,\"right_position\":%.4f,"
        "\"left_target\":%.4f,\"right_target\":%.4f,"
        "\"left_error\":%.4f,\"right_error\":%.4f,"
        "\"left_pwm\":%.1f,\"right_pwm\":%.1f,"
        "\"accel_x\":%.4f,\"accel_y\":%.4f,\"accel_z\":%.4f,"
        "\"gyro_x\":%.4f,\"gyro_y\":%.4f,\"gyro_z\":%.4f"
        "}\n",
        left_wheel_velocity_ms,      right_wheel_velocity_ms,
        left_wheel_position_metres,  right_wheel_position_metres,
        left_slewed_target_ms,       right_slewed_target_ms,
        left_velocity_error_ms,      right_velocity_error_ms,
        left_motor_pwm,              right_motor_pwm,
        accel_x_ms2, accel_y_ms2, accel_z_ms2,
        gyro_x_rads, gyro_y_rads, gyro_z_rads
    );
    Serial.print(output_line);
}

// setup 
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

    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncA_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  leftEncB_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightEncB_ISR, CHANGE);

    Wire.begin(IMU_SDA, IMU_SCL);
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("{\"warn\":\"MPU6050 not found\"}");
    }

    last_pid_time_us      = micros();
    last_feedback_time_us = micros();
    last_command_time_ms  = millis();

    Serial.println("{\"info\":\"firmware ready\"}");
}

// loop 
void loop() {
    unsigned long current_time_us = micros();
    unsigned long current_time_ms = millis();

    parseSerialCommand();

    // Command timeout: if no new command arrives within the timeout window, stop the robot by setting targets to zero. This prevents runaway if the controlling computer crashes or disconnects.
    if ((current_time_ms - last_command_time_ms) > CMD_TIMEOUT_MS) {
        left_wheel_target_ms  = 0.0f;
        right_wheel_target_ms = 0.0f;
    }
    
    // PID control loop  at PID_Hz
    if ((current_time_us - last_pid_time_us) >= PID_PERIOD_US) {
        float delta_time_sec = (current_time_us - last_pid_time_us) * 1e-6f;
        last_pid_time_us     = current_time_us;

        noInterrupts();
        long left_tick_count  = left_encoder_ticks;
        long right_tick_count = right_encoder_ticks;
        left_encoder_ticks    = 0;
        right_encoder_ticks   = 0;
        interrupts();

        float metres_per_encoder_tick = (2.0f * M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
        // actual velocity
        left_wheel_velocity_ms  = (left_tick_count  * metres_per_encoder_tick) / delta_time_sec;
        right_wheel_velocity_ms = (right_tick_count * metres_per_encoder_tick) / delta_time_sec;
        //actual position (integrate velocity)
        left_wheel_position_metres  += left_tick_count  * metres_per_encoder_tick;
        right_wheel_position_metres += right_tick_count * metres_per_encoder_tick;

        left_slewed_target_ms  = slewToward(left_slewed_target_ms,  left_wheel_target_ms,  MAX_ACCEL_PER_TICK);
        right_slewed_target_ms = slewToward(right_slewed_target_ms, right_wheel_target_ms, MAX_ACCEL_PER_TICK);

        //target velocity with dead-band compensation (if target is nonzero but very low, raise it to minimum to overcome static friction)
        float left_command_ms  = left_slewed_target_ms;
        float right_command_ms = right_slewed_target_ms;
        if (fabsf(left_command_ms)  > 0.0f && fabsf(left_command_ms)  < MIN_WHEEL_SPEED)
            left_command_ms  = copysignf(MIN_WHEEL_SPEED, left_command_ms);
        if (fabsf(right_command_ms) > 0.0f && fabsf(right_command_ms) < MIN_WHEEL_SPEED)
            right_command_ms = copysignf(MIN_WHEEL_SPEED, right_command_ms);

        if (fabsf(left_slewed_target_ms)  < 0.01f) { left_command_ms  = 0.0f; left_integral_sum  = 0.0f; }
        if (fabsf(right_slewed_target_ms) < 0.01f) { right_command_ms = 0.0f; right_integral_sum = 0.0f; }

        float raw_left_pwm = pidStep(
            left_command_ms, left_wheel_velocity_ms,
            left_integral_sum, left_previous_error,
            left_kp, left_ki, left_kd,
            delta_time_sec, left_velocity_error_ms
        );
        float raw_right_pwm = pidStep(
            right_command_ms, right_wheel_velocity_ms,
            right_integral_sum, right_previous_error,
            right_kp, right_ki, right_kd,
            delta_time_sec, right_velocity_error_ms
        );

        left_motor_pwm  = constrain(raw_left_pwm,  PID_MIN_OUTPUT, PID_MAX_OUTPUT);
        right_motor_pwm = constrain(raw_right_pwm, PID_MIN_OUTPUT, PID_MAX_OUTPUT);

        if (!left_motor_stalled)  setLeftMotor(left_motor_pwm);
        if (!right_motor_stalled) setRightMotor(right_motor_pwm);

        checkStall(current_time_ms);
    }

    if ((current_time_us - last_feedback_time_us) >= FEEDBACK_PERIOD_US) {
        last_feedback_time_us = current_time_us;
        sendFeedback();
    }
}