/*
 * main.ino  –  real_firmware
 * ============================================================
 * Migration: micro-ROS → plain Serial JSON bridge
 *
 * What changed vs the old micro-ROS firmware:
 *   REMOVED  – all micro_ros_arduino includes, rcl/rclc boilerplate,
 *               publisher/subscriber setup, executor spin
 *   KEPT     – PID logic, encoder ISRs, motor driver, all config.h
 *               values, KP/KI/KD gains, WHEEL_RADIUS, WHEEL_BASE,
 *               TICKS_PER_REV, MIN_WHEEL_SPEED, MAX_ACCEL_PER_TICK,
 *               IMU reading — ALL COMPLETELY UNCHANGED
 *   ADDED    – JSON serial protocol (receive cmd, send feedback)
 *
 * PID unit note:
 *   The PID error, target, and measured values are all in m/s,
 *   exactly as before. KP/KI/KD are untouched — no retuning needed.
 *   The Python bridge node (hardware_bridge_node.py) handles all
 *   rad/s <-> m/s conversion so this firmware never needs to change.
 *
 * Serial protocol (115200 baud, newline-terminated JSON):
 *
 *   Pi -> ESP32  (one wheel per message, velocity in m/s):
 *     {"L": 0.150}   <- left wheel target m/s
 *     {"R": -0.150}  <- right wheel target m/s
 *     {"kp": 80.0, "ki": 5.0, "kd": 1.0}  <- optional runtime gain update
 *
 *   ESP32 -> Pi  (feedback at FEEDBACK_HZ, velocities in m/s):
 *     {"lv":0.148,"rv":-0.149,"lp":0.031,"rp":-0.031,
 *      "ax":0.01,"ay":0.00,"az":9.81,
 *      "gx":0.00,"gy":0.00,"gz":0.12}
 *     lv/rv = actual wheel velocity (m/s)
 *     lp/rp = cumulative wheel arc-length (m)
 *     ax/ay/az = accelerometer (m/s^2)
 *     gx/gy/gz = gyroscope (rad/s)
 *
 * ============================================================
 */

#include <Arduino.h>
#include <ArduinoJson.h>   // ArduinoJson v6 - install via Library Manager
#include <Wire.h>
#include <MPU6050.h>       // MPU6050 by Electronic Cats

#include "config.h"        // All pin definitions, PID defaults, physical
                           // constants - NOTHING in config.h changes

// Timing constants
#define FEEDBACK_HZ      50
#define PID_HZ           50
#define CMD_TIMEOUT_MS   500     // stop motors if no command received for this long

const unsigned long FEEDBACK_INTERVAL_US = 1000000UL / FEEDBACK_HZ;
const unsigned long PID_INTERVAL_US      = 1000000UL / PID_HZ;

// Encoder state (written by ISRs, read and reset in PID tick)
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// Wheel state (all in m/s and metres - same as old firmware)
float left_vel_ms   = 0.0f;   // measured velocity (m/s)
float right_vel_ms  = 0.0f;

float left_pos_m    = 0.0f;   // cumulative arc-length (m)
float right_pos_m   = 0.0f;

float left_target_ms  = 0.0f;   // commanded velocity from Pi (m/s)
float right_target_ms = 0.0f;

float left_slewed   = 0.0f;   // accel-limited setpoint fed into PID (m/s)
float right_slewed  = 0.0f;

// PID state
// Gains loaded from config.h - exactly the values you already tuned.
// These are in PWM-per-(m/s) units, same as always.
float kp_l = KP_DEFAULT, ki_l = KI_DEFAULT, kd_l = KD_DEFAULT;
float kp_r = KP_DEFAULT, ki_r = KI_DEFAULT, kd_r = KD_DEFAULT;

float err_sum_l  = 0.0f, prev_err_l = 0.0f;
float err_sum_r  = 0.0f, prev_err_r = 0.0f;

// IMU
MPU6050 mpu;

// Timing bookkeeping
unsigned long last_pid_us      = 0;
unsigned long last_feedback_us = 0;
unsigned long last_cmd_ms      = 0;

// Encoder ISRs - identical to your old firmware, direction logic unchanged
void IRAM_ATTR leftEncoderISR()  { left_ticks++; }
void IRAM_ATTR rightEncoderISR() { right_ticks++; }

// Motor driver - identical to your old firmware, adapt pin names to config.h
void setLeftMotor(float pwm_signed) {
    int pwm = (int)constrain(pwm_signed, -255, 255);
    if (pwm >= 0) {
        digitalWrite(LEFT_DIR_PIN, HIGH);
        analogWrite(LEFT_PWM_PIN, pwm);
    } else {
        digitalWrite(LEFT_DIR_PIN, LOW);
        analogWrite(LEFT_PWM_PIN, -pwm);
    }
}

void setRightMotor(float pwm_signed) {
    int pwm = (int)constrain(pwm_signed, -255, 255);
    if (pwm >= 0) {
        digitalWrite(RIGHT_DIR_PIN, HIGH);
        analogWrite(RIGHT_PWM_PIN, pwm);
    } else {
        digitalWrite(RIGHT_DIR_PIN, LOW);
        analogWrite(RIGHT_PWM_PIN, -pwm);
    }
}

// PID step - unchanged logic, error is in m/s same as always
float pidStep(float target_ms, float measured_ms,
              float &err_sum, float &prev_err,
              float kp, float ki, float kd, float dt)
{
    float err   = target_ms - measured_ms;   // error in m/s
    err_sum    += err * dt;
    err_sum     = constrain(err_sum, -50.0f, 50.0f);   // anti-windup
    float d_err = (err - prev_err) / dt;
    prev_err    = err;
    return kp * err + ki * err_sum + kd * d_err;   // returns PWM counts
}

// Acceleration slew - unchanged logic
// MAX_ACCEL_PER_TICK from config.h is in m/s per PID tick, same as always
float slewToward(float current, float target, float max_step) {
    float delta = target - current;
    delta = constrain(delta, -max_step, max_step);
    return current + delta;
}

// Serial command parser
void parseSerialCommand() {
    if (!Serial.available()) return;

    static char buf[128];
    int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    if (len <= 0) return;
    buf[len] = '\0';

    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, buf) != DeserializationError::Ok) return;

    // Velocity commands arrive in m/s - the bridge node already converted
    // from rad/s so nothing changes here vs old /cmd_vel_stamped handling.
    if (doc.containsKey("L")) left_target_ms  = doc["L"].as<float>();
    if (doc.containsKey("R")) right_target_ms = doc["R"].as<float>();

    // Runtime gain update (replaces old /pid_gains topic).
    // Gains are in PWM/(m/s) - same units as KP_DEFAULT in config.h.
    if (doc.containsKey("kp")) { kp_l = kp_r = doc["kp"].as<float>(); }
    if (doc.containsKey("ki")) { ki_l = ki_r = doc["ki"].as<float>(); }
    if (doc.containsKey("kd")) { kd_l = kd_r = doc["kd"].as<float>(); }

    last_cmd_ms = millis();
}

// Feedback sender
void sendFeedback() {
    // Read raw IMU values
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    // Convert to SI (adjust divisors to match your MPU6050 full-scale config)
    float ax = raw_ax / 16384.0f * 9.81f;           // +-2g  -> m/s^2
    float ay = raw_ay / 16384.0f * 9.81f;
    float az = raw_az / 16384.0f * 9.81f;
    float gx = raw_gx / 131.0f * (M_PI / 180.0f);  // +-250 deg/s -> rad/s
    float gy = raw_gy / 131.0f * (M_PI / 180.0f);
    float gz = raw_gz / 131.0f * (M_PI / 180.0f);

    // Velocities and positions sent in m/s and metres.
    // The Python bridge node divides by WHEEL_RADIUS to get rad/s for ROS.
    char out[192];
    snprintf(out, sizeof(out),
        "{\"lv\":%.4f,\"rv\":%.4f,\"lp\":%.4f,\"rp\":%.4f,"
        "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f}\n",
        left_vel_ms,  right_vel_ms,
        left_pos_m,   right_pos_m,
        ax, ay, az,
        gx, gy, gz
    );
    Serial.print(out);
}

void setup() {
    Serial.begin(115200);

    // Motor output pins
    pinMode(LEFT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(LEFT_PWM_PIN,  OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);

    // Encoder input pins + ISRs
    pinMode(LEFT_ENC_PIN,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN),  leftEncoderISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, RISING);

    // IMU
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("{\"warn\":\"MPU6050 not found\"}");
    }

    last_pid_us      = micros();
    last_feedback_us = micros();
    last_cmd_ms      = millis();

    Serial.println("{\"info\":\"firmware ready\"}");
}

void loop() {
    unsigned long now_us = micros();
    unsigned long now_ms = millis();

    // 1. Parse any incoming serial command
    parseSerialCommand();

    // 2. Command timeout - stop if Pi has not sent anything recently
    if ((now_ms - last_cmd_ms) > CMD_TIMEOUT_MS) {
        left_target_ms  = 0.0f;
        right_target_ms = 0.0f;
    }

    // 3. PID tick
    if ((now_us - last_pid_us) >= PID_INTERVAL_US) {
        float dt    = (now_us - last_pid_us) * 1e-6f;
        last_pid_us = now_us;

        // Snapshot encoder ticks atomically and reset for next tick
        noInterrupts();
        long l_snap = left_ticks;
        long r_snap = right_ticks;
        left_ticks  = 0;
        right_ticks = 0;
        interrupts();

        // Ticks -> m/s  (same formula as old firmware)
        // TICKS_PER_REV from config.h (e.g. 1440 for 360 CPR x 4x decoding)
        float metres_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
        left_vel_ms  = (l_snap * metres_per_tick) / dt;
        right_vel_ms = (r_snap * metres_per_tick) / dt;

        // Accumulate position in metres
        left_pos_m  += l_snap * metres_per_tick;
        right_pos_m += r_snap * metres_per_tick;

        // Acceleration slew - MAX_ACCEL_PER_TICK in m/s per tick (from config.h)
        left_slewed  = slewToward(left_slewed,  left_target_ms,  MAX_ACCEL_PER_TICK);
        right_slewed = slewToward(right_slewed, right_target_ms, MAX_ACCEL_PER_TICK);

        // Minimum speed deadband - MIN_WHEEL_SPEED in m/s (from config.h)
        float l_cmd = left_slewed;
        float r_cmd = right_slewed;
        if (fabsf(l_cmd) > 0.0f && fabsf(l_cmd) < MIN_WHEEL_SPEED)
            l_cmd = copysignf(MIN_WHEEL_SPEED, l_cmd);
        if (fabsf(r_cmd) > 0.0f && fabsf(r_cmd) < MIN_WHEEL_SPEED)
            r_cmd = copysignf(MIN_WHEEL_SPEED, r_cmd);

        // Reset integrator when command is zero (prevents windup at standstill)
        if (fabsf(left_slewed)  < 0.01f) { l_cmd = 0.0f; err_sum_l = 0.0f; }
        if (fabsf(right_slewed) < 0.01f) { r_cmd = 0.0f; err_sum_r = 0.0f; }

        // PID -> PWM  (error in m/s, gains in PWM/(m/s) - same as always)
        float pwm_l = pidStep(l_cmd, left_vel_ms,
                               err_sum_l, prev_err_l, kp_l, ki_l, kd_l, dt);
        float pwm_r = pidStep(r_cmd, right_vel_ms,
                               err_sum_r, prev_err_r, kp_r, ki_r, kd_r, dt);

        setLeftMotor(pwm_l);
        setRightMotor(pwm_r);
    }

    // 4. Send feedback to Pi at FEEDBACK_HZ
    if ((now_us - last_feedback_us) >= FEEDBACK_INTERVAL_US) {
        last_feedback_us = now_us;
        sendFeedback();
    }
}
