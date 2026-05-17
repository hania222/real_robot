/*
    
 *
 * Serial protocol (115200 baud, newline-terminated JSON):
 *
 *   Pi → ESP32  (unchanged):
 *     {"L": 0.150}                          ← left  wheel target m/s
 *     {"R": -0.150}                         ← right wheel target m/s
 *     {"kp": 80.0, "ki": 5.0, "kd": 1.0}   ← runtime gain update
 *
 *   ESP32 → Pi  (feedback at FEEDBACK_HZ):
 *     {
 *       "lv": 0.148,   ← actual left  velocity  (m/s)
 *       "rv":-0.149,   ← actual right velocity  (m/s)
 *       "lp": 0.031,   ← cumulative left  arc-length (m)
 *       "rp":-0.031,   ← cumulative right arc-length (m)
 *       "lt": 0.150,   ← NEW: slewed left  target fed into PID (m/s)
 *       "rt":-0.150,   ← NEW: slewed right target fed into PID (m/s)
 *       "el": 0.002,   ← NEW: left  PID error = lt - lv (m/s)
 *       "er":-0.001,   ← NEW: right PID error = rt - rv (m/s)
 *       "pl":127.0,    ← NEW: left  PWM output sent to motor driver
 *       "pr":-128.0,   ← NEW: right PWM output sent to motor driver
 *       "ax": 0.01, "ay": 0.00, "az": 9.81,
 *       "gx": 0.00, "gy": 0.00, "gz": 0.12
 *     }
 *
 * The Python bridge maps these to /pid_debug as:
 *   data[0] = lt / WHEEL_RADIUS  (target left  rad/s)
 *   data[1] = lv / WHEEL_RADIUS  (actual left  rad/s)
 *   data[2] = el / WHEEL_RADIUS  (error  left  rad/s)
 *   data[3] = pl                 (pwm    left)
 *   data[4] = rt / WHEEL_RADIUS  (target right rad/s)
 *   data[5] = rv / WHEEL_RADIUS  (actual right rad/s)
 *   data[6] = er / WHEEL_RADIUS  (error  right rad/s)
 *   data[7] = pr                 (pwm    right)
 *
 * ============================================================
 */

#include <Arduino.h>
#include <ArduinoJson.h>   
#include <Wire.h>
#include <MPU6050.h>       // MPU6050 by Electronic Cats

#include "config.h"        // Pin definitions, PID defaults, physical constants

// ── Timing constants ──────────────────────────────────────────────────────────
#define FEEDBACK_HZ      50
#define PID_HZ           50
#define CMD_TIMEOUT_MS   500     // stop motors if no command received for this long

const unsigned long FEEDBACK_INTERVAL_US = 1000000UL / FEEDBACK_HZ;
const unsigned long PID_INTERVAL_US      = 1000000UL / PID_HZ;

// ── Encoder state (written by ISRs, read and reset in PID tick) ───────────────
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// ── Wheel state (all in m/s and metres) ──────────────────────────────────────
float left_vel_ms    = 0.0f;   // measured actual velocity (m/s)
float right_vel_ms   = 0.0f;

float left_pos_m     = 0.0f;   // cumulative arc-length (m)
float right_pos_m    = 0.0f;

float left_target_ms  = 0.0f;  // commanded velocity from Pi (m/s)
float right_target_ms = 0.0f;

float left_slewed    = 0.0f;   // accel-limited setpoint fed into PID (m/s)
float right_slewed   = 0.0f;

// ── PID debug state (populated each PID tick, sent in feedback) ───────────────
float left_pwm_out   = 0.0f;   // last PWM written to left  motor
float right_pwm_out  = 0.0f;   // last PWM written to right motor
float left_error     = 0.0f;   // last PID error left  (m/s)
float right_error    = 0.0f;   // last PID error right (m/s)

// ── PID gains (loaded from config.h, updatable at runtime via serial) ─────────
float kp_l = KP_DEFAULT, ki_l = KI_DEFAULT, kd_l = KD_DEFAULT;
float kp_r = KP_DEFAULT, ki_r = KI_DEFAULT, kd_r = KD_DEFAULT;

float err_sum_l  = 0.0f, prev_err_l = 0.0f;
float err_sum_r  = 0.0f, prev_err_r = 0.0f;

// ── IMU ───────────────────────────────────────────────────────────────────────
MPU6050 mpu;

// ── Timing bookkeeping ────────────────────────────────────────────────────────
unsigned long last_pid_us      = 0;
unsigned long last_feedback_us = 0;
unsigned long last_cmd_ms      = 0;

// ── Encoder ISRs ──────────────────────────────────────────────────────────────
void IRAM_ATTR leftEncoderISR()  { left_ticks++; }
void IRAM_ATTR rightEncoderISR() { right_ticks++; }

// ── Motor driver ──────────────────────────────────────────────────────────────
void setLeftMotor(float pwm_signed) {
    int pwm = (int)constrain(pwm_signed, -255, 255);
    if (pwm >= 0) {
        digitalWrite(LEFT_DIR_PIN, HIGH);
        analogWrite(LEFT_PWM_PIN,  pwm);
    } else {
        digitalWrite(LEFT_DIR_PIN, LOW);
        analogWrite(LEFT_PWM_PIN, -pwm);
    }
}

void setRightMotor(float pwm_signed) {
    int pwm = (int)constrain(pwm_signed, -255, 255);
    if (pwm >= 0) {
        digitalWrite(RIGHT_DIR_PIN, HIGH);
        analogWrite(RIGHT_PWM_PIN,  pwm);
    } else {
        digitalWrite(RIGHT_DIR_PIN, LOW);
        analogWrite(RIGHT_PWM_PIN, -pwm);
    }
}

// ── PID step ──────────────────────────────────────────────────────────────────
// Returns PWM output.  Error, integral, and derivative all in m/s.
float pidStep(float target_ms, float measured_ms,
              float &err_sum, float &prev_err,
              float kp, float ki, float kd, float dt,
              float &err_out)   // ← NEW: pass error back to caller for debug
{
    float err   = target_ms - measured_ms;
    err_out     = err;                              // store for feedback packet
    err_sum    += err * dt;
    err_sum     = constrain(err_sum, -50.0f, 50.0f);  // anti-windup
    float d_err = (err - prev_err) / dt;
    prev_err    = err;
    return kp * err + ki * err_sum + kd * d_err;   // PWM counts
}

// ── Acceleration slew ─────────────────────────────────────────────────────────
float slewToward(float current, float target, float max_step) {
    float delta = constrain(target - current, -max_step, max_step);
    return current + delta;
}

// ── Serial command parser ─────────────────────────────────────────────────────
void parseSerialCommand() {
    if (!Serial.available()) return;

    static char buf[128];
    int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    if (len <= 0) return;
    buf[len] = '\0';

    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, buf) != DeserializationError::Ok) return;

    // Wheel velocity commands (m/s — bridge already converted from rad/s)
    if (doc.containsKey("L")) left_target_ms  = doc["L"].as<float>();
    if (doc.containsKey("R")) right_target_ms = doc["R"].as<float>();

    // Runtime gain update — same keys as config.h defaults
    if (doc.containsKey("kp")) { kp_l = kp_r = doc["kp"].as<float>(); }
    if (doc.containsKey("ki")) { ki_l = ki_r = doc["ki"].as<float>(); }
    if (doc.containsKey("kd")) { kd_l = kd_r = doc["kd"].as<float>(); }

    last_cmd_ms = millis();
}

// ── Feedback sender ───────────────────────────────────────────────────────────
void sendFeedback() {
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    float ax = raw_ax / 16384.0f * 9.81f;
    float ay = raw_ay / 16384.0f * 9.81f;
    float az = raw_az / 16384.0f * 9.81f;
    float gx = raw_gx / 131.0f * (M_PI / 180.0f);
    float gy = raw_gy / 131.0f * (M_PI / 180.0f);
    float gz = raw_gz / 131.0f * (M_PI / 180.0f);

    // Buffer size increased for the five extra fields (lt, rt, el, er, pl, pr)
    char out[256];
    snprintf(out, sizeof(out),
        "{"
        "\"lv\":%.4f,\"rv\":%.4f,"
        "\"lp\":%.4f,\"rp\":%.4f,"
        "\"lt\":%.4f,\"rt\":%.4f,"   // slewed targets (m/s) — NEW
        "\"el\":%.4f,\"er\":%.4f,"   // PID errors     (m/s) — NEW
        "\"pl\":%.1f,\"pr\":%.1f,"   // PWM outputs          — NEW
        "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f"
        "}\n",
        left_vel_ms,  right_vel_ms,
        left_pos_m,   right_pos_m,
        left_slewed,  right_slewed,   // NEW
        left_error,   right_error,    // NEW
        left_pwm_out, right_pwm_out,  // NEW
        ax, ay, az,
        gx, gy, gz
    );
    Serial.print(out);
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(LEFT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(LEFT_PWM_PIN,  OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);

    pinMode(LEFT_ENC_PIN,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN),  leftEncoderISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, RISING);

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

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now_us = micros();
    unsigned long now_ms = millis();

    // 1. Parse incoming serial commands
    parseSerialCommand();

    // 2. Command timeout — stop wheels if Pi goes silent
    if ((now_ms - last_cmd_ms) > CMD_TIMEOUT_MS) {
        left_target_ms  = 0.0f;
        right_target_ms = 0.0f;
    }

    // 3. PID tick at PID_HZ
    if ((now_us - last_pid_us) >= PID_INTERVAL_US) {
        float dt    = (now_us - last_pid_us) * 1e-6f;
        last_pid_us = now_us;

        // Atomically snapshot and reset encoder counts
        noInterrupts();
        long l_snap = left_ticks;
        long r_snap = right_ticks;
        left_ticks  = 0;
        right_ticks = 0;
        interrupts();

        // Ticks → m/s
        float metres_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
        left_vel_ms  = (l_snap * metres_per_tick) / dt;
        right_vel_ms = (r_snap * metres_per_tick) / dt;

        // Accumulate arc-length
        left_pos_m  += l_snap * metres_per_tick;
        right_pos_m += r_snap * metres_per_tick;

        // Acceleration slew
        left_slewed  = slewToward(left_slewed,  left_target_ms,  MAX_ACCEL_PER_TICK);
        right_slewed = slewToward(right_slewed, right_target_ms, MAX_ACCEL_PER_TICK);

        // Minimum speed deadband
        float l_cmd = left_slewed;
        float r_cmd = right_slewed;
        if (fabsf(l_cmd) > 0.0f && fabsf(l_cmd) < MIN_WHEEL_SPEED)
            l_cmd = copysignf(MIN_WHEEL_SPEED, l_cmd);
        if (fabsf(r_cmd) > 0.0f && fabsf(r_cmd) < MIN_WHEEL_SPEED)
            r_cmd = copysignf(MIN_WHEEL_SPEED, r_cmd);

        // Reset integrator at standstill
        if (fabsf(left_slewed)  < 0.01f) { l_cmd = 0.0f; err_sum_l = 0.0f; }
        if (fabsf(right_slewed) < 0.01f) { r_cmd = 0.0f; err_sum_r = 0.0f; }

        // PID → PWM  (error units: m/s, gains units: PWM per (m/s))
        float pwm_l = pidStep(l_cmd, left_vel_ms,
                               err_sum_l, prev_err_l,
                               kp_l, ki_l, kd_l, dt,
                               left_error);   // ← captures error for feedback

        float pwm_r = pidStep(r_cmd, right_vel_ms,
                               err_sum_r, prev_err_r,
                               kp_r, ki_r, kd_r, dt,
                               right_error);  // ← captures error for feedback

        // Store PWM for feedback packet before clamping changes the value
        left_pwm_out  = constrain(pwm_l, -255.0f, 255.0f);
        right_pwm_out = constrain(pwm_r, -255.0f, 255.0f);

        setLeftMotor(pwm_l);
        setRightMotor(pwm_r);
    }

    // 4. Send feedback at FEEDBACK_HZ
    if ((now_us - last_feedback_us) >= FEEDBACK_INTERVAL_US) {
        last_feedback_us = now_us;
        sendFeedback();
    }
}