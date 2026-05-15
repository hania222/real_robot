//  Real Robot ESP32 Firmware
//  micro-ROS Jazzy | Arduino IDE 2.x | ESP32 core 2.0.2
//
//  Publishes:  /odom        (nav_msgs/Odometry)
//              /imu/data    (sensor_msgs/Imu)
//              /joint_states (sensor_msgs/JointState)
//              /pid_debug   (std_msgs/Float32MultiArray)
//                           [target_L, actual_L, error_L, pwm_L,
//                            target_R, actual_R, error_R, pwm_R]
//  Subscribes: /cmd_vel_stamped  (geometry_msgs/TwistStamped)
//              /pid_gains        (std_msgs/String)
//                           format: "kp_l:1.0 ki_l:0.0 kd_l:0.0
//                                    kp_r:1.0 ki_r:0.0 kd_r:0.0"
//  closed loop control: cmd--> target speed
//                       encoders-->actual speed
//                       PID-->computes PWm 
//
//  Motor protection layers (all three active simultaneously):
//    Main 4:
//    1. Stall detection  —> cuts power if PWM is near-max but wheel stays
//                          stationary for STALL_TIME_MS ms. prevents winding
//                          burn when robot is blocked by an obstacle or stuck
//    2. Slew rate limiter —>limits how fast the target speed can change each
//                           tick (MAX_ACCEL_PER_TICK), to eliminate inrush
//                           current spikes and gearbox jerk on sudden commands
//    3. PWM dead-band    —> forces PWM to 0 below PWM_DEADBAND counts. prevents
//                           sustained low-voltage heating when the motor cannot
//                           overcome gearbox static friction
//    4.command timeout   —> if no new cmd_vel after CMD_TIMEOUT_MS ms,(if ROS crashes ,USB disconnects, Pi freezes,nav2 died, wifi drops),
//                                stop motor , reset PID targets and integral, reset stall state
//    some smaller stability protections:
//            1. PID inetgral anti-windup
//            2. derivative spike prevention
//            3. safe startup
//            4. encoder sanity check —> skips any tick where encoder reports a physically impossible speed (caused byelectrical noise from motor PWM switching)
//                  
//


#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <std_msgs/msg/float32_multi_array.h>  // for /pid_debug topic
#include <std_msgs/msg/string.h>               // for /pid_gains topic

#include <Wire.h>
#include <MPU6050.h>

#include "config.h"

// micro-ROS variables
rcl_node_t            node;       //ros2 node object (equivalent to rclpy.create_node())
rcl_allocator_t       allocator; // for dynamic memory management in ROS 2
rclc_support_t        support;  //for init and shutdown of ROS context
rclc_executor_t       executor;//run callbacks + timers +subscriptions (equivalent to rclpy.spin(node))

// Publishers and subscriber from micro-ROS node
rcl_publisher_t       odom_pub;
rcl_publisher_t       imu_pub;
rcl_publisher_t       joint_pub;
rcl_publisher_t       debug_pub;   // publishes PID internals so PlotJuggler can graph them
rcl_subscription_t    cmd_sub;
rcl_subscription_t    gains_sub;   // receives new PID gains at runtime

rcl_timer_t           odom_timer; //runs odemtry calculations and publishes @ 20 Hz(periodically)
rcl_timer_t           imu_timer; //runs IMU reading and publishes @ 50 Hz (periodically)
                                //note that: ekf publishes @ 30Hz rate

// Message variables
nav_msgs__msg__Odometry           odom_msg;
sensor_msgs__msg__Imu             imu_msg;
sensor_msgs__msg__JointState      joint_msg;
geometry_msgs__msg__TwistStamped  cmd_msg;

// debug_msg holds 8 floats sent to /pid_debug every odom tick so
// PlotJuggler can show target vs actual speed for both wheels live
std_msgs__msg__Float32MultiArray  debug_msg;
float                             debug_data[8];

//gains_msg receives plain text like "kp_l:1.5 ki_l:0.1 kd_l:0.0 ..."
// from Topic Publisher so can retune without reflashing
std_msgs__msg__String             gains_msg;
char                              gains_buf[128];

//Encoder variables (volatile bc they are modified in ISR) 
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// robot position variables 
float pos_x   = 0.0f;
float pos_y   = 0.0f;
float heading = 0.0f;   // yaw angle (radians)

//previous encoder ticks for odometry calculation
long  last_left_ticks  = 0; 
long  last_right_ticks = 0;

//Motor command state
float cmd_linear  = 0.0f;       //forward/backward speed in m/s
float cmd_angular = 0.0f;      //rotational speed in rad/s
unsigned long last_cmd_ms = 0;//last time cmd_vel recieved

//PID target wheel velocities (set in cmd_vel_callback, used in odom_timer)
float target_v_left  = 0.0f;
float target_v_right = 0.0f;

// Slew-rate-limited versions of the target velocities,
static float slewed_left  = 0.0f;
static float slewed_right = 0.0f;

// PID state variables 
// integral accumulates error over time (fixes steady-state offset)
// prev_error is needed to compute the derivative (rate of change of error)
float pid_integral_left    = 0.0f;
float pid_integral_right   = 0.0f;
float pid_prev_error_left  = 0.0f;
float pid_prev_error_right = 0.0f;

// First-call guard: on the very first odom tick prev_error is 0, so the error becomes very large and causes a big derivative spike that can jerk the robot before it starts moving
//Setting prev_error to the first measured error on the first tick eliminates that PWM spike,The flag is set true here and cleared after the first tick in setup() so the derivative term is primed with the real error from the start of active control
bool pid_first_call = true;

//Live-tunable PID gains 
//Initialised from config.h but can be updated at runtime via /pid_gains topic
volatile float live_kp_left  = KP_LEFT;
volatile float live_ki_left  = KI_LEFT;
volatile float live_kd_left  = KD_LEFT;
volatile float live_kp_right = KP_RIGHT;
volatile float live_ki_right = KI_RIGHT;
volatile float live_kd_right = KD_RIGHT;

//IMU object (represnt the sensor)
MPU6050 mpu;

//Joint state arrays (raw data of joint positions) (2 wheels)
// js_positions accumulates indefinitely (never wraps) ,robot_state_publisher
double  js_positions[2] = {0.0, 0.0};
double  js_velocities[2] = {0.0, 0.0};
rosidl_runtime_c__String js_names[2]; //ros string type for joint names
char    js_name0[] = "left_drive";
char    js_name1[] = "right_drive";

// Stall detection state
static unsigned long stall_start_ms = 0;


//X4 Encoder ISRs 
//  ESP32 has two types of RAMS (Flash(slow) and IRAM(fast)) and ISRs must be in IRAM to run fast and correctly.
static inline void IRAM_ATTR update_left() {
    bool a = digitalRead(LEFT_ENC_A);
    bool b = digitalRead(LEFT_ENC_B);
    if (a == b)
        left_ticks++;   // increment tick count when moving forward
    else
        left_ticks--;   // decrement tick counts when moving backward
}

static inline void IRAM_ATTR update_right() {
    bool a = digitalRead(RIGHT_ENC_A);
    bool b = digitalRead(RIGHT_ENC_B);
    if (a == b)
        right_ticks++; 
    else
        right_ticks--;  
}

// Four ISR entry points, A and B edges for each wheel
void IRAM_ATTR left_enc_isr_A()  { update_left();  }
void IRAM_ATTR left_enc_isr_B()  { update_left();  }
void IRAM_ATTR right_enc_isr_A() { update_right(); }
void IRAM_ATTR right_enc_isr_B() { update_right(); }


//  Motor control , takes speed values: -255 to +255
void set_motor_left(int speed) {
    if (speed >= 0) {
        //forward and set reverse PWM to 0
        analogWrite(L_RPWM, speed);
        analogWrite(L_LPWM, 0);
    } else {
        analogWrite(L_RPWM, 0);
        analogWrite(L_LPWM, -speed);
    }
}

void set_motor_right(int speed) {
    if (speed >= 0) {
        analogWrite(R_RPWM, speed);
        analogWrite(R_LPWM, 0);
    } else {
        analogWrite(R_RPWM, 0);
        analogWrite(R_LPWM, -speed);
    }
}

void stop_motors() {
    analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
    analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0);
}

// Slew rate limiter
float slew(float current, float desired, float max_step) {
    float diff = desired - current;
    if (fabsf(diff) <= max_step) return desired; 
    return current + (diff > 0.0f ? max_step : -max_step); //if diff is positive:add max_step(accelerate) ,otherwise:subtract max_step(decelerate)
}

//PWM dead-band filter 
int apply_deadband(float pwm) {
    if (fabsf(pwm) < (float)PWM_DEADBAND) return 0;
    return (int)pwm;
}

//  PID computation — >called once per wheel per odom tick
//  Returns a PWM value in the range [PID_MIN_OUTPUT, PID_MAX_OUTPUT]
float compute_pid(float target, float actual, float kp, float ki, float kd, float &integral, float &prev_error, float dt, bool first_call) {
    float error = target - actual;         
    // First-call guard: skip derivative on the very first tick so prev_error=0 does not cause a large derivative spike before the motors have moved
    // After the first tick prev_error is primed with the real error and the derivative term works normally from tick 2
    if (first_call) {
        prev_error = error;
    }

    integral        += error * dt;               // accumulate over time
    float derivative = (error - prev_error) / dt; // rate of change of error
    prev_error       = error;

    // Anti-windup:prevents integral from growing forever, if robot is stuck (target is high, actual=0)
    if (ki > 1e-6f) { //avoid dividing by zero
        integral = constrain(integral, PID_MIN_OUTPUT / ki, PID_MAX_OUTPUT / ki); //since (integral * ki <= PID_MAX_OUTPUT)
    } else {
        integral = 0.0f;                                                         // no point accumulating if Ki is off
    }

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    return constrain(output, PID_MIN_OUTPUT, PID_MAX_OUTPUT);
}

//  /pid_gains callback (runs when ros send new PID)
//  Receives a plain string from Topic Publisher e.g:"kp_l:1.5 ki_l:0.1 kd_l:0.0 kp_r:1.5 ki_r:0.1 kd_r:0.0"
//  Updates live gains immediately without reflashing the esp32
void gains_callback(const void * msg_in) {
    const std_msgs__msg__String * m =
        (const std_msgs__msg__String *)msg_in;

    sscanf(m->data.data, "kp_l:%f ki_l:%f kd_l:%f kp_r:%f ki_r:%f kd_r:%f",
           (float*)&live_kp_left,  (float*)&live_ki_left,  (float*)&live_kd_left,
           (float*)&live_kp_right, (float*)&live_ki_right, (float*)&live_kd_right);

    // reset integrals when gains change so old wind-up doesn't carry over
    pid_integral_left  = 0.0f;
    pid_integral_right = 0.0f;

    // reset first-call guard so the derivative term is re-primed with the current error on the next tick to avoida derivative spike after retuning
    pid_first_call = true;
}


//cmd_vel callback
void cmd_vel_callback(const void * msg_in) {
    //convert raw pointer to ROS message type
    const geometry_msgs__msg__TwistStamped * msg =
        (const geometry_msgs__msg__TwistStamped *)msg_in;

    cmd_linear  = msg->twist.linear.x;   //read forward speed from msg
    cmd_angular = msg->twist.angular.z; //read rotational speed from msg
    last_cmd_ms = millis();            //store current time

    // differential drive velocity -> wheel speeds (m/s)
    target_v_left  = cmd_linear - (cmd_angular * WHEEL_BASE / 2.0f);
    target_v_right = cmd_linear + (cmd_angular * WHEEL_BASE / 2.0f);
}

//  Odometry timer callback (runs at 20 Hz)
void odom_timer_callback(rcl_timer_t * timer, int64_t /*last_call*/) {
    if (timer == NULL) return;
    // Stop motors if cmd_vel timeout
    if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
        stop_motors();
        cmd_linear       = 0.0f;
        cmd_angular      = 0.0f;
        //clear PID targets so loop doesn't keepdriving
        target_v_left    = 0.0f;   
        target_v_right   = 0.0f;
        // reset slew state so the next command starts from 0
        slewed_left      = 0.0f;
        slewed_right     = 0.0f;
        // reset integrals so there's no wind-up on restart
        pid_integral_left  = 0.0f; 
        pid_integral_right = 0.0f;
        // re-prime derivative on next active command to avoid spike from accumulated error
        pid_first_call     = true; 
        stall_start_ms     = 0;    // clear stall timer (BC robot is intentionally stopped)
    }

    // read encoder ticks (a copy)
    noInterrupts(); //stop ISR temporarily so I can copy encoder values without them changing mid-read (which would cause incorrect odometry calculations)
    long cur_left  = left_ticks;
    long cur_right = right_ticks;
    interrupts(); //resume execution of ISRs

    //encoder difference(delta): how much each encoder moved since last time we checked (in ticks)
    long d_left  = cur_left  - last_left_ticks;
    long d_right = cur_right - last_right_ticks;
    last_left_ticks  = cur_left;
    last_right_ticks = cur_right;

    // Convert ticks -> metres
    float dist_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
    float dl = d_left  * dist_per_tick;
    float dr = d_right * dist_per_tick;

    // calculate velocity, dt in seconds
    float dt = ODOM_PUBLISH_MS / 1000.0f;

    // Actual wheel velocities derived from encoder ticks this period (m/s)
    float actual_v_left  = dl / dt;
    float actual_v_right = dr / dt;

    //encoder sanity checkup, bc the PID would react to a fake 50 m/s error and send full PWM instantly, causing a violent jerk
    if (fabsf(actual_v_left)  > MAX_PHYSICAL_SPEED ||
        fabsf(actual_v_right) > MAX_PHYSICAL_SPEED) {
        // Reset tick baseline so next tick compares correctly from here
        last_left_ticks  = cur_left;
        last_right_ticks = cur_right;
        return;
    }

    // PID motor drive
    float pwm_left  = 0.0f;
    float pwm_right = 0.0f;

    if (millis() - last_cmd_ms <= CMD_TIMEOUT_MS) {

        // Slew rate limiter 
        //move the current wheel target gradually toward the desired target instead of jumping instantly
        slewed_left  = slew(slewed_left,  target_v_left,  MAX_ACCEL_PER_TICK);
        slewed_right = slew(slewed_right, target_v_right, MAX_ACCEL_PER_TICK);

        // compute_pid returns a PWM value [-255, +255] 
        pwm_left = compute_pid(
            slewed_left,  actual_v_left,
            live_kp_left,  live_ki_left,  live_kd_left,
            pid_integral_left,  pid_prev_error_left,  dt, pid_first_call);

        pwm_right = compute_pid(
            slewed_right, actual_v_right,
            live_kp_right, live_ki_right, live_kd_right,
            pid_integral_right, pid_prev_error_right, dt, pid_first_call);

        // Clear first-call flag after both wheels have been primed this tick
        pid_first_call = false;

        // Stall detection
        bool stalled =
            (fabsf(pwm_left)  > STALL_PWM_THRESHOLD &&
             fabsf(actual_v_left)  < STALL_SPEED_THRESHOLD) ||
            (fabsf(pwm_right) > STALL_PWM_THRESHOLD &&
             fabsf(actual_v_right) < STALL_SPEED_THRESHOLD);

        if (stalled) {
            if (stall_start_ms == 0) {
                stall_start_ms = millis(); // start timing the stall
            }
            if (millis() - stall_start_ms > STALL_TIME_MS) {
                //if Stall persisted too long 
                stop_motors();
                target_v_left    = 0.0f;
                target_v_right   = 0.0f;
                slewed_left      = 0.0f;
                slewed_right     = 0.0f;
                pid_integral_left  = 0.0f;
                pid_integral_right = 0.0f;
                pid_first_call     = true;
                pwm_left  = 0.0f;
                pwm_right = 0.0f;
                return; 
            }
        } else {
            stall_start_ms = 0; // wheel is moving normally, reset timer
        }

        //PWM dead-band 
        set_motor_left(apply_deadband(pwm_left));
        set_motor_right(apply_deadband(pwm_right));
    }

    // Publish /pid_debug so PlotJuggler can graph it 
    // Note: debug_data[0]/[4] show the slew-limited target, not the raw cmd,
    debug_data[0] = slewed_left;                              // slewed target (left)
    debug_data[1] = actual_v_left;           
    debug_data[2] = slewed_left - actual_v_left;            // error (left)
    debug_data[3] = pwm_left;                              //PWM the PID output (left)  
    debug_data[4] = slewed_right;                         // slewed target (right)
    debug_data[5] = actual_v_right;          
    debug_data[6] = slewed_right - actual_v_right;       // error (right)
    debug_data[7] = pwm_right;                          // PWM the PID output (right)
    rcl_publish(&debug_pub, &debug_msg, NULL);

    //differential drive odometry
    float d_centre  = (dl + dr) / 2.0f;        //how far robot moved forward
    float d_heading = (dr - dl) / WHEEL_BASE; //how much robot rotated (radians)
    heading += d_heading;                    //yaw 

    //normalizing heading to [-π, π] to prevent sinf/cosf precision loss from floating-point drift accumulating over long runs
    heading = atan2f(sinf(heading), cosf(heading));

    pos_x   += d_centre * cosf(heading);  //cosine is the horizontal component of the movement
    pos_y   += d_centre * sinf(heading);  //sine is the vertical component of the movement

    float v_linear  = d_centre  / dt;
    float v_angular = d_heading / dt;

    // theta=L/r, Joint positions (radians) and velocities, needed with the urdf for robot_state_publisher to work in ROS 2    
    js_positions[0] += dl / WHEEL_RADIUS;
    js_positions[1] += dr / WHEEL_RADIUS;
    js_velocities[0] = actual_v_left  / WHEEL_RADIUS; // rad/s
    js_velocities[1] = actual_v_right / WHEEL_RADIUS; // rad/s

    // Timestamp , in ros2 time is reprsented as secs + nano secs 
    int64_t now_ns = rmw_uros_epoch_nanos(); //built in micro ros function
    int32_t sec    = (int32_t)(now_ns / 1000000000LL);
    uint32_t nsec  = (uint32_t)(now_ns % 1000000000LL);

    // Publish /odom 
    //attaching time to this message and fill in the position and velocity data
    odom_msg.header.stamp.sec     = sec;
    odom_msg.header.stamp.nanosec = nsec;
    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;

    // Heading -> quaternion (yaw only, 2D robot)
    //quaternion are 4 numbers (x, y, z, w)--> represent rotation in 3D space (Quaternion math internally represents rotation using: theta/2 not theta)
    odom_msg.pose.pose.orientation.z = sinf(heading / 2.0f);   //yaw rotation
    odom_msg.pose.pose.orientation.w = cosf(heading / 2.0f);  //scaler part
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;

    // Odometry covariance for EKF (only the diagonal entries the EKF)
    // Pose covariance (6×6 row-major): [x, y, z, roll, pitch, yaw]
    odom_msg.pose.covariance[0]  = 0.001f;  // x   ->72 000 CPR encoder is very accurate
    odom_msg.pose.covariance[7]  = 0.001f;  // y    —>same
    odom_msg.pose.covariance[35] = 0.05f;   // yaw  —>  642 mm wheelbase limits heading drift

    // Twist covariance (6×6 row-major): [vx, vy, vz, vroll, vpitch, vyaw]
    odom_msg.twist.covariance[0]  = 0.001f; // vx   — >high-res encoder velocity is precise
    odom_msg.twist.covariance[35] = 0.05f;  // vyaw — matches pose yaw confidence

    //publish the linear and angular velocity in the twist part of the odometry message (used by the EKF, it predicts robot motion using velocities)
    odom_msg.twist.twist.linear.x  = v_linear;
    odom_msg.twist.twist.angular.z = v_angular;

    rcl_publish(&odom_pub, &odom_msg, NULL);

    //Publish /joint_states 
    joint_msg.header.stamp.sec     = sec;
    joint_msg.header.stamp.nanosec = nsec;
    joint_msg.position.data        = js_positions;
    joint_msg.velocity.data        = js_velocities;

    rcl_publish(&joint_pub, &joint_msg, NULL);
}


//  IMU timer callback, runs @ 50 Hz 
void imu_timer_callback(rcl_timer_t * timer, int64_t /*last_call*/) {
    if (timer == NULL) return;

    int16_t ax, ay, az, gx, gy, gz; //raw values from IMU, acceleration in x,y,z(m/s²) and angular velocities(rad/sec)

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //read 6 values (3 acc+3 gyro)

    //convert raw values --> real physical units using the sensor's sensitivity scale factors
    // Accel: ±2g range → 16384 LSB/g → m/s²
    // Gyro:  ±250°/s   → 131 LSB/°/s → rad/s
    const float ACCEL_SCALE = 9.81f / 16384.0f;
    const float GYRO_SCALE  = (M_PI / 180.0f) / 131.0f;

    int64_t now_ns = rmw_uros_epoch_nanos(); //built in micro ros function, more precise than millis() and works well with ROS time which is in secs + nanosecs

    imu_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    imu_msg.linear_acceleration.x = ax * ACCEL_SCALE;
    imu_msg.linear_acceleration.y = ay * ACCEL_SCALE;
    imu_msg.linear_acceleration.z = az * ACCEL_SCALE;

    imu_msg.angular_velocity.x = gx * GYRO_SCALE;
    imu_msg.angular_velocity.y = gy * GYRO_SCALE;
    imu_msg.angular_velocity.z = gz * GYRO_SCALE;

    // No orientation estimate from MPU-6050 alone, set to identity
    imu_msg.orientation.w = 1.0f;
    imu_msg.orientation.x = 0.0f;
    imu_msg.orientation.y = 0.0f;
    imu_msg.orientation.z = 0.0f;

    // IMU covariance for EKF 
    imu_msg.orientation_covariance[0]      = -1.0f;  // -1 = orientation not provided

    // Only yaw rate (index 8) is fused by the EKF (imu0_config row 3, col 3).
    imu_msg.angular_velocity_covariance[8] =  0.005f; // yaw rate (rad/s)² — fused

    // linear_acceleration covariance (3×3 row-major): [ax, ay, az]
    //not fused by ekf but set to realistic value based on sensor noise, so the EKF knows how much to trust it if we decide to fuse it in the future by changing imu0_config
    imu_msg.linear_acceleration_covariance[0] =  0.01f; // ax 
    imu_msg.linear_acceleration_covariance[4] =  0.01f; // ay 
    imu_msg.linear_acceleration_covariance[8] =  0.01f; // az 

    rcl_publish(&imu_pub, &imu_msg, NULL);
}


//  setup(), is doing 5 main things:
// 1.connect ESP32 to micro-ROS agent via serial
// 2.initialize hardware (motors, encoders, IMU)
// 3.create micro-Ros node
// 4.create publishers + subscribers 
// 5.create timers + executor for callbacks(scheduler for when to run the code in the callbacks)

void setup() {

    //Serial for micro-ROS 
    Serial.begin(MICROROS_SERIAL_BAUD);
    set_microros_serial_transports(Serial);
    delay(2000);   // wait for agent to be ready

    //BTS7960 enable pins 
    pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
    pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
    pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
    pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

    //PWM pins
    pinMode(L_RPWM, OUTPUT);
    pinMode(L_LPWM, OUTPUT);
    pinMode(R_RPWM, OUTPUT);
    pinMode(R_LPWM, OUTPUT);
    stop_motors();  //safety start

    // Encoders 
    // X4 mode: all four encoder pins need INPUT_PULLUP and a CHANGE interrupt
    pinMode(LEFT_ENC_A,  INPUT_PULLUP);  //input_PULLUP : internal resitor (clean signal)
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);  
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);  

    // X4 CHANGE mode: triggers on both rising AND falling edges of BOTH channels
    //e.g: Whenever LEFT_ENC_A changes state(up/down), call left_enc_isr_A() 
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  left_enc_isr_A,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  left_enc_isr_B,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), right_enc_isr_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), right_enc_isr_B, CHANGE);

    // MPU-6050 
    Wire.begin(IMU_SDA, IMU_SCL);                       //Initialize I2C communication for the IMU
    mpu.initialize();                                  //Initialize the MPU-6050 sensor
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   //set the ranges (default)
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    //creating ros2 node named "esp32_robot_node" on ESP32 
    //API functions for creating ROS 2 nodes and publishers/subscribers,
    //micro-ros does not have dynamic memory alloaction so we need allocator and
    allocator = rcl_get_default_allocator();          //micro ros needs explicit memory handling
    rclc_support_init(&support, 0, NULL, &allocator); //initialize this ros2 runtime env on ESP32 using this memory allocator
    rclc_node_init_default(&node, "esp32_robot_node", "", &support);

    // publisher that will send Odometry messages on the /odom topic
    rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom");

    rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data");

    rclc_publisher_init_default(
        &joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states");

    // debug publisher, streams PID internals to /pid_debug at 20 Hz
    rclc_publisher_init_default(
        &debug_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pid_debug");

    // Subscribers
    // /cmd_vel_stamped for TwistStamped
    rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        "/cmd_vel_stamped");

    // gains subscriber
    rclc_subscription_init_default(
        &gains_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/pid_gains");

    // Timers = call functions automatically at a set frequency (periodically)
    // these run the main control loop and publish odometry and IMU data
    rclc_timer_init_default(
        &odom_timer, &support,
        RCL_MS_TO_NS(ODOM_PUBLISH_MS),
        odom_timer_callback);

    rclc_timer_init_default(
        &imu_timer, &support,
        RCL_MS_TO_NS(IMU_PUBLISH_MS),
        imu_timer_callback);

    //Joint state message setup
    js_names[0].data = js_name0;
    js_names[0].size = strlen(js_name0);
    js_names[1].data = js_name1;
    js_names[1].size = strlen(js_name1);
    
    //memory binding, bind ros msg to our joint state C arrays, so updating js_positions[] and js_velocities[] in the odom timer
    joint_msg.name.data     = js_names;
    joint_msg.name.size     = 2;    
    joint_msg.name.capacity = 2;

    joint_msg.position.data     = js_positions;
    joint_msg.position.size     = 2;
    joint_msg.position.capacity = 2;

    joint_msg.velocity.data     = js_velocities;
    joint_msg.velocity.size     = 2;
    joint_msg.velocity.capacity = 2;

    // Frame IDs 
    static char odom_frame[]       = "odom";
    static char chassis_frame[]    = "CHASSIS";
    static char imu_frame[]        = "imu_link";

    odom_msg.header.frame_id.data     = odom_frame;
    odom_msg.header.frame_id.size     = strlen(odom_frame);
    odom_msg.child_frame_id.data      = chassis_frame;
    odom_msg.child_frame_id.size      = strlen(chassis_frame);

    imu_msg.header.frame_id.data      = imu_frame;
    imu_msg.header.frame_id.size      = strlen(imu_frame);

    joint_msg.header.frame_id.data    = chassis_frame;
    joint_msg.header.frame_id.size    = strlen(chassis_frame);

    // Zero ALL covariance arrays first so unset entries are clean zeros,
    // not random stack memory. The EKF ignores entries we don't fill,
    // but random non-zero values there would confuse it.
    //memset: is C func to fill a block of memory with a value, memset(pointer, value, num_bytes)
    memset(odom_msg.pose.covariance,  0, sizeof(odom_msg.pose.covariance));
    memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
    memset(imu_msg.orientation_covariance,         0, sizeof(imu_msg.orientation_covariance));
    memset(imu_msg.angular_velocity_covariance,    0, sizeof(imu_msg.angular_velocity_covariance));
    memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));

    // debug message array setup
    debug_msg.data.data     = debug_data;
    debug_msg.data.size     = 8;
    debug_msg.data.capacity = 8;

    //gains message buffer setup
    gains_msg.data.data     = gains_buf;
    gains_msg.data.capacity = 128;
    gains_msg.data.size     = 0;

    // Executor(Brain) —> 4 handles: 2 timers + 2 subscribers
    // Note: publishers don't consume executor handles, only timers and subscribers do
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_timer(&executor, &odom_timer);
    rclc_executor_add_timer(&executor, &imu_timer);

    // Subscriber callback is only called when a new message arrives, so we use ON_NEW_DATA to specify that
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA);
    
    // gains_sub updates PID gains in real time 
    rclc_executor_add_subscription(&executor, &gains_sub, &gains_msg, &gains_callback, ON_NEW_DATA);

    // Sync time with Pi 5 via micro-ROS agent
    // The argument (1000) is the timeout in ms per attempt, so it will keep trying every second until it succeeds
    // rmw_uros_epoch_nanos() returns 0 and all message timestamps will be wrong
    while (rmw_uros_sync_session(1000) != RCL_RET_OK) {
        delay(100);  // agent not ready yet, keep trying
    }
}

//  loop()
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
