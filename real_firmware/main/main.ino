// ═══════════════════════════════════════════════════════
//  Real Robot ESP32 Firmware
//  micro-ROS Jazzy | Arduino IDE 2.x | ESP32 core 2.0.2
//
//  Publishes:  /odom        (nav_msgs/Odometry)
//              /imu/data    (sensor_msgs/Imu)
//              /joint_states (sensor_msgs/JointState)
//  Subscribes: /cmd_vel     (geometry_msgs/TwistStamped)
// ═══════════════════════════════════════════════════════

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist_stamped.h>

#include <Wire.h>
#include <MPU6050.h>

#include "config.h"

// ── micro-ROS variables─────────────────────────────────
rcl_node_t            node;
rcl_allocator_t       allocator; // for dynamic memory management in ROS 2
rclc_support_t        support;  //for init and shutdown of ROS context
rclc_executor_t       executor; //run callbacks + timers

// Publishers and subscriber from micro-ROS node
rcl_publisher_t       odom_pub;
rcl_publisher_t       imu_pub;
rcl_publisher_t       joint_pub;
rcl_subscription_t    cmd_sub;

rcl_timer_t           odom_timer; //runs odemtry calculations and publishes at 20 Hz(periodically)
rcl_timer_t           imu_timer; //runs IMU reading and publishes at 50 Hz (periodically)

// ── Message variables──────────────────────────────────────────
nav_msgs__msg__Odometry           odom_msg;
sensor_msgs__msg__Imu             imu_msg;
sensor_msgs__msg__JointState      joint_msg;
geometry_msgs__msg__TwistStamped  cmd_msg;

// ── Encoder variables (volatile — modified in ISR) ────────
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// ── robot position variables ────────────────────────────────────
float pos_x   = 0.0f;
float pos_y   = 0.0f;
float heading = 0.0f;   // direction (radians)

//previous encoder ticks for odometry calculation
long  last_left_ticks  = 0; 
long  last_right_ticks = 0;

// ── Motor command state ───────────────────────────────
float cmd_linear  = 0.0f;       //forward/backward speed in m/s
float cmd_angular = 0.0f;      //rotational speed in rad/s
unsigned long last_cmd_ms = 0; //last time cmd_vel recieved

// ── IMU object (represnt the sensor)───────────────────────────────────────────────
MPU6050 mpu;

// ── Joint state arrays (raw data of joint positions) (2 wheels) ─────────────────────
double  js_positions[2] = {0.0, 0.0};
double  js_velocities[2] = {0.0, 0.0};
rosidl_runtime_c__String js_names[2]; //ros string type for joint names
char    js_name0[] = "left_wheel";
char    js_name1[] = "right_wheel";

// ═══════════════════════════════════════════════════════
//Encoder ISRs (interrupt service routines)
//ESP32 has two types of RAMS (Flash(slow) and IRAM(fast)) and ISRs must be in IRAM to run fast and correctly
// ═══════════════════════════════════════════════════════
void IRAM_ATTR left_enc_isr()  { left_ticks++;  } 
void IRAM_ATTR right_enc_isr() { right_ticks++; }

// ══════════════════════════════════════════════════════
//  Motor control — BTS7960
//  takes speed values: -255 to +255
// ═══════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════
//  cmd_vel callback
//  Converts linear/angular velocity → left/right PWM
// ═══════════════════════════════════════════════════════
void cmd_vel_callback(const void * msg_in) {
    //convert raw pointer to ROS message type
    const geometry_msgs__msg__TwistStamped * msg =
        (const geometry_msgs__msg__TwistStamped *)msg_in;

    cmd_linear  = msg->twist.linear.x; //read forward speed from message
    cmd_angular = msg->twist.angular.z; //read rotational speed from message
    last_cmd_ms = millis();             //store current time

    // Differential drive velocity → wheel speeds (m/s)
    float v_left  = cmd_linear - (cmd_angular * WHEEL_BASE / 2.0f);
    float v_right = cmd_linear + (cmd_angular * WHEEL_BASE / 2.0f);

    // Max wheel speed at 200 RPM = w x r =((200/60) * 2π )* (0.065 )≈ 1.36 m/s
    const float MAX_SPEED = 1.36f;

    // convert speed to motor power, Normalise to PWM range -255 to +255
    int pwm_left  = (int)((v_left  / MAX_SPEED) * 255.0f);
    int pwm_right = (int)((v_right / MAX_SPEED) * 255.0f);

    //limits PWM to -255 to +255
    pwm_left  = constrain(pwm_left,  -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

    set_motor_left(pwm_left);
    set_motor_right(pwm_right);
}

// ═══════════════════════════════════════════════════════
//  Odometry timer callback — runs at 20 Hz
// ═══════════════════════════════════════════════════════
void odom_timer_callback(rcl_timer_t * timer, int64_t /*last_call*/) {
    if (timer == NULL) return;

    // Stop motors if cmd_vel timeout
    if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
        stop_motors();
        cmd_linear  = 0.0f;
        cmd_angular = 0.0f;
    }

    // Read encoder ticks (atomic copy)
    noInterrupts(); //stop ISR temporarily so I can copy encoder values without them changing mid-read (which would cause incorrect odometry calculations)
    long cur_left  = left_ticks;
    long cur_right = right_ticks;
    interrupts(); //resume execution of ISRs

    //encoder difference : how much each encoder moved since last time we checked (in ticks)
    long d_left  = cur_left  - last_left_ticks;
    long d_right = cur_right - last_right_ticks;
    last_left_ticks  = cur_left;
    last_right_ticks = cur_right;

    // Convert ticks → metres
    float dist_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
    float dl = d_left  * dist_per_tick;
    float dr = d_right * dist_per_tick;

    // Differential drive odometry
    float d_centre  = (dl + dr) / 2.0f;       //how far robot moved forward
    float d_heading = (dr - dl) / WHEEL_BASE; //how much robot rotated (radians)

    //robot keeps updating its x and y position , and rotation over time and we use that to calculate the change in x and y position based on the distance it has moved forward (d_centre) and its current heading
    heading += d_heading;
    pos_x   += d_centre * cosf(heading);  //cosine is the horizontal component of the movement
    pos_y   += d_centre * sinf(heading);  //sine is the vertical component of the movement

    // calculate velocity, dt in seconds
    float dt = ODOM_PUBLISH_MS / 1000.0f;
    float v_linear  = d_centre  / dt;
    float v_angular = d_heading / dt;

    // theta=L/r, Joint positions (radians) and velocities, needed with the urdf for robot_state_publisher to work in ROS 2    
    js_positions[0] += dl / WHEEL_RADIUS;
    js_positions[1] += dr / WHEEL_RADIUS;
    js_velocities[0] = (dl / WHEEL_RADIUS) / dt;
    js_velocities[1] = (dr / WHEEL_RADIUS) / dt;

    // Timestamp , in ros2 time is reprsented as secs + nano secs 
    int64_t now_ns = rmw_uros_epoch_nanos(); //built in micro ros function
    int32_t sec    = (int32_t)(now_ns / 1000000000LL);
    uint32_t nsec  = (uint32_t)(now_ns % 1000000000LL);

    // ── Publish /odom ──────────────────────────────────
    //attach time to this message and fill in the position and velocity data
    odom_msg.header.stamp.sec     = sec;
    odom_msg.header.stamp.nanosec = nsec;
    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;

    // Heading → quaternion (yaw only, 2D robot)
    //quaternion are 4 numbers (x, y, z, w)--> represent rotation in 3D space
    odom_msg.pose.pose.orientation.z = sinf(heading / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(heading / 2.0f);
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;

    odom_msg.twist.twist.linear.x  = v_linear;
    odom_msg.twist.twist.angular.z = v_angular;

    rcl_publish(&odom_pub, &odom_msg, NULL);

    // ── Publish /joint_states ──────────────────────────
    joint_msg.header.stamp.sec     = sec;
    joint_msg.header.stamp.nanosec = nsec;
    joint_msg.position.data        = js_positions;
    joint_msg.velocity.data        = js_velocities;

    rcl_publish(&joint_pub, &joint_msg, NULL);
}

// ═══════════════════════════════════════════════════════
//  IMU timer callback — runs at 50 Hz (50 times /sec)
// ═══════════════════════════════════════════════════════
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

    // -1 = covariance unknown
    imu_msg.orientation_covariance[0]         = -1.0f;
    imu_msg.angular_velocity_covariance[0]    =  0.02f;
    imu_msg.linear_acceleration_covariance[0] =  0.04f;

    rcl_publish(&imu_pub, &imu_msg, NULL);
}

// ═══════════════════════════════════════════════════════
//  setup() does 5 main things
// 1.connect ESP32 to micro-ROS agent via serial
// 2.initialize hardware (motors, encoders, IMU)
// 3.create micro-Ros node
// 4.create 2publishers, 1 subscriber
// 5.create timers + executor for callbacks(scheduler for when to run the code in the callbacks)
// ═════════════════════════════════════
void setup() {

    // ── Serial for micro-ROS ───────────────────────────
    Serial.begin(MICROROS_SERIAL_BAUD);
    set_microros_serial_transports(Serial);
    delay(2000);   // wait for agent to be ready

    // ── BTS7960 enable pins ────────────────────────────
    pinMode(L_R_EN, OUTPUT); digitalWrite(L_R_EN, HIGH);
    pinMode(L_L_EN, OUTPUT); digitalWrite(L_L_EN, HIGH);
    pinMode(R_R_EN, OUTPUT); digitalWrite(R_R_EN, HIGH);
    pinMode(R_L_EN, OUTPUT); digitalWrite(R_L_EN, HIGH);

    // ── PWM pins ───────────────────────────────────────
    pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
    pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
    stop_motors();  //safety start

    // ── Encoders ───────────────────────────────────────
    pinMode(LEFT_ENC_A,  INPUT_PULLUP);  //input_PULLUP : internal resitor (clean signal)
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  left_enc_isr,  RISING); //every time signal goes from low->high it calls left_enc_isr() function to add tick
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), right_enc_isr, RISING);

    // ── MPU-6050 ───────────────────────────────────────
    Wire.begin(IMU_SDA, IMU_SCL); //Initialize I2C communication for the IMU
    mpu.initialize();              //Initialize the MPU-6050 sensor
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); //set the ranges (default)
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    // ── creating ros2 node named "esp32_robot_node" on ESP32 ────────────────────────────────
    //API functions for creating ROS 2 nodes and publishers/subscribers are a bit more complex than in regular ROS 2 because micro-ROS is designed to run on resource-constrained devices and doesn't have dynamic memory allocation by default, so we need to set up an allocator and support structure explicitly.
    allocator = rcl_get_default_allocator();          //micro ros needs explicit memory handling
    rclc_support_init(&support, 0, NULL, &allocator); //initialize this ros2 runtime env on ESP32 using this memory allocator
    rclc_node_init_default(&node, "esp32_robot_node", "", &support);

    // ── Publishers ────────────────────────────────────
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

    // ── Subscriber ────────────────────────────────────
    rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        "/cmd_vel_stamped");

    // ── Timers = periodic functions────────────────────────────────────────
    rclc_timer_init_default(
        &odom_timer, &support,
        RCL_MS_TO_NS(ODOM_PUBLISH_MS),
        odom_timer_callback);

    rclc_timer_init_default(
        &imu_timer, &support,
        RCL_MS_TO_NS(IMU_PUBLISH_MS),
        imu_timer_callback);

    // ── Joint state message setup ─────────────────────
    js_names[0].data = js_name0;
    js_names[0].size = strlen(js_name0);
    js_names[1].data = js_name1;
    js_names[1].size = strlen(js_name1);
    
    //joint_msg is a ros msg of type sensor_msgs/JointState which has arrays for joint names, positions, and velocities. We need to set up these arrays with the correct data and sizes so that when we publish joint states, the message is correctly formed and can be understood by other ROS 2 nodes (like robot_state_publisher) that subscribe to /joint_states
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
    odom_msg.header.frame_id.data   = (char*)"odom";
    odom_msg.child_frame_id.data    = (char*)"CHASSIS";
    imu_msg.header.frame_id.data    = (char*)"imu_link";
    joint_msg.header.frame_id.data  = (char*)"CHASSIS";

    // ── Executor(Brain) — 3 handles: 2 timers + 1 subscriber ─
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_timer(&executor, &odom_timer);
    rclc_executor_add_timer(&executor, &imu_timer);
    // Subscriber callback is only called when a new message arrives, so we use ON_NEW_DATA to specify that
    rclc_executor_add_subscription(
        &executor, &cmd_sub, &cmd_msg,
        &cmd_vel_callback, ON_NEW_DATA);

    // Sync time with Pi 5 via micro-ROS agent
    rmw_uros_sync_session(1000);
}

// ═══════════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════════
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}