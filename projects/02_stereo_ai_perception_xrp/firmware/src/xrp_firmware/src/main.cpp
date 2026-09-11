/*
 * XRP firmware - micro-ROS bridge between ROS 2 and the robot.
 *
 * Built on the three phase 1 test projects: the encoder ISRs and motor
 * mapping come from encoder_test, the IMU register work from imu_test, and
 * the bus configuration from i2c_scanner. Those are now drivetrain.h and
 * imu.h; this file is only the ROS 2 plumbing.
 *
 * Publishes                                   Subscribes
 *   /xrp/imu          sensor_msgs/Imu           /cmd_vel        geometry_msgs/Twist
 *   /xrp/odom         nav_msgs/Odometry         /xrp/motor_cmd  Float32MultiArray
 *   /xrp/joint_states sensor_msgs/JointState    /xrp/enable     std_msgs/Bool
 *   /xrp/encoders     Int32MultiArray           /xrp/reset_odom std_msgs/Bool
 *
 * Everything worth tuning is a ROS 2 parameter, so speeds, geometry and the
 * control mode can be changed at runtime without reflashing.
 *
 * Transport is USB serial to the Pi 5:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

/* config.h first: it defines USE_PARAMETER_SERVER, and the #if below needs
 * that macro to already exist. Include it later and the guard silently
 * evaluates to 0, the header never arrives, and every use of
 * rclc_parameter_server_t further down fails to compile. */
#include "config.h"

#if USE_PARAMETER_SERVER
#include <rclc_parameter/rclc_parameter.h>
#endif

#include "drivetrain.h"
#include "imu.h"

/* ------------------------------------------------------------- ROS state */

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_odom;
static rcl_publisher_t pub_joints;
static rcl_publisher_t pub_encoders;

static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_motor_cmd;
static rcl_subscription_t sub_enable;
static rcl_subscription_t sub_reset_odom;

static rcl_timer_t control_timer;

static sensor_msgs__msg__Imu msg_imu;
static nav_msgs__msg__Odometry msg_odom;
static sensor_msgs__msg__JointState msg_joints;
static std_msgs__msg__Int32MultiArray msg_encoders;
static geometry_msgs__msg__Twist msg_cmd_vel;
static std_msgs__msg__Float32MultiArray msg_motor_cmd;
static std_msgs__msg__Bool msg_enable;
static std_msgs__msg__Bool msg_reset_odom;

#if USE_PARAMETER_SERVER
static rclc_parameter_server_t param_server;
#endif

static Drivetrain drive;
static Imu imu;

static unsigned long cmd_timeout_ms = DEFAULT_CMD_TIMEOUT_MS;
static uint32_t tick = 0;

/* Static storage for the message arrays. micro-ROS will not allocate for us,
 * and a heap allocation per publish on a microcontroller is a bug waiting to
 * happen. */
static double joint_positions[2];
static double joint_velocities[2];
static rosidl_runtime_c__String joint_names[2];
static int32_t encoder_values[MOTOR_COUNT];
static float motor_cmd_values[4];

/* Give up and reboot rather than sit in a half-initialised state. */
#define RCCHECK(fn)                     \
    {                                   \
        rcl_ret_t rc = fn;              \
        if (rc != RCL_RET_OK) {         \
            errorLoop();                \
        }                               \
    }
#define RCSOFT(fn)          \
    {                       \
        rcl_ret_t rc = fn;  \
        (void)rc;           \
    }

static void errorLoop() {
    drive.stop();
    /* Blink so a board with no serial attached still says something. */
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(150);
    }
}

/* ------------------------------------------------------------- callbacks */

static void onCmdVel(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    drive.setTwist((float)msg->linear.x, (float)msg->angular.z);
}

static void onMotorCmd(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size >= 2) {
        drive.setDirect(msg->data.data[0], msg->data.data[1]);
    }
}

static void onEnable(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    drive.enabled = msg->data;
    if (!drive.enabled) {
        drive.stop();
    }
}

static void onResetOdom(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    if (msg->data) {
        drive.resetOdometry();
    }
}

/* --------------------------------------------------------------- publish */

static void fillStamp(builtin_interfaces__msg__Time *stamp) {
    const int64_t ns = rmw_uros_epoch_nanos();
    stamp->sec = (int32_t)(ns / 1000000000LL);
    stamp->nanosec = (uint32_t)(ns % 1000000000LL);
}

static void publishImu() {
    if (!imu.present) {
        return;
    }
    imu.read();

    fillStamp(&msg_imu.header.stamp);
    msg_imu.linear_acceleration.x = imu.ax;
    msg_imu.linear_acceleration.y = imu.ay;
    msg_imu.linear_acceleration.z = imu.az;
    msg_imu.angular_velocity.x = imu.gx;
    msg_imu.angular_velocity.y = imu.gy;
    msg_imu.angular_velocity.z = imu.gz;

    /* -1 in the first element of the orientation covariance is REP-145 for
     * "this message carries no orientation". We only have a 6-DoF IMU, so
     * saying so is more useful than publishing a fake identity quaternion. */
    msg_imu.orientation_covariance[0] = -1.0;

    RCSOFT(rcl_publish(&pub_imu, &msg_imu, NULL));
}

static void publishOdom() {
    fillStamp(&msg_odom.header.stamp);

    msg_odom.pose.pose.position.x = drive.odom.x;
    msg_odom.pose.pose.position.y = drive.odom.y;
    msg_odom.pose.pose.position.z = 0.0;

    /* Yaw-only quaternion. */
    const float half = drive.odom.theta * 0.5f;
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sinf(half);
    msg_odom.pose.pose.orientation.w = cosf(half);

    msg_odom.twist.twist.linear.x = drive.odom.linear;
    msg_odom.twist.twist.angular.z = drive.odom.angular;

    RCSOFT(rcl_publish(&pub_odom, &msg_odom, NULL));
}

static void publishJoints() {
    fillStamp(&msg_joints.header.stamp);

    joint_positions[0] = drive.left.position_rad;
    joint_positions[1] = drive.right.position_rad;
    joint_velocities[0] = drive.left.velocity_rad_s;
    joint_velocities[1] = drive.right.velocity_rad_s;

    RCSOFT(rcl_publish(&pub_joints, &msg_joints, NULL));
}

static void publishEncoders() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        encoder_values[i] = (int32_t)drive.rawCounts(i);
    }
    RCSOFT(rcl_publish(&pub_encoders, &msg_encoders, NULL));
}

/* ------------------------------------------------------------ parameters */

#if USE_PARAMETER_SERVER
static bool onParameterChanged(const Parameter *old_param,
                               const Parameter *new_param,
                               void *context) {
    (void)context;
    if (old_param == NULL || new_param == NULL) {
        return false;                 // reject adds and deletes
    }

    const char *name = new_param->name.data;

    if (strcmp(name, "max_linear_mps") == 0) {
        drive.max_linear = (float)new_param->value.double_value;
    } else if (strcmp(name, "max_angular_rps") == 0) {
        drive.max_angular = (float)new_param->value.double_value;
    } else if (strcmp(name, "wheel_radius_m") == 0) {
        drive.wheel_radius = (float)new_param->value.double_value;
    } else if (strcmp(name, "wheel_separation_m") == 0) {
        drive.wheel_separation = (float)new_param->value.double_value;
    } else if (strcmp(name, "counts_per_rev") == 0) {
        drive.counts_per_rev = (float)new_param->value.double_value;
    } else if (strcmp(name, "min_pwm") == 0) {
        drive.min_pwm = (int)new_param->value.integer_value;
    } else if (strcmp(name, "max_pwm") == 0) {
        drive.max_pwm = (int)new_param->value.integer_value;
    } else if (strcmp(name, "cmd_timeout_ms") == 0) {
        cmd_timeout_ms = (unsigned long)new_param->value.integer_value;
    } else if (strcmp(name, "closed_loop") == 0) {
        drive.closed_loop = new_param->value.bool_value;
    } else if (strcmp(name, "kp") == 0) {
        drive.kp = (float)new_param->value.double_value;
    } else if (strcmp(name, "ki") == 0) {
        drive.ki = (float)new_param->value.double_value;
    } else if (strcmp(name, "invert_left") == 0) {
        drive.invert_left = new_param->value.bool_value;
    } else if (strcmp(name, "invert_right") == 0) {
        drive.invert_right = new_param->value.bool_value;
    } else if (strcmp(name, "enabled") == 0) {
        drive.enabled = new_param->value.bool_value;
        if (!drive.enabled) {
            drive.stop();
        }
    } else {
        return false;
    }
    return true;
}

static void declareParameters() {
    rclc_add_parameter(&param_server, "max_linear_mps", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "max_angular_rps", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "wheel_radius_m", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "wheel_separation_m", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "counts_per_rev", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "min_pwm", RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "max_pwm", RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "cmd_timeout_ms", RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "closed_loop", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "kp", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "ki", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "invert_left", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "invert_right", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "enabled", RCLC_PARAMETER_BOOL);

    rclc_parameter_set_double(&param_server, "max_linear_mps", drive.max_linear);
    rclc_parameter_set_double(&param_server, "max_angular_rps", drive.max_angular);
    rclc_parameter_set_double(&param_server, "wheel_radius_m", drive.wheel_radius);
    rclc_parameter_set_double(&param_server, "wheel_separation_m", drive.wheel_separation);
    rclc_parameter_set_double(&param_server, "counts_per_rev", drive.counts_per_rev);
    rclc_parameter_set_int(&param_server, "min_pwm", drive.min_pwm);
    rclc_parameter_set_int(&param_server, "max_pwm", drive.max_pwm);
    rclc_parameter_set_int(&param_server, "cmd_timeout_ms", (int)cmd_timeout_ms);
    rclc_parameter_set_bool(&param_server, "closed_loop", drive.closed_loop);
    rclc_parameter_set_double(&param_server, "kp", drive.kp);
    rclc_parameter_set_double(&param_server, "ki", drive.ki);
    rclc_parameter_set_bool(&param_server, "invert_left", drive.invert_left);
    rclc_parameter_set_bool(&param_server, "invert_right", drive.invert_right);
    rclc_parameter_set_bool(&param_server, "enabled", drive.enabled);
}
#endif  /* USE_PARAMETER_SERVER */

/* ------------------------------------------------------------ control loop */

static void onControlTimer(rcl_timer_t *timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    /* A controller that stops talking must not leave the robot driving. */
    if (drive.commandStale(cmd_timeout_ms)) {
        drive.setTwist(0.0f, 0.0f);
    }

    drive.update();
    tick++;

    const uint32_t imu_every = CONTROL_HZ / IMU_PUBLISH_HZ;
    const uint32_t odom_every = CONTROL_HZ / ODOM_PUBLISH_HZ;
    const uint32_t joint_every = CONTROL_HZ / JOINT_PUBLISH_HZ;

    if (imu_every && tick % imu_every == 0) {
        publishImu();
    }
    if (odom_every && tick % odom_every == 0) {
        publishOdom();
    }
    if (joint_every && tick % joint_every == 0) {
        publishJoints();
        publishEncoders();
    }
}

/* ------------------------------------------------------------ message init */

static void setString(rosidl_runtime_c__String *str, const char *value) {
    str->data = (char *)value;
    str->size = strlen(value);
    str->capacity = str->size + 1;
}

static void initMessages() {
    sensor_msgs__msg__Imu__init(&msg_imu);
    setString(&msg_imu.header.frame_id, IMU_FRAME);

    nav_msgs__msg__Odometry__init(&msg_odom);
    setString(&msg_odom.header.frame_id, ODOM_FRAME);
    setString(&msg_odom.child_frame_id, BASE_FRAME);

    sensor_msgs__msg__JointState__init(&msg_joints);
    setString(&msg_joints.header.frame_id, BASE_FRAME);
    setString(&joint_names[0], "left_wheel_joint");
    setString(&joint_names[1], "right_wheel_joint");
    msg_joints.name.data = joint_names;
    msg_joints.name.size = 2;
    msg_joints.name.capacity = 2;
    msg_joints.position.data = joint_positions;
    msg_joints.position.size = 2;
    msg_joints.position.capacity = 2;
    msg_joints.velocity.data = joint_velocities;
    msg_joints.velocity.size = 2;
    msg_joints.velocity.capacity = 2;

    std_msgs__msg__Int32MultiArray__init(&msg_encoders);
    msg_encoders.data.data = encoder_values;
    msg_encoders.data.size = MOTOR_COUNT;
    msg_encoders.data.capacity = MOTOR_COUNT;

    /* Incoming arrays need somewhere to land before the callback runs. */
    std_msgs__msg__Float32MultiArray__init(&msg_motor_cmd);
    msg_motor_cmd.data.data = motor_cmd_values;
    msg_motor_cmd.data.size = 0;
    msg_motor_cmd.data.capacity = 4;
}

/* -------------------------------------------------------------- lifecycle */

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    drive.begin();
    imu.begin();
    /* Keep the robot still for this - it is measuring what "not moving"
     * looks like to the gyro. */
    imu.calibrateGyro(120);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "xrp_firmware", "", &support));

    initMessages();

    RCCHECK(rclc_publisher_init_best_effort(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "xrp/imu"));
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "xrp/odom"));
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_joints, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "xrp/joint_states"));
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_encoders, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "xrp/encoders"));

    RCCHECK(rclc_subscription_init_default(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(
        &sub_motor_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "xrp/motor_cmd"));
    RCCHECK(rclc_subscription_init_default(
        &sub_enable, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "xrp/enable"));
    RCCHECK(rclc_subscription_init_default(
        &sub_reset_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "xrp/reset_odom"));

    RCCHECK(rclc_timer_init_default(
        &control_timer, &support, RCL_MS_TO_NS(1000 / CONTROL_HZ), onControlTimer));

#if USE_PARAMETER_SERVER
    rclc_parameter_options_t param_options = {
        .notify_changed_over_dds = true,
        .max_params = 16,
        .allow_undeclared_parameters = false,
        .low_mem_mode = true,
    };
    RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_options));
    const size_t handles = 4 + 1 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
#else
    const size_t handles = 4 + 1;
#endif

    RCCHECK(rclc_executor_init(&executor, &support.context, handles, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_cmd_vel, &msg_cmd_vel, &onCmdVel, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_motor_cmd, &msg_motor_cmd, &onMotorCmd, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_enable, &msg_enable, &onEnable, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_reset_odom, &msg_reset_odom, &onResetOdom, ON_NEW_DATA));

#if USE_PARAMETER_SERVER
    declareParameters();
    RCCHECK(rclc_executor_add_parameter_server(
        &executor, &param_server, onParameterChanged));
#endif

    /* Ask the agent for the time, so stamps line up with the rest of ROS 2. */
    RCSOFT(rmw_uros_sync_session(1000));

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    static unsigned long last_blink = 0;
    if (millis() - last_blink >= 500) {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
