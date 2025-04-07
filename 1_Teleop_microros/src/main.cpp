/******************************************************************
* Project: Tele operation                                         *
* -----                                                           *
* Authors: Tambu Precious April 2025                              *                               
* -----                                                           *
* Goal: Master Robotics                                           *
*******************************************************************/


#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>


// WiFi credentials
char* ssid = "YOUR_SS";
char* password = "";

// // ROS 2 Agent IP address and port
uint16_t agent_port = 8888;
char* agent_ip = "192.168.1.";

// char* ssid = "Nova"; 
// char* password = "ydnGyu74"; 


// uint16_t agent_port = 8888;
// char* agent_ip = "10.42.0.1";

// Motor control pins
const int ENB = 4;
const int IN3 = 19;
const int IN4 = 21;

const int ENA = 2;
const int IN1 = 5;
const int IN2 = 18;


// micro-ROS entities
rcl_subscription_t motor_subscriber;
std_msgs__msg__Int32MultiArray motor_msg;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Message memory
int32_t motor_values[4];

// Function prototypes
void MotorControl(int left_pwm, int left_dir, int right_pwm, int right_dir); 
bool micro_ros_init();

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 micro-ROS Motor Controller");
  
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT);
  
//   ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
//   ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
//   ledcAttachPin(ENB, LEFT_PWM_CHANNEL);
//   ledcAttachPin(ENA, RIGHT_PWM_CHANNEL);
  
  //setMotorSpeed(ENB, IN3, IN4, 0, 1, LEFT_PWM_CHANNEL);
  //setMotorSpeed(ENA, IN1, IN2, 0, 1, RIGHT_PWM_CHANNEL);
  
  micro_ros_init();
}

void loop() {
  if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
    Serial.println("micro-ROS agent not available, trying to reconnect...");
    if (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) {
      Serial.println("micro-ROS agent reconnected!");
    } else {
      MotorControl(180, 1, 180, 1);
      delay(1000);
      return;
    }
  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}


void motor_command_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  if (msg->data.size >= 4) {
   MotorControl(msg->data.data[0], msg->data.data[1], msg->data.data[2], msg->data.data[3]);
    Serial.printf("Left: PWM=%d, Dir=%d | Right: PWM=%d, Dir=%d\n", msg->data.data[0], msg->data.data[1], msg->data.data[2], msg->data.data[3]);
  }
}

bool micro_ros_init() {
  Serial.println("Initializing micro-ROS...");
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "micro_ros_motor_controller", "", &support);
  
  motor_msg.data.capacity = 4;
  motor_msg.data.size = 4;
  motor_msg.data.data = motor_values;
  
  rclc_subscription_init_default(&motor_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/motor_command");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_command_callback, ON_NEW_DATA);
  
  Serial.println("micro-ROS initialized successfully");
  return true;
}
void MotorControl(int left_pwm, int left_dir, int right_pwm, int right_dir){
    analogWrite(ENA, right_pwm); 
    analogWrite(ENB, left_pwm);
    Serial.println("MOtor called");
    if(left_dir==1 && right_dir==1){
        digitalWrite(IN3, HIGH); 
        digitalWrite(IN4, LOW);
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW);

    } else if(left_dir==0 && right_dir ==1){
        digitalWrite(IN3, LOW); 
        digitalWrite(IN4, HIGH);
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW);   

    } else if(left_dir==1 && right_dir ==0){
        digitalWrite(IN3, HIGH); 
        digitalWrite(IN4, LOW);
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, HIGH); 

    } else{
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW); 
        digitalWrite(IN4, HIGH);

    }
}
