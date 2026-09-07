/*********************************************************
 *                                                       *
 *  Project:     RosChallengeProject 1                   *
 *  File:        teleop_to_motor.cpp                     *
 *  Author:      Tambu Precious Takum                    *
 *  Date:        2025-03-27                              *
 *                                                       *
 *  Description:                                         *
 *  - Executable converts ROS velocity messages to PWM   *
 *    signals for Arduino in the range [0-255] for a     *
 *    differential drive robot. Publishes this integer   *
 *    to the micro-ROS agent.                            *
 *                                                       *
 *  Updates:                                             *
 *  - Changed message type to Int32MultiArray to send    *
 *    multiple values in one message:                    *
 *    - Left motor PWM (0-255)                           *
 *    - Left motor direction (1 for forward, 0 for back) *
 *    - Right motor PWM (0-255)                          *
 *    - Right motor direction (1 for forward, 0 for back)*
 *                                                       *
 *  - Implemented proper differential drive kinematics   *
 *    to calculate wheel speeds based on:                *
 *    - Linear velocity (forward/backward)               *
 *    - Angular velocity (turning)                       *
 *    - Wheel separation distance                        *
 *                                                       *
 *  - Added mapping function to convert velocities to    *
 *    PWM values (0-255).                                *
 *  - Added direction control for each motor to handle   *
 *    negative velocities.                               *
 *                                                       *
 *********************************************************/




#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class TeleopToMotor : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    
    // Constants for differential drive
    const double WHEEL_SEPARATION = 0.2; // Distance between wheels in meters (adjust to your robot)
    const int MAX_PWM = 255;            // Maximum PWM value
    const double MAX_LINEAR_SPEED = 1.0; // Maximum linear speed in m/s
    const double MAX_ANGULAR_SPEED = 2.0; // Maximum angular speed in rad/s
    
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Get linear and angular velocities
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;
        
        // Calculate wheel velocities using differential drive kinematics
        double left_wheel_speed = linear_x - (angular_z * WHEEL_SEPARATION / 2.0);
        double right_wheel_speed = linear_x + (angular_z * WHEEL_SEPARATION / 2.0);
        
        // Map wheel speeds to PWM values (0-255)
        int left_pwm = mapToPWM(left_wheel_speed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        int right_pwm = mapToPWM(right_wheel_speed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        
        // Determine direction (for H-bridge control)
        int left_dir = (left_wheel_speed >= 0) ? 1 : 0;
        int right_dir = (right_wheel_speed >= 0) ? 1 : 0;
        
        // Create message with motor commands
        auto motor_msg = std_msgs::msg::Int32MultiArray();
        motor_msg.data = {
            abs(left_pwm),   // Left motor PWM
            left_dir,        // Left motor direction
            abs(right_pwm),  // Right motor PWM
            right_dir        // Right motor direction
        };
        
        // Publish the motor speed command
        publisher_->publish(motor_msg);
        
        RCLCPP_INFO(
            this->get_logger(), 
            "Linear: %.2f, Angular: %.2f | Left PWM: %d (dir: %d), Right PWM: %d (dir: %d)",
            linear_x, angular_z, abs(left_pwm), left_dir, abs(right_pwm), right_dir
        );
    }
    
    // Map a value from input range to output range (0-255)
    int mapToPWM(double value, double in_min, double in_max) {
        // First clamp the value to the input range
        if (value > in_max) value = in_max;
        if (value < in_min) value = in_min;
        
        // Map to 0-MAX_PWM range
        return static_cast<int>((abs(value) / std::max(abs(in_min), abs(in_max))) * MAX_PWM);
    }
    
public:
    TeleopToMotor() : Node("teleop_to_motor") {
        // Create a subscriber to listen to /cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TeleopToMotor::twist_callback, this, std::placeholders::_1)
        );
        
        // Create a publisher to send motor commands
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/motor_command", 10);
        
        RCLCPP_INFO(this->get_logger(), "Differential drive node initialized");
        RCLCPP_INFO(this->get_logger(), "Listening to /cmd_vel and publishing to /motor_command");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopToMotor>());
    rclcpp::shutdown();
    return 0;
}
