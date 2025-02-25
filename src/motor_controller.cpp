#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <vector>
#include <cmath>

#define I2C_BUS 1
#define MOTOR_ADDR 0x34
#define MOTOR_FIXED_SPEED_ADDR 0x33  // Base address for motor speed control

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {
        // Initialize WiringPi and I2C
        wiringPiSetup();
        motor_fd_ = wiringPiI2CSetup(MOTOR_ADDR);
        if (motor_fd_ == -1) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize I2C communication. Exiting...");
            throw std::runtime_error("I2C initialization failed");
        }

        // Subscribe to /cmd_vel topic
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10, 
            std::bind(&MotorController::cmdVelCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Motor controller node started, waiting for /diffbot_base_controller/cmd_vel_unstamped messages...");
    }

private:
    int motor_fd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_x  = msg->linear.x/4;   // Forward/backward speed
        double angular_z = msg->angular.z;  // Rotation speed

        // Robot parameters
        const double wheel_base = 0.175; // Distance between wheels (meters)
        const double max_speed  = 100;   // Maximum motor speed (scaled to -100 to 100)

        // Differential drive formula: Convert linear & angular velocity to wheel speeds
        double left_speed  = (linear_x - (wheel_base * angular_z)) * max_speed;
        double right_speed = (linear_x + (wheel_base * angular_z)) * max_speed;

        // Clamp speed values to [-100, 100] range
        left_speed  = std::round(std::clamp(left_speed,  -100.0, 100.0));
        right_speed = std::round(std::clamp(right_speed, -100.0, 100.0));

        // Convert to int8_t for I2C communication
        std::vector<int8_t> motor_speeds = {static_cast<int8_t>(right_speed), 0,
                                            static_cast<int8_t>(-left_speed), 0};

        // Send motor speeds via I2C and check for errors
        for (size_t i = 0; i < motor_speeds.size(); i++) {
            int status = wiringPiI2CWriteReg8(motor_fd_, MOTOR_FIXED_SPEED_ADDR + i, motor_speeds[i]);
            if (status == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write speed to motor register %02X", static_cast<unsigned int>(MOTOR_FIXED_SPEED_ADDR + i));
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Set motor speeds: L=%d, R=%d", motor_speeds[2], motor_speeds[0]);
    }
};

// Main function
int main(int argc, char *argv[]) {
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<MotorController>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

