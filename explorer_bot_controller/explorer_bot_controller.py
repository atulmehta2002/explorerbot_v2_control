import time
import board
import busio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import smbus
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# I2C Config for Motors
I2C_BUS = 1
MOTOR_ADDR = 0x34

# Motor Register Addresses
MOTOR_TYPE_ADDR = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_SPEED_ADDR = 0x33

# Motor Type Configuration
MOTOR_TYPE_JGB37_520_12V_110RPM = 3  # Default motor type
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0  # Default polarity

# Initialize I2C Bus
bus = smbus.SMBus(I2C_BUS)

# Initialize I2C and PCA9685 for Servo Control
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Set PWM frequency to 50Hz

# Initialize Servo on Channel 0
servo0 = servo.Servo(pca.channels[0])

# Set initial Servo Angle
servo_angle = 90  # Start at center
servo0.angle = servo_angle


class ExplorerBotController(Node):
    def __init__(self):
        super().__init__("explorer_bot_controller")

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist, "/diffbot_base_controller/cmd_vel_unstamped", self.cmd_vel_callback, 10
        )

        self.get_logger().info("Robot Controller Node Started.")

        # Initialize Motors
        self.motor_init()

        # Timer to reset the servo if no commands received
        self.last_command_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.check_timeout)

    def motor_init(self):
        """Initialize Motor Type and Encoder Polarity"""
        bus.write_byte_data(MOTOR_ADDR, MOTOR_TYPE_ADDR, MotorType)
        time.sleep(0.5)
        bus.write_byte_data(MOTOR_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)
        self.get_logger().info("Motors Initialized.")

    def cmd_vel_callback(self, msg):
        """Handles Twist messages for motor and servo control"""
        global servo_angle

        # Extract linear and angular velocity
        linear_velocity = msg.linear.x  # Forward/Backward
        angular_velocity = msg.angular.z  # Left/Right

        # if angular_velocity == 0.0 & linear_velocity != 0.0:
        #     left_speed = int((linear_velocity) * 35)
        #     right_speed = int((linear_velocity) * 35)
            
        #     left_speed = max(-50, min(50, left_speed))  # Clamp between -100 and 100
        #     right_speed = max(-50, min(50, right_speed))  # Clamp between -100 and 100
            
        if angular_velocity != 0.0 and linear_velocity == 0.0:
            left_speed = int(angular_velocity * 16)
            right_speed = int(-angular_velocity * 16)
        
        else:
            left_speed = int(linear_velocity * 35)
            right_speed = int(linear_velocity * 35)
        
        # Clamp speeds between -50 and 50
        left_speed = max(-50, min(50, left_speed))
        right_speed = max(-50, min(50, right_speed))

        # Create motor command array
        motor_speeds = [-left_speed, 0, right_speed, 0]

        # Send motor speed via I2C
        bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, motor_speeds)

        # Convert angular velocity to servo movement
        step_size = 5  # Change in degrees per message
        if angular_velocity > 0.1:  # Turn right
            servo_angle = max(servo_angle - step_size, 60)
        elif angular_velocity < -0.1:  # Turn left
            servo_angle = min(servo_angle + step_size, 120)

        # Apply new servo angle
        servo0.angle = servo_angle
        self.last_command_time = self.get_clock().now()

        self.get_logger().info(f"Motor_L: {left_speed}, Motor_R: {right_speed}, Servo Angle: {servo_angle}° (angular.z: {angular_velocity})")

    def check_timeout(self):
        """Checks if the last command was received more than 2 seconds ago and resets motors and servo"""
        time_elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if time_elapsed > 0.3:
            self.stop_motors()
            global servo_angle
            servo_angle = 90  # Reset servo to center
            servo0.angle = servo_angle
            self.get_logger().info("No command received in 0.3s, resetting motors and servo.")

    def stop_motors(self):
        """Stops the motors on shutdown"""
        bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, [0, 0, 0, 0])
        self.get_logger().info("Motors stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerBotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        pca.deinit()
        rclpy.shutdown()
        print("Robot control stopped. PCA9685 deinitialized.")


if __name__ == "__main__":
    main()


