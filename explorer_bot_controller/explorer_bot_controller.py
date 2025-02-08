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
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.get_logger().info("Robot Controller Node Started.")

        # Initialize Motors
        self.motor_init()

        # Timer to reset the servo if no commands received
        self.last_command_time = self.get_clock().now()

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

        # Convert linear velocity to motor speed
        speed = int(linear_velocity * 25)  # Adjust scale factor as needed
        speed = max(-100, min(100, speed))  # Clamp between -100 and 100
        motor_speeds = [-speed, 0, speed, 0]  # Assuming all motors move together

        # Send motor speed via I2C
        bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, motor_speeds)

        # Convert angular velocity to servo movement
        step_size = 5  # Change in degrees per message
        if angular_velocity > 0.1:  # Turn right
            servo_angle = max(servo_angle - step_size, 35)
        elif angular_velocity < -0.1:  # Turn left
            servo_angle = min(servo_angle + step_size, 150)

        # Apply new servo angle
        servo0.angle = servo_angle
        self.last_command_time = self.get_clock().now()

        self.get_logger().info(f"Motors: {speed}, Servo Angle: {servo_angle}° (angular.z: {angular_velocity})")

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


