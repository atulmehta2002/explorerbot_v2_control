import rclpy
from rclpy.node import Node
import smbus2  # I2C library
import time

I2C_BUS = 1
MOTOR_ADDR = 0x34
MOTOR_FIXED_SPEED_ADDR = 0x33  # Register to set motor speed

class I2CMotorDriver(Node):
    def __init__(self):
        super().__init__('i2c_motor_driver')

        # Initialize I2C
        self.bus = smbus2.SMBus(I2C_BUS)
        
        # Example: Set motor speed (can be replaced with ROS2 topic commands)
        self.set_motor_speed([50, 50, 50, 50])  
        self.get_logger().info("I2C Motor Driver Initialized")

    def set_motor_speed(self, speeds):
        """ Send speed commands over I2C """
        try:
            self.bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, speeds)
            self.get_logger().info(f"Set motor speeds: {speeds}")
        except Exception as e:
            self.get_logger().error(f"I2C error: {e}")

def main(args=None):
    rclpy.init(args=args)
    motor_driver = I2CMotorDriver()
    rclpy.spin(motor_driver)
    motor_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
