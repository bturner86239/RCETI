import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time  # for delays/testing
import lgpio

class RCETIRobotController(Node):

    def __init__(self):
        super().__init__('rceti_controller')
        
        self.x_position_sub = self.create_subscription(
            Float32,
            'rceti/x_position',
            self.x_position_callback,
            10)
        self.z_position_sub = self.create_subscription(
            Float32,
            'rceti/z_position',
            self.z_position_callback,
            10)
        self.pitch_angle_sub = self.create_subscription(
            Float32,
            'rceti/pitch_angle',
            self.pitch_angle_callback,
            10)

        #prevent unused variable warning
        self.x_position_sub  
        self.z_position_sub
        self.pitch_angle_sub

        #position Tracking
        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.0

        self.X_DIRECTION_PIN = 21
        self.X_PULSE_PIN = 20
        self.Z_DIRECTION_PIN = 19
        self.Z_PULSE_PIN = 18

        # open the gpio chip and set the LED pin as output
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.X_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.X_PULSE_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_PULSE_PIN)


    def x_position_callback(self, msg):
        if self.x_position != msg.data:
            self.get_logger().info(f"Moving X to {msg.data}")
            self.move_stepper(500, 1, self.X_DIRECTION_PIN, self.X_PULSE_PIN)
            time.sleep(1)
            self.move_stepper(500, 0, self.X_DIRECTION_PIN, self.X_PULSE_PIN)
            self.x_position = msg.data


    def z_position_callback(self, msg):
        if self.z_position != msg.data:
            self.get_logger().info(f"Moving Z to {msg.data}")
            self.move_stepper(500, 1, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
            time.sleep(1)
            self.move_stepper(500, 0, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
            self.z_position = msg.data

    def pitch_angle_callback(self, msg):
        if self.pitch_angle != msg.data:
            self.get_logger().info(f"Adjusting Pitch to {msg.data}")
            self.pitch_angle = msg.data
            # TODO: Implement pitch 

    def move_stepper(self, steps, direction, direction_pin, pulse_pin, delay=0.001):
        lgpio.gpio_write(self.chip, direction_pin, direction)

        for _ in range(steps):
                lgpio.gpio_write(self.chip, pulse_pin, 1)
                time.sleep(delay)
                lgpio.gpio_write(self.chip, pulse_pin, 0)
                time.sleep(delay)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
