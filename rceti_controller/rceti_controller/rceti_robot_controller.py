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

        self.LED = 26

        # open the gpio chip and set the LED pin as output
        self.test = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.test, self.LED)

    def x_position_callback(self, msg):
        if self.x_position != msg.data:
            self.get_logger().info(f"Moving X to {msg.data}")
            self.x_position = msg.data
            lgpio.gpio_write(self.test, self.LED, 1)
            time.sleep(1)

            # Turn the GPIO pin off
            lgpio.gpio_write(self.test, self.LED, 0)
            time.sleep(1)

    def z_position_callback(self, msg):
        if self.z_position != msg.data:
            self.get_logger().info(f"Moving Z to {msg.data}")
            self.z_position = msg.data

    def pitch_angle_callback(self, msg):
        if self.pitch_angle != msg.data:
            self.get_logger().info(f"Adjusting Pitch to {msg.data}")
            self.pitch_angle = msg.data
            # TODO: Implement pitch 

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
