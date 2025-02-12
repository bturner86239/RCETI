import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class RCETIRobotController(Node):

    def __init__(self):
        super().__init__('rceti_controller')
        self.x_position = self.create_subscription(
            Float32,
            'rceti/x_position',
            self.x_position_callback,
            10)
        self.z_position = self.create_subscription(
            Float32,
            'rceti/z_position',
            self.z_position_callback,
            10)
        self.pitch_angle = self.create_subscription(
            Float32,
            'rceti/pitch_angle',
            self.pitch_angle_callback,
            10)
        self.x_position # prevent unused variable warning
        self.z_position
        self.pitch_angle
        
    def x_position_callback(self, msg):
        self.get_logger().info('I am going to x:"%s"' % msg.data)
        
    def z_position_callback(self, msg):
        print("Hi")
        # THIS IS WHERE THE Z POSITION ON THE ROBOT SIDE WILL MOVE THE ROBOT
        
        ## TODO ##
        
    def pitch_angle_callback(self, msg):
        print("Hi")
        # THIS IS WHERE THE PITCH ANGLE ON THE ROBOT SIDE WILL MOVE THE ROBOT
        
        ## TODO ##


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RCETIRobotController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()