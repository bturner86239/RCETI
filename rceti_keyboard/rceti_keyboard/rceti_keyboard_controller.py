import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class RCETIKeyboardController(Node):

    def __init__(self):
        super().__init__('rceti_keyboard')
        self.x_position_ = self.create_publisher(Float32, 'rceti/x_position', 10)
        self.z_position_ = self.create_publisher(Float32, 'rceti/z_position', 10)
        self.pitch_angle_ = self.create_publisher(Float32, 'rceti/pitch_angle', 10)
        timer_period = 0.1  # seconds
        self.x_timer = self.create_timer(timer_period, self.x_position_callback)
        self.z_timer = self.create_timer(timer_period, self.z_position_callback)
        self.pitch_timer = self.create_timer(timer_period, self.pitch_angle_callback)
        self.i = 0


    # def timer_callback(self):
    #     msg = ()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1
    
    def x_position_callback(self):
        print("Hi")
        # THIS IS WHERE THE X POSITION ON THE KEYBOARD SIDE WILL MOVE THE ROBOT
        
        ## TODO ##
        
    def z_position_callback(self):
        print("Hi")
        # THIS IS WHERE THE Z POSITION ON THE KEYBOARD SIDE
        
        ## TODO ##
        
    def pitch_angle_callback(self):
        print("Hi")
        # THIS IS WHERE THE PITCH ANGLE ON THE KEYBOARD SIDE
        
        ## TODO ##


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RCETIKeyboardController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()