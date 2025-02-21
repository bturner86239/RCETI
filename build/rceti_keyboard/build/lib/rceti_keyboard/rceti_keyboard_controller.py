import sys
import termios
import tty
from select import select
from std_msgs.msg import Float32
import rclpy
# import keyboard
from rclpy.node import Node

class RcetiKeyboardController(Node):
    def __init__(self):
        super().__init__('rceti_keyboard')
        self.x_position_ = self.create_publisher(Float32, 'rceti/x_position', 10)
        self.z_position_ = self.create_publisher(Float32, 'rceti/z_position', 10)
        self.pitch_angle_ = self.create_publisher(Float32, 'rceti/pitch_angle', 10)
        timer_period = 0.5  # seconds
        self.x_timer = self.create_timer(timer_period, self.x_position_callback)
        self.z_timer = self.create_timer(timer_period, self.z_position_callback)
        self.pitch_timer = self.create_timer(timer_period, self.pitch_angle_callback)
        self.declare_parameter('key_timeout', 0.5)
        self.i = 0

    def detectKey(self, settings, timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)

    def x_position_callback(self):
        settings = self.saveTerminalSettings()
        key_timeout = self.get_parameter("key_timeout").get_parameter_value().double_value
        key = self.detectKey(settings, key_timeout)
        print(key)

    def z_position_callback(self):
        msg = Float32()
        # THIS IS WHERE THE Z POSITION ON THE KEYBOARD SIDE

    def pitch_angle_callback(self):
        msg = Float32()
        # THIS IS WHERE THE PITCH ANGLE ON THE KEYBOARD SIDE

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = RcetiKeyboardController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()