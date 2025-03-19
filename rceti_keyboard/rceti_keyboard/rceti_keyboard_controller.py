import sys
import termios
import tty
from select import select
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node

class RcetiKeyboardController(Node):
    def __init__(self):
        super().__init__('rceti_keyboard')
        self.x_position_publisher = self.create_publisher(Float32, 'rceti/x_position', 10)
        self.z_position_publisher = self.create_publisher(Float32, 'rceti/z_position', 10)
        self.pitch_angle_publisher = self.create_publisher(Float32, 'rceti/pitch_angle', 10)
        
        self.x_position = 0.0  # Initialize x position
        self.z_position = 0.0  # Initialize z position
        self.pitch_angle = 0.0 # Initialize pitch angle (NOTE: CONTROLLED BY 'P' AND L' KEYS)
        
        timer_period = 0.01  # Adjust the timer period for responsiveness
        self.timer = self.create_timer(timer_period, self.keyboard_callback)

        self.declare_parameter('key_timeout', 0.1)

    def detectKey(self, settings, timeout):
        """Detects a single key press with a timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def saveTerminalSettings(self):
        """Saves the terminal settings."""
        return termios.tcgetattr(sys.stdin)

    def keyboard_callback(self):
        """Handles keyboard input and publishes to topics."""
        settings = self.saveTerminalSettings()
        key_timeout = self.get_parameter("key_timeout").get_parameter_value().double_value
        key = self.detectKey(settings, key_timeout)

        if key:
            # Changed to 0.01 instead of .1, since .1 is 10cm
            if key == 'a':  # Move left (decrease x)
                self.x_position -= 0.01
            elif key == 'd':  # Move right (increase x)
                self.x_position += 0.01
            elif key == 'w':  # Move forward (increase z)
                self.z_position += 0.01
            elif key == 's':  # Move backward (decrease z)
                self.z_position -= 0.01
            elif key == 'p':  # Increase pitch angle
                self.pitch_angle += 0.1
            elif key == 'l':  # Increase pitch angle
                self.pitch_angle -= 0.1
            elif key == '\x03':  # Ctrl+C to exit
                rclpy.shutdown()
                sys.exit(0)

            self.publish_positions()

    def publish_positions(self):
        """Publishes the updated x and z positions, as well as the pitch angle, rounded to 2 decimal places."""
        x_msg = Float32()
        x_msg.data = round(self.x_position, 2)  # Round to 2 decimal places
        self.x_position_publisher.publish(x_msg)

        z_msg = Float32()
        z_msg.data = round(self.z_position, 2)  # Round to 2 decimal places
        self.z_position_publisher.publish(z_msg)

        pitch_msg = Float32()
        pitch_msg.data = round(self.pitch_angle, 2)  # Round to 2 decimal places
        self.pitch_angle_publisher.publish(pitch_msg)

        self.get_logger().info(f"Published X: {x_msg.data}, Z: {z_msg.data}, Pitch: {pitch_msg.data}")


def main(args=None):
    """Main function to initialize the ROS node."""
    rclpy.init(args=args)
    keyboard_controller = RcetiKeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
