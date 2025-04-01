import sys
import termios
import tty
from select import select
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

class RcetiKeyboardController(Node):
    def __init__(self):
        super().__init__('rceti_keyboard')
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.x_position = 0.0  # Initialize x position
        self.z_position = 0.0  # Initialize z position
        self.pitch_angle = 0.0  # Initialize pitch angle
        
        # Timer for keyboard input
        self.keyboard_timer = self.create_timer(0.01, self.keyboard_callback)
        
        # Timer for continuous joint state publishing
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)

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
        """Handles keyboard input and updates joint states."""
        settings = self.saveTerminalSettings()
        key_timeout = self.get_parameter("key_timeout").get_parameter_value().double_value
        key = self.detectKey(settings, key_timeout)

        if key:
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
            elif key == 'l':  # Decrease pitch angle
                self.pitch_angle -= 0.1
            elif key == '\x03':  # Ctrl+C to exit
                rclpy.shutdown()
                sys.exit(0)

    def publish_joint_states(self):
        """Publishes the updated joint states."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['x_actuator_to_x_moving', 'z_actuator_to_z_moving', 'z_moving_to_pitch_actuator']
        joint_state.position = [self.x_position, self.z_position, self.pitch_angle]
        self.joint_state_publisher.publish(joint_state)

        self.get_logger().info(f"Published Joint States: X: {self.x_position}, Z: {self.z_position}, Pitch: {self.pitch_angle}")


def main(args=None):
    """Main function to initialize the ROS node."""
    rclpy.init(args=args)
    keyboard_controller = RcetiKeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
