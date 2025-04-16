import sys
import termios
import tty
from select import select
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

class RcetiKeyboardController(Node):
    """
    Keyboard-based calibration tool to manually position the X and Z actuators and adjust pitch.
    Publishes joint states for visualization or future mapping to real actuator commands.
    """

    def __init__(self):
        super().__init__('rceti_keyboard')

        # Publisher to joint_states topic
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Internal joint state variables
        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.0

        # Terminal settings saved once
        self.settings = termios.tcgetattr(sys.stdin)

        # Parameters
        self.declare_parameter('key_timeout', 0.1)

        # Timers
        self.keyboard_timer = self.create_timer(0.01, self.keyboard_callback)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info("""
RCETI Calibration Mode - Keyboard Controls:
  a/d - Move X actuator (left/right)
  w/s - Move Z actuator (up/down)
  l/p - Adjust pitch (down/up)
  r   - Reset all joints to zero
  Ctrl+C - Exit
""")

    def detect_key(self, timeout):
        """Detects a key press with timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_callback(self):
        """Processes keyboard input and updates joint positions."""
        key_timeout = self.get_parameter("key_timeout").get_parameter_value().double_value
        key = self.detect_key(key_timeout)

        if key:
            if key == 'd':  # Move left (decrease x)
                self.x_position -= 0.01
            elif key == 'a':  # Move right (increase x)
                self.x_position += 0.01
            elif key == 's':  # Move forward (increase z)
                self.z_position += 0.01
            elif key == 'w':  # Move backward (decrease z)
                self.z_position -= 0.01
            elif key == 'p':  # Increase pitch angle
                self.pitch_angle += 0.1
            elif key == 'l':  # Decrease pitch angle
                self.pitch_angle -= 0.1
            elif key == 'r':  # Reset all joint states
                self.x_position = 0.0
                self.z_position = 0.0
                self.pitch_angle = 0.0
                self.get_logger().info("Joint states reset to zero.")
            elif key == '\x03':  # Ctrl+C to exit
                rclpy.shutdown()
                sys.exit(0)

            # Only log on actual key press that changed state
            self.get_logger().info(f"Updated Joint States: X: {self.x_position:.3f}, Z: {self.z_position:.3f}, Pitch: {self.pitch_angle:.2f}")

    def publish_joint_states(self):
        """Publishes current joint state values."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['x_actuator_to_x_slider', 'z_actuator_to_z_slider', 'z_slider_to_pitch_servo']
        joint_state.position = [self.x_position, self.z_position, self.pitch_angle]
        self.joint_state_publisher.publish(joint_state)

def main(args=None):
    """Main function to start the calibration node."""
    rclpy.init(args=args)
    keyboard_controller = RcetiKeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()
