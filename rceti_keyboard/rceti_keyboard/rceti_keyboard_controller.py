import math
import sys
import termios
import tty
import time
from select import select
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

FIVE_DEGREES_FUNCTIONAL = 0.050208333
class RcetiKeyboardController(Node):
    """RcetiKeyboardController is a ROS 2 node that handles keyboard input to control the RCETI robot's joint states.

    Args:
        Node (Node): The node of the ROS 2 system that handles the keyboard input and joint state publishing.
    """

    def __init__(self):
        """Initializes the RcetiKeyboardController node, sets up the publisher for joint states, and initializes parameters.
        """
        super().__init__('rceti_keyboard')
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.x_position = 0.0  # Initialize x position
        self.z_position = 0.0  # Initialize z position
        self.pitch_angle = 0.730 # start at the maximum
        self.continuum_motor_1_angle = 0.0
        self.continuum_motor_2_angle = 0.0

        self.MAX_X_POSITION = 0.309  # Maximum x position
        self.MIN_X_POSITION = 0.0  # Minimum x position
        self.MAX_Z_POSITION = 0.309  # Maximum z position
        self.MIN_Z_POSITION = 0.0  # Minimum z position
        self.MAX_PITCH_ANGLE = 0.730 # Maximum pitch angle (in radians)
        self.MIN_PITCH_ANGLE = -0.475 # Minimum pitch angle (in radians)
        self.MAX_CONTINUUM_MOTOR_ANGLE = 0.5
        self.MIN_CONTINUUM_MOTOR_ANGLE = -0.5
        
        # Timer for keyboard input
        self.keyboard_timer = self.create_timer(0.01, self.keyboard_callback)
        
        # Timer for continuous joint state publishing
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)

        self.declare_parameter('key_timeout', 0.1)

    def detectKey(self, settings, timeout):
        """Detects a key press from the keyboard with a timeout.

        Args:
            settings (String): The settings of the terminal for keyboard input.
            timeout (float): The timeout duration in seconds.

        Returns:
            string: The key pressed or an empty string if no key was pressed within the timeout.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)  # Read the first character
            if key == '\x1b':  # Escape character
                key += sys.stdin.read(1)  # Read the '['
                key += sys.stdin.read(1)  # Read the final character (e.g., 'A', 'B', 'C', or 'D')
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            time.sleep(0.05)
            return key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return ''

    def saveTerminalSettings(self):
        """Saves the current terminal settings for keyboard input

        Returns:
            string: The current terminal settings
        """
        return termios.tcgetattr(sys.stdin)

    def keyboard_callback(self):
        """Handles keyboard input to control the robot's joint states."""
        settings = self.saveTerminalSettings()
        key_timeout = self.get_parameter("key_timeout").get_parameter_value().double_value
        key = self.detectKey(settings, key_timeout)

        if key:
            if key == 'd':  # Move left (decrease x)
                if (self.x_position - 0.01) < self.MIN_X_POSITION:
                    self.get_logger().info("Cannot move right, at minimum x position")
                else:
                    self.x_position -= 0.01
            elif key == 'a':  # Move right (increase x)
                if (self.x_position + 0.01) > self.MAX_X_POSITION:
                    self.get_logger().info("Cannot move left, at maximum x position")
                else:
                    self.x_position += 0.01
            elif key == 's':  # Move forward (increase z)
                if (self.z_position + 0.01) > self.MAX_Z_POSITION:
                    self.get_logger().info("Cannot move down, at minimum z position")
                else:
                    self.z_position += 0.01
            elif key == 'w':  # Move backward (decrease z)
                if (self.z_position - 0.01) < self.MIN_Z_POSITION:
                    self.get_logger().info("Cannot move left, at minimum x position")
                else:
                    self.z_position -= 0.01
            elif key == 'p':  # Decrease pitch angle
                if (self.pitch_angle - FIVE_DEGREES_FUNCTIONAL) < self.MIN_PITCH_ANGLE:
                    self.get_logger().info("Cannot move down, at minimum pitch angle")
                else:
                    self.pitch_angle -= FIVE_DEGREES_FUNCTIONAL
            elif key == 'l':  # Increase pitch angle
                if(self.pitch_angle + FIVE_DEGREES_FUNCTIONAL) > self.MAX_PITCH_ANGLE:
                    self.get_logger().info("Cannot move up, at maximum pitch angle")
                else:
                    self.pitch_angle += FIVE_DEGREES_FUNCTIONAL
            elif key == 'l':  # Decrease pitch angle
                if (self.pitch_angle - FIVE_DEGREES_FUNCTIONAL) < self.MIN_PITCH_ANGLE:
                    self.get_logger().info("Cannot move down, at minimum pitch angle")
                else:
                    self.pitch_angle -= FIVE_DEGREES_FUNCTIONAL
            elif key == '\x1b[A': #Up Arrow
                if (self.continuum_motor_1_angle + FIVE_DEGREES_FUNCTIONAL) > self.MAX_CONTINUUM_MOTOR_ANGLE:
                    self.get_logger().info("Cannot move up, at maximum continuum motor angle")
                else:
                    self.continuum_motor_1_angle += FIVE_DEGREES_FUNCTIONAL
            elif key == '\x1b[B': #Down Arrow
                if (self.continuum_motor_1_angle - FIVE_DEGREES_FUNCTIONAL) < self.MIN_CONTINUUM_MOTOR_ANGLE:
                    self.get_logger().info("Cannot move down, at minimum continuum motor angle")
                else:
                    self.continuum_motor_1_angle -= FIVE_DEGREES_FUNCTIONAL
            elif key == '\x1b[C': #Right Arrow
                self.get_logger().info("Right arrow key detected")
                if (self.continuum_motor_2_angle + FIVE_DEGREES_FUNCTIONAL) > self.MAX_CONTINUUM_MOTOR_ANGLE:
                    self.get_logger().info("Cannot move up, at maximum continuum motor angle")
                else:
                    self.continuum_motor_2_angle += FIVE_DEGREES_FUNCTIONAL
            elif key == '\x1b[D': #Left Arrow
                if (self.continuum_motor_2_angle - FIVE_DEGREES_FUNCTIONAL) < self.MIN_CONTINUUM_MOTOR_ANGLE:
                    self.get_logger().info("Cannot move down, at minimum continuum motor angle")
                else:
                    self.continuum_motor_2_angle -= FIVE_DEGREES_FUNCTIONAL
            elif key == '\x03':  # Ctrl+C to exit
                rclpy.shutdown()
                sys.exit(0)

    def publish_joint_states(self):
        """Publishes the current joint states to the /joint_states topic.
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['x_actuator_to_x_slider', 'z_actuator_to_z_slider', 'z_slider_to_pitch_servo', 'continuum_motor_1', 'continuum_motor_2']
        joint_state.position = [self.x_position, self.z_position, self.pitch_angle, self.continuum_motor_1_angle, self.continuum_motor_2_angle]
        self.joint_state_publisher.publish(joint_state)

        self.get_logger().info(f"Published Joint States: X: {self.x_position}, Z: {self.z_position}, Pitch: {self.pitch_angle}, Continuum Motor 1: {self.continuum_motor_1_angle}, Continuum Motor 2: {self.continuum_motor_2_angle}")


def main(args=None):
    """Main function to initialize the RcetiKeyboardController node and start the ROS 2 event loop.

    Args:
        args (N/A optional):Defaults to None, shouldn't be set to anything
    """
    rclpy.init(args=args)
    keyboard_controller = RcetiKeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
