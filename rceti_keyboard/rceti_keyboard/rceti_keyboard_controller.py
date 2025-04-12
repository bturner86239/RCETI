import sys
import time
import select
import termios
import tty
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

class RcetiKeyboardController(Node):
    """
    ...
    """
    def __init__(self):
        """
        ...
        """
        super().__init__('rceti_keyboard')
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.0

        self.x_velocity = 0.0
        self.z_velocity = 0.0
        self.pitch_velocity = 0.0

        self.x_target_velocity = 0.0
        self.z_target_velocity = 0.0
        self.pitch_target_velocity = 0.0

        self.max_velocity = 0.05
        self.acceleration = 0.1
        self.deceleration = 0.2

        self.MAX_X_POSITION = 0.1545
        self.MIN_X_POSITION = -0.1545
        self.MAX_Z_POSITION = 0.309
        self.MIN_Z_POSITION = 0.0
        self.MAX_PITCH_ANGLE = 1.57
        self.MIN_PITCH_ANGLE = -1.57
        
        self.key_states = {'w': False, 'a': False, 's': False, 'd': False, 
                           'p': False, 'l': False, ' ': False}
        self.last_key = None

        self.settings = termios.tcgetattr(sys.stdin)

        self.keyboard_timer = self.create_timer(0.01, self.keyboard_callback)
        self.motion_timer = self.create_timer(0.01, self.motion_update)
        self.publish_timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info("Keyboard Controller Started")
        self.get_logger().info("Use WASD to control X/Z axes, P/L for pitch, Space to stop")

    def detect_keys(self):
        """
        ...
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], 0.01)
        
        if rlist:
            key = sys.stdin.read(1)
            
            if key == '\x03' or key == '\x1b': 
                self.get_logger().info("Exit requested")
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                rclpy.shutdown()
                sys.exit(0)
                
            if key in self.key_states:
                self.key_states[key] = not self.key_states[key]
                self.last_key = key
                self.get_logger().debug(f"Key {key} is {'pressed' if self.key_states[key] else 'released'}")
                
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def keyboard_callback(self):
        """
        ...
        """
        self.detect_keys()
        
        self.x_target_velocity = 0.0
        self.z_target_velocity = 0.0
        self.pitch_target_velocity = 0.0
        
        if self.key_states['a']:
            self.x_target_velocity = self.max_velocity  # Move right
        if self.key_states['d']:
            self.x_target_velocity = -self.max_velocity  # Move left
            
        if self.key_states['w']:
            self.z_target_velocity = -self.max_velocity  # Move up
        if self.key_states['s']:
            self.z_target_velocity = self.max_velocity   # Move down
            
        if self.key_states['p']:
            self.pitch_target_velocity = self.max_velocity   # Increase pitch
        if self.key_states['l']:
            self.pitch_target_velocity = -self.max_velocity  # Decrease pitch
            
        if self.key_states[' ']:  # Immediate stop
            self.x_target_velocity = 0.0
            self.z_target_velocity = 0.0
            self.pitch_target_velocity = 0.0
            self.x_velocity = 0.0
            self.z_velocity = 0.0
            self.pitch_velocity = 0.0

    def motion_update(self):
        """
        ...
        """
        dt = 0.01  # Time step matching timer frequency
        
        # X-axis velocity
        if self.x_velocity < self.x_target_velocity:
            # Accelerate
            self.x_velocity = min(
                self.x_velocity + self.acceleration * dt,
                self.x_target_velocity
            )
        elif self.x_velocity > self.x_target_velocity:
            # Decelerate
            self.x_velocity = max(
                self.x_velocity - self.deceleration * dt,
                self.x_target_velocity
            )
            
        # Z-axis velocity
        if self.z_velocity < self.z_target_velocity:
            # Accelerate
            self.z_velocity = min(
                self.z_velocity + self.acceleration * dt,
                self.z_target_velocity
            )
        elif self.z_velocity > self.z_target_velocity:
            # Decelerate
            self.z_velocity = max(
                self.z_velocity - self.deceleration * dt,
                self.z_target_velocity
            )
            
        # Pitch velocity update
        if self.pitch_velocity < self.pitch_target_velocity:
            # Accelerate
            self.pitch_velocity = min(
                self.pitch_velocity + self.acceleration * dt,
                self.pitch_target_velocity
            )
        elif self.pitch_velocity > self.pitch_target_velocity:
            # Decelerate
            self.pitch_velocity = max(
                self.pitch_velocity - self.deceleration * dt,
                self.pitch_target_velocity
            )
            
        self.x_position += self.x_velocity * dt
        self.z_position += self.z_velocity * dt
        self.pitch_angle += self.pitch_velocity * dt
        
        self.x_position = max(min(self.x_position, self.MAX_X_POSITION), self.MIN_X_POSITION)
        self.z_position = max(min(self.z_position, self.MAX_Z_POSITION), self.MIN_Z_POSITION)
        self.pitch_angle = max(min(self.pitch_angle, self.MAX_PITCH_ANGLE), self.MIN_PITCH_ANGLE)
        
    def publish_joint_states(self):
        """
        ...
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['x_actuator_to_x_moving', 'z_actuator_to_z_moving', 'z_moving_to_pitch_actuator']
        joint_state.position = [self.x_position, self.z_position, self.pitch_angle]
        joint_state.velocity = [self.x_velocity, self.z_velocity, self.pitch_velocity]
        
        self.joint_state_publisher.publish(joint_state)
        self.get_logger().info(
            f"Position: X={self.x_position:.4f}, Z={self.z_position:.4f}, P={self.pitch_angle:.4f} | "
            f"Velocity: X={self.x_velocity:.4f}, Z={self.z_velocity:.4f}, P={self.pitch_velocity:.4f}"
        )


def main(args=None):
    """
    ...
    """
    rclpy.init(args=args)
    keyboard_controller = RcetiKeyboardController()
    try:
        rclpy.spin(keyboard_controller)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keyboard_controller.settings)
        keyboard_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()