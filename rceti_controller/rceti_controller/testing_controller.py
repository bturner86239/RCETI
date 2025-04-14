import rclpy
from rclpy.node import Node
import time
import lgpio
from adafruit_servokit import ServoKit

class RCETIRobotController(Node):
    """A simplified ROS 2 node for controlling the RCETI robot's stepper motors and servos.
    
    This controller provides basic movement functions for:
    - X-axis movement (stepper motor)
    - Z-axis movement (stepper motor)
    - Pitch angle adjustment (servo)
    - Internal forward/backward control (servo)
    - Internal left/right control (servo)
    """

    def __init__(self):
        """Initializes the RCETIRobotController node and sets up GPIO pins and servos."""
        super().__init__('simplified_rceti_controller')
        
        # Position tracking
        self.x_position = 0.0  # in mm
        self.z_position = 0.0  # in mm
        self.pitch_angle = 0.0  # in degrees - will be mapped to radians for servo control
        self.internal_fb = 0.0  # forward/backward servo position in radians
        self.internal_lr = 0.0  # left/right servo position in radians

        # GPIO pin definitions for stepper motors
        self.X_DIRECTION_PIN = 21
        self.X_PULSE_PIN = 20
        self.Z_DIRECTION_PIN = 19
        self.Z_PULSE_PIN = 18

        # Open the GPIO chip and set the pins as output
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.X_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.X_PULSE_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_PULSE_PIN)

        # Initialize servo controller
        self.servo_kit = ServoKit(channels=16)
        self.PITCH_SERVO = 0  # Servo channel for pitch
        self.FB_SERVO = 1     # Servo channel for forward/backward movement
        self.LR_SERVO = 2     # Servo channel for left/right movement
        
        # Set servo initial positions
        self.servo_kit.servo[self.PITCH_SERVO].angle = self.pitch_angle
        self.servo_kit.servo[self.FB_SERVO].angle = self.internal_fb
        self.servo_kit.servo[self.LR_SERVO].angle = self.internal_lr

        # Define stepper steps per unit
        self.steps_per_mm_x = 40
        self.steps_per_mm_z = 40
        
        # Movement limits
        self.x_min = 0.0
        self.x_max = 309.0  # mm
        self.z_min = 0.0
        self.z_max = 309.0  # mm
        self.pitch_min = -1.57  # radians (approximately -90 degrees)
        self.pitch_max = 1.57   # radians (approximately 90 degrees)
        self.internal_fb_min = -1.57  # radians
        self.internal_fb_max = 1.57   # radians
        self.internal_lr_min = -1.57  # radians
        self.internal_lr_max = 1.57   # radians
        
        self.get_logger().info("Robot controller initialized")

    def move_stepper(self, steps, direction, direction_pin, pulse_pin, delay=0.001):
        """Moves a stepper motor the specified number of steps in the given direction.

        Args:
            steps (int): Number of steps to move the motor
            direction (int): Direction to move (0 for backward, 1 for forward)
            direction_pin (int): GPIO pin controlling the direction
            pulse_pin (int): GPIO pin sending pulses to the motor
            delay (float, optional): Delay between steps (controls speed). Defaults to 0.001.
        """
        lgpio.gpio_write(self.chip, direction_pin, direction)

        for _ in range(steps):
            lgpio.gpio_write(self.chip, pulse_pin, 1)
            time.sleep(delay)
            lgpio.gpio_write(self.chip, pulse_pin, 0)
            time.sleep(delay)

    def move_x_absolute(self, position, speed=0.001):
        """Moves the X-axis to an absolute position in mm.

        Args:
            position (float): Target position in mm
            speed (float, optional): Movement speed (delay between steps). Defaults to 0.001.
        """
        # Ensure position is within limits
        position = max(self.x_min, min(position, self.x_max))
        
        # Calculate steps to move
        distance = position - self.x_position
        steps = int(abs(distance) * self.steps_per_mm_x)
        direction = 1 if distance > 0 else 0
        
        self.get_logger().info(f"Moving X to {position} mm")
        self.move_stepper(steps, direction, self.X_DIRECTION_PIN, self.X_PULSE_PIN, speed)
        self.x_position = position

    def move_x_relative(self, distance, speed=0.001):
        """Moves the X-axis by a relative distance in mm.

        Args:
            distance (float): Distance to move in mm (positive or negative)
            speed (float, optional): Movement speed. Defaults to 0.001.
        """
        new_position = self.x_position + distance
        self.move_x_absolute(new_position, speed)

    def move_z_absolute(self, position, speed=0.001):
        """Moves the Z-axis to an absolute position in mm.

        Args:
            position (float): Target position in mm
            speed (float, optional): Movement speed (delay between steps). Defaults to 0.001.
        """
        # Ensure position is within limits
        position = max(self.z_min, min(position, self.z_max))
        
        # Calculate steps to move
        distance = position - self.z_position
        steps = int(abs(distance) * self.steps_per_mm_z)
        direction = 1 if distance > 0 else 0
        
        self.get_logger().info(f"Moving Z to {position} mm")
        self.move_stepper(steps, direction, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN, speed)
        self.z_position = position

    def move_z_relative(self, distance, speed=0.001):
        """Moves the Z-axis by a relative distance in mm.

        Args:
            distance (float): Distance to move in mm (positive or negative)
            speed (float, optional): Movement speed. Defaults to 0.001.
        """
        new_position = self.z_position + distance
        self.move_z_absolute(new_position, speed)

    def set_pitch_angle(self, angle_rad):
        """Sets the pitch servo to a specific angle.

        Args:
            angle_rad (float): Angle in radians
        """
        # Ensure angle is within valid range
        angle_rad = max(self.pitch_min, min(angle_rad, self.pitch_max))
        
        # Convert radians to degrees for servo control (servos use 0-180 degrees)
        # Map from [-1.57, 1.57] radians to [0, 180] degrees
        servo_angle = ((angle_rad - self.pitch_min) / (self.pitch_max - self.pitch_min)) * 180
        
        self.get_logger().info(f"Setting pitch to {angle_rad:.2f} radians ({servo_angle:.1f} degrees)")
        self.servo_kit.servo[self.PITCH_SERVO].angle = servo_angle
        self.pitch_angle = angle_rad

    def set_internal_fb(self, angle_rad):
        """Sets the internal forward/backward servo to a specific angle.

        Args:
            angle_rad (float): Angle in radians
        """
        # Ensure angle is within valid range
        angle_rad = max(self.internal_fb_min, min(angle_rad, self.internal_fb_max))
        
        # Convert radians to degrees for servo control
        servo_angle = ((angle_rad - self.internal_fb_min) / (self.internal_fb_max - self.internal_fb_min)) * 180
        
        self.get_logger().info(f"Setting internal F/B to {angle_rad:.2f} radians ({servo_angle:.1f} degrees)")
        self.servo_kit.servo[self.FB_SERVO].angle = servo_angle
        self.internal_fb = angle_rad

    def set_internal_lr(self, angle_rad):
        """Sets the internal left/right servo to a specific angle.

        Args:
            angle_rad (float): Angle in radians
        """
        # Ensure angle is within valid range
        angle_rad = max(self.internal_lr_min, min(angle_rad, self.internal_lr_max))
        
        # Convert radians to degrees for servo control
        servo_angle = ((angle_rad - self.internal_lr_min) / (self.internal_lr_max - self.internal_lr_min)) * 180
        
        self.get_logger().info(f"Setting internal L/R to {angle_rad:.2f} radians ({servo_angle:.1f} degrees)")
        self.servo_kit.servo[self.LR_SERVO].angle = servo_angle
        self.internal_lr = angle_rad

    def home_position(self):
        """Moves all motors to their home positions."""
        self.get_logger().info("Moving to home position")
        
        # Set servos to middle positions (0 radians)
        self.set_pitch_angle(0.0)
        self.set_internal_fb(0.0)
        self.set_internal_lr(0.0)
        
        # Move steppers to home position
        self.move_z_absolute(0)  # Z should go first for safety
        self.move_x_absolute(0)
        
        self.get_logger().info("Home position reached")

    def shutdown(self):
        """Safely shuts down the robot controller."""
        self.get_logger().info("Shutting down robot controller")
        
        # Release GPIO resources
        lgpio.gpiochip_close(self.chip)

    def move_to_position(self, x, z, pitch, fb, lr, speed=0.001):
        """Convenience function to move all axes to specified positions.
        
        Args:
            x (float): X-axis position in mm
            z (float): Z-axis position in mm
            pitch (float): Pitch angle in radians
            fb (float): Forward/backward angle in radians
            lr (float): Left/right angle in radians
            speed (float, optional): Movement speed for steppers. Defaults to 0.001.
        """
        self.get_logger().info(f"Moving to position: X={x}, Z={z}, Pitch={pitch}, FB={fb}, LR={lr}")
        
        # Set servo positions first (they're faster)
        self.set_pitch_angle(pitch)
        self.set_internal_fb(fb)
        self.set_internal_lr(lr)
        
        # Then move the steppers
        self.move_z_absolute(z, speed)
        self.move_x_absolute(x, speed)
        
        self.get_logger().info("Position reached")
    
    def preset_position_1(self):
        """Move to a preset position (example)"""
        self.move_to_position(100, 100, 0.5, 0.0, 0.0)
    
    def preset_position_2(self):
        """Move to another preset position (example)"""
        self.move_to_position(200, 200, -0.5, 0.5, 0.5)
    
    def create_grid_pattern(self, x_start, x_end, z_start, z_end, rows, cols):
        """Move in a grid pattern across the specified area.
        
        Args:
            x_start (float): Starting X position
            x_end (float): Ending X position
            z_start (float): Starting Z position
            z_end (float): Ending Z position
            rows (int): Number of rows in the grid
            cols (int): Number of columns in the grid
        """
        x_step = (x_end - x_start) / (cols - 1) if cols > 1 else 0
        z_step = (z_end - z_start) / (rows - 1) if rows > 1 else 0
        
        self.get_logger().info(f"Starting grid pattern: {rows}x{cols}")
        
        # Move to starting position
        self.move_x_absolute(x_start)
        self.move_z_absolute(z_start)
        
        # Loop through the grid
        for row in range(rows):
            # Determine Z position for this row
            z_pos = z_start + (row * z_step)
            self.move_z_absolute(z_pos)
            
            # If even row, go left to right, else right to left
            x_positions = [x_start + (col * x_step) for col in range(cols)]
            if row % 2 == 1:
                x_positions.reverse()  # Alternate direction for efficiency
                
            for x_pos in x_positions:
                self.move_x_absolute(x_pos)
                time.sleep(0.1)  # Pause at each grid point
        
        self.get_logger().info("Grid pattern completed")


def main(args=None):
    """Main function to initialize and run the robot controller."""
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    
    # Example of how to use the controller (uncomment to test)
    # robot_controller.home_position()
    # robot_controller.move_to_position(100, 50, 0.5, 0.0, 0.0)
    # robot_controller.create_grid_pattern(50, 250, 50, 250, 5, 5)
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.shutdown()
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()