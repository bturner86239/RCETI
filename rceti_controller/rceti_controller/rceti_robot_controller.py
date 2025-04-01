import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time  # for delays/testing
import lgpio

# Constants:
# Maximum amount of request stepper motors will take at any given time
TOPIC_SUBSCRIPTION_BUFFER = 5

class RCETIRobotController(Node):

    def __init__(self):
        super().__init__('rceti_controller')

        # Subscribe to the /joint_states topic
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            TOPIC_SUBSCRIPTION_BUFFER
        )

        # Position tracking
        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.0

        # GPIO pin definitions
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

        # Define stepper steps per unit in X and Z directions
        self.steps_per_mm_x = 40
        self.steps_per_mm_z = 40

    def joint_state_callback(self, msg):
        """Callback to handle joint state updates."""
        # Extract joint positions from the message
        try:
            x_index = msg.name.index('x_actuator_to_x_moving')
            z_index = msg.name.index('z_actuator_to_z_moving')
            pitch_index = msg.name.index('z_moving_to_pitch_actuator')

            new_x_position = msg.position[x_index]
            new_z_position = msg.position[z_index]
            new_pitch_angle = msg.position[pitch_index]

            # Handle X-axis movement
            if self.x_position != new_x_position:
                self.get_logger().info(f"Moving X to {new_x_position}")
                steps = int(abs(new_x_position - self.x_position) * 1000 * self.steps_per_mm_x)
                direction = 1 if new_x_position > self.x_position else 0
                self.move_stepper(steps, direction, self.X_DIRECTION_PIN, self.X_PULSE_PIN)
                self.x_position = new_x_position

            # Handle Z-axis movement
            if self.z_position != new_z_position:
                self.get_logger().info(f"Moving Z to {new_z_position}")
                steps = int(abs(new_z_position - self.z_position) * 1000 * self.steps_per_mm_z)
                direction = 1 if new_z_position > self.z_position else 0
                self.move_stepper(steps, direction, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
                self.z_position = new_z_position

            # Handle pitch angle (if implemented in hardware)
            if self.pitch_angle != new_pitch_angle:
                self.get_logger().info(f"Adjusting pitch to {new_pitch_angle}")
                # TODO: Implement pitch angle movement if hardware supports it
                self.pitch_angle = new_pitch_angle

        except ValueError as e:
            self.get_logger().error(f"Joint name not found in joint_states: {e}")

    def move_stepper(self, steps, direction, direction_pin, pulse_pin, delay=0.001):
        """Moves the stepper motor."""
        self.get_logger().info(f"Moving stepper: steps={steps}, direction={direction}, direction_pin={direction_pin}, pulse_pin={pulse_pin}")
        lgpio.gpio_write(self.chip, direction_pin, direction)

        for _ in range(steps):
            lgpio.gpio_write(self.chip, pulse_pin, 1)
            time.sleep(delay)
            lgpio.gpio_write(self.chip, pulse_pin, 0)
            time.sleep(delay)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
