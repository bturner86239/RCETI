import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import lgpio

TOPIC_SUBSCRIPTION_BUFFER = 5

class RCETIRobotController(Node):
    """
    ...
    """
    def __init__(self):
        """
        ...
        """
        super().__init__('rceti_controller')

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            TOPIC_SUBSCRIPTION_BUFFER
        )

        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.0

        self.x_velocity = 0.0
        self.z_velocity = 0.0
        self.pitch_velocity = 0.0

        self.x_target = 0.0
        self.z_target = 0.0
        self.pitch_target = 0.0

        self.x_step_accumulator = 0.0
        self.z_step_accumulator = 0.0

        self.X_DIRECTION_PIN = 21
        self.X_PULSE_PIN = 20
        self.Z_DIRECTION_PIN = 19
        self.Z_PULSE_PIN = 18

        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.X_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.X_PULSE_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_PULSE_PIN)

        self.steps_per_mm_x = 40
        self.steps_per_mm_z = 40

        self.motor_timer = self.create_timer(0.005, self.motor_control_update)

    def joint_state_callback(self, msg):
        """
        ...
        """
        try:
            x_index = msg.name.index('x_actuator_to_x_moving')
            z_index = msg.name.index('z_actuator_to_z_moving')
            pitch_index = msg.name.index('z_moving_to_pitch_actuator')

            self.x_target = msg.position[x_index]
            self.z_target = msg.position[z_index]
            self.pitch_target = msg.position[pitch_index]
            
            # Update velocities if provided
            if len(msg.velocity) >= 3:
                self.x_velocity = msg.velocity[x_index]
                self.z_velocity = msg.velocity[z_index]
                self.pitch_velocity = msg.velocity[pitch_index]
                
                self.get_logger().debug(
                    f"Received velocities: X={self.x_velocity:.4f}, "
                    f"Z={self.z_velocity:.4f}, Pitch={self.pitch_velocity:.4f}"
                )
                
        except ValueError as e:
            self.get_logger().error(f"Joint name not found in joint_states: {e}")

    def motor_control_update(self):
        """
        ...
        """
        dt = 0.005 
        
        x_mm_increment = self.x_velocity * dt * 1000
        x_steps = x_mm_increment * self.steps_per_mm_x
        
        self.x_step_accumulator += x_steps
        x_steps_to_take = int(self.x_step_accumulator)
        self.x_step_accumulator -= x_steps_to_take
        
        if x_steps_to_take != 0:
            direction = 1 if x_steps_to_take > 0 else 0
            self.move_stepper(
                abs(x_steps_to_take),
                direction,
                self.X_DIRECTION_PIN,
                self.X_PULSE_PIN,
                0.0001
            )
            
        z_mm_increment = self.z_velocity * dt * 1000
        z_steps = z_mm_increment * self.steps_per_mm_z
        
        self.z_step_accumulator += z_steps
        z_steps_to_take = int(self.z_step_accumulator)
        self.z_step_accumulator -= z_steps_to_take
        
        if z_steps_to_take != 0:
            direction = 1 if z_steps_to_take > 0 else 0
            self.move_stepper(
                abs(z_steps_to_take),
                direction,
                self.Z_DIRECTION_PIN,
                self.Z_PULSE_PIN,
                0.0001
            )
        
        if x_steps_to_take != 0 or z_steps_to_take != 0:
            self.x_position += x_steps_to_take / (self.steps_per_mm_x * 1000)
            self.z_position += z_steps_to_take / (self.steps_per_mm_z * 1000)
            
            if hasattr(self, 'update_counter'):
                self.update_counter += 1
            else:
                self.update_counter = 0
                
            if self.update_counter % 100 == 0:
                self.get_logger().info(
                    f"Current position: X={self.x_position:.4f}, Z={self.z_position:.4f} | "
                    f"Target: X={self.x_target:.4f}, Z={self.z_target:.4f}"
                )

    def move_stepper(self, steps, direction, direction_pin, pulse_pin, delay=0.0001):
        """
        ...
        """
        lgpio.gpio_write(self.chip, direction_pin, direction)
        
        if steps <= 2:
            for _ in range(steps):
                lgpio.gpio_write(self.chip, pulse_pin, 1)
                time.sleep(delay)
                lgpio.gpio_write(self.chip, pulse_pin, 0)
                time.sleep(delay)
        else:
            self.get_logger().debug(
                f"Moving stepper: steps={steps}, direction={direction}"
            )
            
            for _ in range(steps):
                lgpio.gpio_write(self.chip, pulse_pin, 1)
                time.sleep(delay)
                lgpio.gpio_write(self.chip, pulse_pin, 0)
                time.sleep(delay)

    def __del__(self):
        """Clean up GPIO on shutdown"""
        try:
            lgpio.gpiochip_close(self.chip)
        except:
            pass


def main(args=None):
    """
    ...
    """
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()