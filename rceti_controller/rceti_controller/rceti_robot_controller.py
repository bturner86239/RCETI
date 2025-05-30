import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time  # for delays/testing
import lgpio

from adafruit_servokit import ServoKit
import adafruit_motor.servo

kit = ServoKit(channels=16)


# Constants:
# Maximum amount of request stepper motors will take at any given time
TOPIC_SUBSCRIPTION_BUFFER = 5

class RCETIRobotController(Node):
    """RCETIRobotController is a ROS 2 node that controls the RCETI robot's stepper motors and pitch servo.

    Args:
        Node (Node): The node of the ROS 2 system that handles the robot's control logic.
    """

    def __init__(self):
        """Initializes the RCETIRobotController node, subscribes to the /joint_states topic, and sets up GPIO pins for stepper motors and servo motors
        """
        super().__init__('rceti_controller')


        # Initialize servo motors
        self.servo1 = kit.servo[0]       # tilts continuum base
        self.servo2 = kit.servo[1]       # pulls continuum
        self.servo3 = kit.servo[2]       # pulls continuum

        self.servo1.actuation_range = 200
        self.servo1.set_pulse_width_range(500, 2000)

        # May need to adjust 
        self.servo2.actuation_range = 200
        self.servo2.set_pulse_width_range(500, 2000)

        # May need to adjust 
        self.servo3.actuation_range = 200
        self.servo3.set_pulse_width_range(500, 2000)

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
        self.pitch_angle = 0.730

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
        """Handles incoming joint state messages and moves the stepper motors accordingly.
        This function is called whenever a new message is received on the /joint_states topic.
        Moves the x stepper motor, z stepper motor, and pitch servo motor based on the joint states received.

        Args:
            msg (sensor_msgs/JointState.msg): The messages recieved from the /joint_states topic.
        """
        # Extract joint positions from the message
        try:
            x_index = msg.name.index('x_actuator_to_x_slider')
            z_index = msg.name.index('z_actuator_to_z_slider')
            pitch_index = msg.name.index('z_slider_to_pitch_servo')
            continuum_index_1 = msg.name.index('continuum_1')
            continuum_index_2 = msg.name.index('continuum_2')   

            new_x_position = msg.position[x_index]
            new_z_position = msg.position[z_index]
            pitch_angle_msg = msg.position[pitch_index]
            continuum_angle_1 = msg.position[continuum_index_1]
            continuum_angle_2 = msg.position[continuum_index_2]


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
            new_pitch_angle = int( ( (pitch_angle_msg + 0.475) / (1.205) ) * 120 )

            # Handle pitch angle (if implemented in hardware)
            new_continuum_1_pitch_angle = int( ( (continuum_angle_1 + 0.475) / (1.205) ) * 120 )

            # Handle pitch angle (if implemented in hardware)
            new_continuum_2_pitch_angle = int( ( (continuum_angle_2 + 0.475) / (1.205) ) * 120 )

            if (self.servo1.angle != new_pitch_angle): 
                self.get_logger().info(f"Adjusting pitch to {new_pitch_angle}")
                self.servo1.angle = new_pitch_angle

            if (self.servo2.angle != new_continuum_1_pitch_angle): 
                self.get_logger().info(f"Adjusting continuum 1 to {new_continuum_1_pitch_angle}")
                self.servo2.angle = new_continuum_1_pitch_angle

            if (self.servo3.angle != new_continuum_2_pitch_angle): 
                self.get_logger().info(f"Adjusting continuum 2 to {new_continuum_2_pitch_angle}")
                self.servo3.angle = new_continuum_2_pitch_angle
                

        except ValueError as e:
            self.get_logger().error(f"Joint name not found in joint_states: {e}")

    def move_stepper(self, steps, direction, direction_pin, pulse_pin, delay=0.001):
        """Stepper Motor Helper Function, handles all stepper movement

        Args:
            steps (int): the amount of "steps" the stepper motor will take
            direction (int/bool): the direction the stepper motor will move, 0 is for backward, 1 is for forward
            direction_pin (int): the GPIO pin used to set the direction of the stepper motor
            pulse_pin (int): the GPIO pin used to pulse the stepper motor
            delay (float, optional): the delay between steps, defaults to 0.001.
        """
        self.get_logger().info(f"Moving stepper: steps={steps}, direction={direction}, direction_pin={direction_pin}, pulse_pin={pulse_pin}")
        lgpio.gpio_write(self.chip, direction_pin, direction)

        for _ in range(steps):
            lgpio.gpio_write(self.chip, pulse_pin, 1)
            time.sleep(delay)
            lgpio.gpio_write(self.chip, pulse_pin, 0)
            time.sleep(delay)

def main(args=None):
    """The main function initializes the ROS 2 node and starts the RCETIRobotController.

    Args:
        args (N/A optional):Defaults to None, shouldn't be set to anything
    """
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
