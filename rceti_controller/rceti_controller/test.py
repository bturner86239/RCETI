
# # pca9685 motor board stuff - board that has the servo motors on it
# import board
# import busio
# from adafruit_pca9685 import PCA9685

# i2c = busio.I2C(board.SCL, board.SDA)
# pca = PCA9685(i2c)
# print("Found PCA9685!")


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time  # for delays/testing
import lgpio



from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

import adafruit_motor.servo


#Constants:
# maximum amount of request stepper motors will take at any given time
TOPIC_SUBSCRIPTION_BUFFER = 5


class RCETIRobotController(Node):

    def __init__(self):
        super().__init__('rceti_controller')

        servo1 = kit.servo[0]
        servo2 = kit.servo[1]
        servo3 = kit.servo[2]

        # SERVO TEST
        x = 0
        angle = 0
        while x < 18 :
            angle = x * 10
            x += 2
            print(f"servos to {angle}")
            servo1.angle = angle
            servo2.angle = angle
            servo3.angle = angle
            time.sleep(1)

        while x > 0 :
            angle = x * 10
            x -= 2
            print(f"servos to {angle}")
            servo1.angle = angle
            servo2.angle = angle
            servo3.angle = angle
            time.sleep(1)
        
        # servo2.angle = 0
        # servo3.angle = 0

        # print("waiting 2 seconds")
        # time.sleep(1)  # Give time to move

        # print("servos to -90 angle")
        # servo1.angle = -90
        # servo2.angle = 0
        # servo3.angle = 0


        
        
        self.x_position_sub = self.create_subscription(
            Float32,
            'rceti/x_position',
            self.x_position_callback,
            TOPIC_SUBSCRIPTION_BUFFER)
        self.z_position_sub = self.create_subscription(
            Float32,
            'rceti/z_position',
            self.z_position_callback,
            TOPIC_SUBSCRIPTION_BUFFER)
        self.pitch_angle_sub = self.create_subscription(
            Float32,
            'rceti/pitch_angle',
            self.pitch_angle_callback,
            TOPIC_SUBSCRIPTION_BUFFER)

        #prevent unused variable warning
        self.x_position_sub  
        self.z_position_sub
        self.pitch_angle_sub

        #position Tracking
        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.0

        self.X_DIRECTION_PIN = 21
        self.X_PULSE_PIN = 20
        self.Z_DIRECTION_PIN = 19
        self.Z_PULSE_PIN = 18

        # open the gpio chip and set the LED pin as output
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.X_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.X_PULSE_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_DIRECTION_PIN)
        lgpio.gpio_claim_output(self.chip, self.Z_PULSE_PIN)

    # Define stepper steps per unit in x and Z directions
    steps_per_mm_x = 40
    steps_per_mm_z = 40

    def x_position_callback(self, msg):
        if self.x_position != msg.data:
            self.get_logger().info(f"Moving X to {msg.data}")

            # Calculate the number of steps
            steps = int( abs(msg.data - self.x_position) * 1000 * self.steps_per_mm_x )
            # Determine direction
            direction = 1 if msg.data > self.x_position else 0

            self.move_stepper(steps, direction, self.X_DIRECTION_PIN, self.X_PULSE_PIN)
            self.x_position = msg.data

            '''
            self.get_logger().info(f"Moving X to {msg.data}")
            self.move_stepper(500, 1, self.X_DIRECTION_PIN, self.X_PULSE_PIN)
            time.sleep(1)
            self.move_stepper(500, 0, self.X_DIRECTION_PIN, self.X_PULSE_PIN)
            self.x_position = msg.data
            '''


    def z_position_callback(self, msg):
        if self.z_position != msg.data:
            self.get_logger().info(f"Moving Z to {msg.data}")

            # Calculate the number of steps
            steps = int(abs(msg.data - self.z_position) * 1000 * self.steps_per_mm_z)
            # Determine direction
            direction = 1 if msg.data > self.z_position else 0

            self.move_stepper(steps, direction, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
            self.z_position = msg.data

            '''
            self.get_logger().info(f"Moving Z to {msg.data}")
            self.move_stepper(500, 1, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
            time.sleep(1)
            self.move_stepper(500, 0, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
            self.z_position = msg.data
            '''

    def pitch_angle_callback(self, msg):
        
        '''
        TODO: finish implementation
        if self.z_position != msg.data:
            self.get_logger().info(f"Moving Z to {msg.data}")

            # Calculate the number of steps
            steps = int(abs(msg.data - self.z_position) * 100 * self.steps_per_mm_z)
            # Determine direction
            direction = 1 if msg.data > self.z_position else 0

            self.move_stepper(steps, direction, self.Z_DIRECTION_PIN, self.Z_PULSE_PIN)
            self.z_position = msg.data'
        '''

    def move_stepper(self, steps, direction, direction_pin, pulse_pin, delay=0.001):
        self.get_logger().info(f"move_stepper function recieved steps:{steps}, direction:{direction}, direction_pin:{direction_pin}, pulse_pin:s{pulse_pin}")
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
