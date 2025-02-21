import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time  # for delays/testing
import gpiozero  # motor control

class RCETIRobotController(Node):

    def __init__(self):
        super().__init__('rceti_controller')
        
        self.x_position_sub = self.create_subscription(
            Float32,
            'rceti/x_position',
            self.x_position_callback,
            10)
        self.z_position_sub = self.create_subscription(
            Float32,
            'rceti/z_position',
            self.z_position_callback,
            10)
        self.pitch_angle_sub = self.create_subscription(
            Float32,
            'rceti/pitch_angle',
            self.pitch_angle_callback,
            10)

        #prevent unused variable warning
        self.x_position_sub  
        self.z_position_sub
        self.pitch_angle_sub

        #position Tracking
        self.x_position = 0.0
        self.z_position = 0.0
        self.pitch_angle = 0.0

        #GPIO motors
        self.x_Axis = gpiozero.Motor(20, 21)  
        self.z_Axis = gpiozero.Motor(18, 19)  

    def x_position_callback(self, msg):
        if self.x_position != msg.data:
            self.get_logger().info(f"Moving X to {msg.data}")
            self.x_position = msg.data

            if msg.data < 0:  #left when A
                self.x_Axis.backward()
            elif msg.data > 0:  #right when D
                self.x_Axis.forward()
            else:
                self.x_Axis.stop()

    def z_position_callback(self, msg):
        if self.z_position != msg.data:
            self.get_logger().info(f"Moving Z to {msg.data}")
            self.z_position = msg.data

            if msg.data > 0:  #up when W
                self.z_Axis.forward()
            elif msg.data < 0:  #down when S
                self.z_Axis.backward()
            else:
                self.z_Axis.stop()  

    def pitch_angle_callback(self, msg):
        """Adjust robot pitch (for tilting platforms, arms, etc.)"""
        if self.pitch_angle != msg.data:
            self.get_logger().info(f"Adjusting Pitch to {msg.data}")
            self.pitch_angle = msg.data
            # TODO: Implement pitch 

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RCETIRobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
