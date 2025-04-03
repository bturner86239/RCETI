#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import termios
import tty
import select

class KeyboardContinuumController(Node):
    def __init__(self):
        super().__init__('keyboard_continuum_controller')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/continuum/control_input', 10)

        self.kappa = [0.0, 0.0]
        self.phi = [0.0, 0.0]

        self.timer = self.create_timer(0.1, self.update)  # sends update every 100ms

        self.get_logger().info('Use keys: W/S = bend seg1, A/D = rotate seg1 | Q/E = bend seg2')
        self.settings = termios.tcgetattr(sys.stdin)

    def update(self):
        key = self.get_key()
        if key == 'w':
            self.kappa[0] += 0.05
        elif key == 's':
            self.kappa[0] -= 0.05
        elif key == 'a':
            self.phi[0] += 0.1
        elif key == 'd':
            self.phi[0] -= 0.1
        elif key == 'q':
            self.kappa[1] += 0.05
        elif key == 'e':
            self.kappa[1] -= 0.05
        elif key == '\x03':  # Ctrl+C
            rclpy.shutdown()
            return

        msg = Float32MultiArray()
        msg.data = [self.kappa[0], self.phi[0], self.kappa[1], self.phi[1]]
        self.publisher_.publish(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardContinuumController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
