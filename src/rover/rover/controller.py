#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class XboxControllerNode(Node):

    def __init__(self):
        super().__init__('xbox_controller_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        self.get_logger().info('Received axes: %s, buttons: %s' % (msg.axes, msg.buttons))

        # Perform tasks based on button press
        if msg.buttons[0] == 1:  # A button
            self.task_a()
        elif msg.buttons[1] == 1:  # B button
            self.task_b()
        # Add more tasks here for different buttons
        
        # Perform tasks based on analog stick values
        left_stick_x = msg.axes[0]
        left_stick_y = msg.axes[1]
        right_stick_x = msg.axes[3]
        right_stick_y = msg.axes[4]

    def task_a(self):
        self.get_logger().info('Task A performed.')

    def task_b(self):
        self.get_logger().info('Task B performed.')

def main(args=None):
    rclpy.init(args=args)
    xbox_controller_node = XboxControllerNode()
    rclpy.spin(xbox_controller_node)

    xbox_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
