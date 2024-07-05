#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sd_msgs.msg import SDControl

class ConverterNode(Node):
    def __init__(self):
        super().__init__('converter_node')
        
        self.subscription = self.create_subscription(
            SDControl,
            'sd_control',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/sd_control/cmd_vel',
            10)

    def listener_callback(self, msg):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.speed = msg.torque  # Map your custom torque to speed
        ackermann_msg.drive.steering_angle = msg.steer * 3.141592653589793 / 180.0  # Assuming steer is in degrees and converting to radians

        self.publisher.publish(ackermann_msg)
        self.get_logger().info('Publishing: "%s"' % ackermann_msg)


def main(args=None):
    rclpy.init(args=args)

    converter_node = ConverterNode()

    rclpy.spin(converter_node)

    converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
