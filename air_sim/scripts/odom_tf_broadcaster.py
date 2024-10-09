#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class GroundTruthToTF(Node):
    def __init__(self):
        super().__init__('ground_truth_to_tf_publisher')

        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to Gazebo's ground truth topic (e.g., base_pose_ground_truth)
        self.ground_truth_subscriber = self.create_subscription(
            Odometry,  # Gazebo typically publishes ground truth as Odometry
            '/base_pose_ground_truth',  # Make sure this matches your Gazebo topic
            self.handle_ground_truth,
            10
        )

        # To track if the first odometry message is processed
        self.initial_odom_received = False
        self.initial_pose = None

    def handle_ground_truth(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Use the timestamp from the odometry message for better sync
        t.header.stamp = msg.header.stamp

        # Parent frame is odom
        t.header.frame_id = 'odom'

        # Child frame is base_link
        t.child_frame_id = 'base_link'

        # On first odometry message, store the initial pose as the reference
        if not self.initial_odom_received:
            self.initial_pose = msg.pose.pose
            self.initial_odom_received = True

        # Compute the transform relative to the initial pose
        t.transform.translation.x = msg.pose.pose.position.x - self.initial_pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y - self.initial_pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z - self.initial_pose.position.z

        # Copy the orientation from ground truth to the transform
        # Assuming the orientation needs no offset, as it is relative
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
