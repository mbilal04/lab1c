#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


from .utils import se3_to_tf

class StaticTfCamPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_cam_publisher')

        # Create a static transform broadcaster
        self.static_br = StaticTransformBroadcaster(self)

        # Publish the static transforms once
        self.publish_static_transforms()

    def publish_static_transforms(self):
        """Precompute and publish static transforms (base_link -> left_cam -> right_cam)"""

        # Base link to left camera transform
        leftcam_to_robot_transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0.05],  # 0.05m to the left (positive y)
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]).astype(float)

        # Left camera to right camera transform
        rightcam_to_leftcam_transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, -0.1],  # 0.1m to the right of left_cam (negative y)
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]).astype(float)

        now = self.get_clock().now()

        # Convert to TF messages
        leftcam_to_robot_msg = se3_to_tf(leftcam_to_robot_transform, now, parent='base_link', child='left_cam')
        rightcam_to_leftcam_msg = se3_to_tf(rightcam_to_leftcam_transform, now, parent='left_cam', child='right_cam')

        # Publish the static transforms
        self.static_br.sendTransform([leftcam_to_robot_msg, rightcam_to_leftcam_msg])

        self.get_logger().info("Published static TFs: base_link -> left_cam, left_cam -> right_cam")


def main(args=None):
    rclpy.init(args=args)

    node = StaticTfCamPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
