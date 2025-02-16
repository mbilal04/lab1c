#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class OpenSpacePublisher(Node):
    def __init__(self):
        super().__init__('open_space_publisher')

        # Create publishers for distance and angle
        self.distance_publisher = self.create_publisher(Float32, 'open_space/distance', 10)
        self.angle_publisher = self.create_publisher(Float32, 'open_space/angle', 10)

        # Create subscription to the 'fake_scan' topic
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def scan_callback(self, scan_msg: LaserScan):
        """Callback to handle incoming laser scans and compute the max range and corresponding angle."""
        if not scan_msg.ranges:
            return

        # Find the maximum range and the index of that maximum
        max_range = max(scan_msg.ranges)
        max_index = scan_msg.ranges.index(max_range)

        # Calculate the angle corresponding to the max range
        # angle = angle_min + (index * angle_increment)
        angle = scan_msg.angle_min + (max_index * scan_msg.angle_increment)

        # Publish the longest range
        distance_msg = Float32()
        distance_msg.data = max_range
        self.distance_publisher.publish(distance_msg)

        # Publish the corresponding angle (in radians)
        angle_msg = Float32()
        angle_msg.data = angle
        self.angle_publisher.publish(angle_msg)

        # Optionally, log info (convert angle to degrees just for easy reading)
        self.get_logger().info(
            f"Longest range: {max_range:.2f} at angle: {math.degrees(angle):.2f} degrees"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OpenSpacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
