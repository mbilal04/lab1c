import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import OpenSpace  # Import the custom message
import math

class OpenSpacePublisher(Node):
    def __init__(self):
        super().__init__('open_space_publisher')

        # Create a publisher for the OpenSpace custom message
        self.publisher = self.create_publisher(OpenSpace, 'open_space', 10)

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

        # Create an instance of the custom message and populate it
        open_space_msg = OpenSpace()
        open_space_msg.angle = angle  # Ensure angle is before distance as required
        open_space_msg.distance = max_range

        # Publish the OpenSpace message
        self.publisher.publish(open_space_msg)

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
