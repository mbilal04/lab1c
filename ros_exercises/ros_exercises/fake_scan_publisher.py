import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import random
import time
import math

class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')

        # Publishers
        self.scan_publisher = self.create_publisher(LaserScan, 'fake_scan', 10)
        self.range_length_publisher = self.create_publisher(Float32, 'range_test', 10)

        # Publish rate
        self.timer_period = 1.0 / 20  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_scan)

        # LaserScan properties
        self.angle_min = -2/3 * math.pi
        self.angle_max = 2/3 * math.pi
        self.angle_increment = (1/300) * math.pi
        self.range_min = 1.0
        self.range_max = 10.0

        # Compute number of range elements
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

    def publish_scan(self):
        # Create LaserScan message
        scan_msg = LaserScan()
        # scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.stamp = rclpy.time.Time().to_msg()
        scan_msg.header.frame_id = "base_link"

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0  # Leaving unset
        scan_msg.scan_time = self.timer_period
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Generate random range values
        scan_msg.ranges = [random.uniform(self.range_min, self.range_max) for _ in range(self.num_ranges)]

        # Publish the LaserScan message
        self.scan_publisher.publish(scan_msg)

        # Publish the length of the ranges array as a Float32
        length_msg = Float32()
        length_msg.data = float(self.num_ranges)
        self.range_length_publisher.publish(length_msg)

        # Log information
        self.get_logger().info(f"Published LaserScan with {self.num_ranges} points")

def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
