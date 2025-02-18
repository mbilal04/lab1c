import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import random
import math

class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')

        # Declare ROS parameters with default values
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('angle_min', -2/3 * math.pi)
        self.declare_parameter('angle_max', 2/3 * math.pi)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_increment', (1/300) * math.pi)
        self.declare_parameter('fake_scan_topic', 'fake_scan')

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self.fake_scan_topic = self.get_parameter('fake_scan_topic').get_parameter_value().string_value

        # Compute the number of range elements
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # Publishers
        self.scan_publisher = self.create_publisher(LaserScan, self.fake_scan_topic, 10)
        self.range_length_publisher = self.create_publisher(Float32, 'range_test', 10)

        # Publish rate
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.publish_scan)

    def publish_scan(self):
        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
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
