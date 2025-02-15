import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'my_random_float',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)

    def listener_callback(self, msg):
        if msg.data > 0:  # Ensure the value is positive for natural log
            log_value = math.log(msg.data)
        else:
            log_value = float('-inf')  # Handle log(0) case

        log_msg = Float32()
        log_msg.data = log_value
        self.publisher_.publish(log_msg)

        self.get_logger().info(f'Received: {msg.data}, Published log: {log_value}')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
