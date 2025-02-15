import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')  # Node name
        self.publisher_ = self.create_publisher(Float32, 'my_random_float', 10)  # Topic name
        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.publish_random_number)

    def publish_random_number(self):
        msg = Float32()
        msg.data = random.uniform(0.0, 10.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
