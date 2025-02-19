import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from .utils import se3_to_tf, tf_to_se3

class BaseLinkTfPublisher(Node):
    def __init__(self):
        super().__init__('base_link_tf_pub')

        self.br = tf2_ros.TransformBroadcaster(self)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.base_link_to_left_cam_transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0.05],  # Left cam is 0.05m to the left
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]).astype(float)

        # Compute the inverse transform (left_cam -> base_link)
        self.left_cam_to_base_link_transform = np.linalg.inv(self.base_link_to_left_cam_transform)

        # Timer callback to fetch and publish transforms
        self.timer = self.create_timer(1.0 / 40, self.node_callback)

    def node_callback(self):
        try:
            # Get odom -> cam_2 transform
            odom_to_left_cam_msg = self.tfBuffer.lookup_transform('odom', 'left_cam', rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().info('Waiting on odom -> leftcam transform')
            return

        # Convert transform to numpy SE(3) matrix
        odom_to_left_cam_transform = tf_to_se3(odom_to_left_cam_msg.transform)

        # Compute odom -> base_link_2 transform
        odom_to_base_link_2_transform = odom_to_left_cam_transform @ self.left_cam_to_base_link_transform

        # Broadcast the new transform
        now = self.get_clock().now()
        odom_to_base_link_2_msg = se3_to_tf(odom_to_base_link_2_transform, now, parent='odom', child='base_link_2')

        self.br.sendTransform([odom_to_base_link_2_msg])
        self.get_logger().info('Published base_link_2 transform')

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
