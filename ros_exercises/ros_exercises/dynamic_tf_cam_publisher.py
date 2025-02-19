import numpy as np
import rclpy
import tf2_ros
import time
from rclpy.node import Node

from .utils import se3_to_tf
from .utils import tf_to_se3


class DynamicTfCamPublisher(Node):

    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')


        timer_period = 1.0 / 40  # hz
        self.timer = self.create_timer(timer_period, self.node_callback)


        self.br = tf2_ros.TransformBroadcaster(self)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def node_callback(self):
        try:
            planet_to_sun_msg: TransformStamped = self.tfBuffer.lookup_transform('odom', 'base_link',
                                                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().info('waiting on parent')
            return


        robot_to_odom_transform = tf_to_se3(planet_to_sun_msg.transform)

        leftcam_to_robot_transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0.05],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]).astype(float)

        leftcam_to_odom_transform = robot_to_odom_transform @ leftcam_to_robot_transform

        #left to right
        rightcam_to_left_cam_transform = leftcam_to_robot_transform = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, -0.1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]).astype(float)
        now = self.get_clock().now()
        leftcam_to_odom_msg = se3_to_tf(leftcam_to_odom_transform, now, parent='odom', child='left_cam')
        rightcam_to_leftcam_msg = se3_to_tf(rightcam_to_left_cam_transform, now, parent='left_cam', child='right_cam')
        self.br.sendTransform([leftcam_to_odom_msg, rightcam_to_leftcam_msg])
        self.get_logger().info('Published')


def main(args=None):
    rclpy.init(args=args)

    node = DynamicTfCamPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
