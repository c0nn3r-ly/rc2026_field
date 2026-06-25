#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.default_parent_frame = self.declare_parameter(
            'default_parent_frame', 'odom'
        ).value
        self.default_child_frame = self.declare_parameter(
            'default_child_frame', 'base_footprint'
        ).value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.get_logger().info(
            f'Broadcasting TF from {self.odom_topic} '
            f'({self.default_parent_frame} -> {self.default_child_frame})'
        )

    def odom_callback(self, msg: Odometry):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = msg.header.frame_id or self.default_parent_frame
        tf_msg.child_frame_id = msg.child_frame_id or self.default_child_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if 'context is not valid' not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
