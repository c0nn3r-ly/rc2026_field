#!/usr/bin/env python3
import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class LidarFrameRelay(Node):
    def __init__(self):
        super().__init__('lidar_frame_relay')

        self.frame_id = self.declare_parameter('frame_id', 'front_mid360').value
        self.raw_scan_topic = self.declare_parameter(
            'raw_scan_topic', '/gz/obstacle_scan'
        ).value
        self.scan_topic = self.declare_parameter('scan_topic', '/obstacle_scan').value
        self.raw_cloud_topic = self.declare_parameter(
            'raw_cloud_topic', '/gz/livox_lidar/points'
        ).value
        self.cloud_topic = self.declare_parameter('cloud_topic', '/livox/lidar').value
        self.scan_cloud_fallback = bool(
            self.declare_parameter('scan_cloud_fallback', True).value
        )
        self.cloud_fallback_timeout = float(
            self.declare_parameter('cloud_fallback_timeout', 1.0).value
        )

        self.last_raw_cloud_time = None

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.raw_scan_topic, self.scan_callback, 10
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.raw_cloud_topic, self.cloud_callback, 10
        )

        self.get_logger().info(
            f'Relaying lidar frames to [{self.frame_id}] '
            f'({self.scan_topic}, {self.cloud_topic})'
        )

    def cloud_recent(self, now) -> bool:
        if self.last_raw_cloud_time is None:
            return False
        age = (now - self.last_raw_cloud_time).nanoseconds * 1e-9
        return age <= self.cloud_fallback_timeout

    def scan_callback(self, msg: LaserScan):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.frame_id
        self.scan_pub.publish(out)

        now = self.get_clock().now()
        if self.scan_cloud_fallback and not self.cloud_recent(now):
            cloud = self.cloud_from_scan(out)
            self.cloud_pub.publish(cloud)

    def cloud_callback(self, msg: PointCloud2):
        self.last_raw_cloud_time = self.get_clock().now()
        out = copy.deepcopy(msg)
        out.header.frame_id = self.frame_id
        self.cloud_pub.publish(out)

    def cloud_from_scan(self, scan: LaserScan) -> PointCloud2:
        points = []
        angle = scan.angle_min
        for range_value in scan.ranges:
            if math.isfinite(range_value):
                in_min = scan.range_min <= 0.0 or range_value >= scan.range_min
                in_max = scan.range_max <= 0.0 or range_value <= scan.range_max
                if in_min and in_max:
                    points.append((
                        range_value * math.cos(angle),
                        range_value * math.sin(angle),
                        0.0,
                    ))
            angle += scan.angle_increment

        return point_cloud2.create_cloud_xyz32(scan.header, points)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrameRelay()
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
