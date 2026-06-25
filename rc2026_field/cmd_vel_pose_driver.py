#!/usr/bin/env python3
import math
from typing import List

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


DEFAULT_JOINT_NAMES = [
    'front_left_str',
    'front_left_drive',
    'front_right_str',
    'front_right_drive',
    'rear_right_str',
    'rear_right_drive',
    'rear_left_str',
    'rear_left_drive',
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
]


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_to_quaternion(yaw: float):
    return rpy_to_quaternion(0.0, 0.0, yaw)


class CmdVelPoseDriver(Node):
    def __init__(self):
        super().__init__('cmd_vel_pose_driver')

        self.entity_name = self.declare_parameter('entity_name', 'mmrobot').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.pose_service_name = self.declare_parameter(
            'pose_service_name', '/simulation/set_entity_pose'
        ).value

        self.update_rate = float(self.declare_parameter('update_rate', 30.0).value)
        self.pose_rate = float(self.declare_parameter('pose_rate', 20.0).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)
        self.initial_x = float(self.declare_parameter('initial_x', 0.0).value)
        self.initial_y = float(self.declare_parameter('initial_y', 0.0).value)
        self.initial_z = float(self.declare_parameter('initial_z', 0.05).value)
        self.initial_yaw = float(self.declare_parameter('initial_yaw', 0.0).value)
        self.max_linear = float(self.declare_parameter('max_linear', 1.2).value)
        self.max_lateral = float(self.declare_parameter('max_lateral', 1.2).value)
        self.max_angular = float(self.declare_parameter('max_angular', 2.0).value)
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.055).value)
        self.lidar_entity_name = self.declare_parameter(
            'lidar_entity_name', ''
        ).value
        self.lidar_offset_x = float(self.declare_parameter('lidar_offset_x', 0.275).value)
        self.lidar_offset_y = float(self.declare_parameter('lidar_offset_y', 0.0).value)
        self.lidar_offset_z = float(self.declare_parameter('lidar_offset_z', 0.12).value)
        self.lidar_roll = float(self.declare_parameter('lidar_roll', 0.0).value)
        self.lidar_pitch = float(
            self.declare_parameter('lidar_pitch', 0.7853981633974483).value
        )
        self.lidar_yaw_offset = float(
            self.declare_parameter('lidar_yaw_offset', 0.0).value
        )
        self.joint_names = list(
            self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES).value
        )

        self.x = self.initial_x
        self.y = self.initial_y
        self.z = self.initial_z
        self.yaw = self.initial_yaw
        self.last_cmd = Twist()
        self.wheel_phase = 0.0

        self.clock = self.get_clock()
        self.last_time = self.clock.now()
        self.last_cmd_time = self.last_time
        self.last_pose_request_time = self.last_time
        self.pending_pose_futures = []
        self.last_service_log_time = self.last_time

        self.pose_client = self.create_client(SetEntityPose, self.pose_service_name)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_callback, 10
        )

        timer_period = 1.0 / max(self.update_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f'Driving Gazebo model [{self.entity_name}] from {self.cmd_vel_topic}'
        )

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.clock.now()

    def timer_callback(self):
        now = self.clock.now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt < 0.0 or dt > 0.25:
            dt = 0.0

        vx = clamp(self.last_cmd.linear.x, self.max_linear)
        vy = clamp(self.last_cmd.linear.y, self.max_lateral)
        wz = clamp(self.last_cmd.angular.z, self.max_angular)

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            vx = 0.0
            vy = 0.0
            wz = 0.0

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + wz * dt),
            math.cos(self.yaw + wz * dt),
        )

        if self.wheel_radius > 0.0:
            self.wheel_phase += vx * dt / self.wheel_radius

        self.publish_odometry(now, vx, vy, wz)
        self.publish_joint_states(now)
        self.request_gazebo_pose(now)

    def publish_odometry(self, now, vx: float, vy: float, wz: float):
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def publish_joint_states(self, now):
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = self.joint_names
        positions: List[float] = []
        for name in self.joint_names:
            if name.endswith('_drive'):
                positions.append(self.wheel_phase)
            else:
                positions.append(0.0)
        msg.position = positions
        self.joint_pub.publish(msg)

    def request_gazebo_pose(self, now):
        pose_period = 1.0 / max(self.pose_rate, 1.0)
        if (now - self.last_pose_request_time).nanoseconds * 1e-9 < pose_period:
            return
        self.last_pose_request_time = now

        if not self.pose_client.service_is_ready():
            if (now - self.last_service_log_time).nanoseconds * 1e-9 > 2.0:
                self.get_logger().warn(
                    f'Waiting for {self.pose_service_name} before moving {self.entity_name}'
                )
                self.last_service_log_time = now
            return

        self.pending_pose_futures = [
            future for future in self.pending_pose_futures if not future.done()
        ]
        if len(self.pending_pose_futures) > 2:
            return

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        self.send_pose_request(self.entity_name, self.x, self.y, self.z, qx, qy, qz, qw)

        if self.lidar_entity_name:
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            lidar_x = self.x + self.lidar_offset_x * cos_yaw - self.lidar_offset_y * sin_yaw
            lidar_y = self.y + self.lidar_offset_x * sin_yaw + self.lidar_offset_y * cos_yaw
            lidar_z = self.z + self.lidar_offset_z
            lqx, lqy, lqz, lqw = rpy_to_quaternion(
                self.lidar_roll,
                self.lidar_pitch,
                self.yaw + self.lidar_yaw_offset,
            )
            self.send_pose_request(
                self.lidar_entity_name,
                lidar_x,
                lidar_y,
                lidar_z,
                lqx,
                lqy,
                lqz,
                lqw,
            )

    def send_pose_request(
        self,
        entity_name: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ):
        request = SetEntityPose.Request()
        request.entity.name = entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.x = qx
        request.pose.orientation.y = qy
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw
        self.pending_pose_futures.append(self.pose_client.call_async(request))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPoseDriver()
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
