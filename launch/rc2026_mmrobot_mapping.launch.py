import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def maybe_start_slam(context, *args, **kwargs):
    slam_value = LaunchConfiguration('slam').perform(context).lower()
    if slam_value not in ('1', 'true', 'yes', 'on'):
        return []

    try:
        get_package_share_directory('slam_toolbox')
    except PackageNotFoundError:
        return [
            LogInfo(
                msg='slam_toolbox is not installed; RViz will still start. '
                    'Install ros-humble-slam-toolbox to enable mapping.'
            )
        ]

    return [
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/obstacle_scan',
                'mode': 'mapping',
            }],
            remappings=[
                ('/scan', '/obstacle_scan'),
            ],
        )
    ]


def generate_launch_description():
    pkg_rc2026_field = get_package_share_directory('rc2026_field')
    rviz_enabled = LaunchConfiguration('rviz')

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rc2026_field, 'launch', 'rc2026_mmrobot_rviz.launch.py')
        ),
        launch_arguments={
            'rviz': rviz_enabled,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz together with mapping.',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Start slam_toolbox if it is installed.',
        ),
        OpaqueFunction(function=maybe_start_slam),
        rviz_launch,
    ])
