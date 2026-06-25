import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rc2026_field = get_package_share_directory('rc2026_field')
    default_rviz_config = os.path.join(pkg_rc2026_field, 'rviz', 'mmrobot_sim.rviz')

    rviz_enabled = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz for MMRobot visualization.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz config file.',
        ),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        rviz,
    ])
