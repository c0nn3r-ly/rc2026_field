import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


WORLD_NAME = 'robocon2026_world_scene'


def generate_launch_description():
    pkg_rc2026_field = get_package_share_directory('rc2026_field')
    # 声明配置路径参数
    kfs_config_path = os.path.join(pkg_rc2026_field, 'config', 'kfs_config.yaml')

    world_path = os.path.join(pkg_rc2026_field, 'worlds', 'robocon2026.world')
    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_rc2026_field, 'models')
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': f'-r {world_path}',
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    gz_pose_bridge = Node(
        package='rc2026_field',
        executable='gz_pose_bridge',
        name='gz_pose_bridge',
        output='screen',
        parameters=[{'world_name': WORLD_NAME}]
    )

    kfs_manager = Node(
        package='rc2026_field',
        executable='kfs_manager',
        name='kfs_manager',
        output='screen',
        parameters=[{'use_sim_time': True, 'config_path': kfs_config_path}]
    )

    field_gui = Node(
        package='rc2026_field',
        executable='field_gui',
        name='field_gui',
        output='screen',
        parameters=[{'use_sim_time': True, 'config_path': kfs_config_path}]
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        clock_bridge,
        gz_pose_bridge,
        kfs_manager,
        field_gui
    ])
