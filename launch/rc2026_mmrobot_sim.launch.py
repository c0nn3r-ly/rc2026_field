import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORLD_NAME = 'robocon2026_world_scene'


def generate_launch_description():
    pkg_rc2026_field = get_package_share_directory('rc2026_field')
    pkg_mmrobot_description = get_package_share_directory('mmrobot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    robot_name = LaunchConfiguration('robot_name')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    gz_headless = LaunchConfiguration('gz_headless')
    gz_partition = LaunchConfiguration('gz_partition')
    pose_driver = LaunchConfiguration('pose_driver')
    robot_static = LaunchConfiguration('robot_static')

    world_path = os.path.join(pkg_rc2026_field, 'worlds', 'robocon2026_mmrobot.world')
    robot_xacro = os.path.join(pkg_mmrobot_description, 'urdf', 'MMRobot.xacro')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', robot_xacro, ' robot_static:=', robot_static]),
            value_type=str,
        ),
        'use_sim_time': True,
    }

    set_field_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_rc2026_field, 'models'),
    )
    set_robot_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_mmrobot_description),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
        }.items(),
        condition=UnlessCondition(gz_headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s {world_path}',
        }.items(),
        condition=IfCondition(gz_headless),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/gz/obstacle_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/gz/livox_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/mmrobot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[
            ('/model/mmrobot/cmd_vel', '/cmd_vel'),
        ],
        output='screen',
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/mmrobot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/mmrobot/odometry', '/odom'),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', WORLD_NAME,
            '-name', robot_name,
            '-allow_renaming', 'false',
            '-x', robot_x,
            '-y', robot_y,
            '-z', robot_z,
            '-Y', robot_yaw,
            '-topic', '/robot_description',
        ],
    )

    gz_pose_bridge = Node(
        package='rc2026_field',
        executable='gz_pose_bridge',
        name='gz_pose_bridge',
        output='screen',
        parameters=[{
            'world_name': WORLD_NAME,
        }],
    )

    cmd_vel_pose_driver = Node(
        package='rc2026_field',
        executable='cmd_vel_pose_driver',
        name='cmd_vel_pose_driver',
        output='screen',
        condition=IfCondition(pose_driver),
        parameters=[{
            'use_sim_time': True,
            'entity_name': ParameterValue(robot_name, value_type=str),
            'initial_x': ParameterValue(robot_x, value_type=float),
            'initial_y': ParameterValue(robot_y, value_type=float),
            'initial_z': ParameterValue(robot_z, value_type=float),
            'initial_yaw': ParameterValue(robot_yaw, value_type=float),
        }],
    )

    lidar_frame_relay = Node(
        package='rc2026_field',
        executable='lidar_frame_relay',
        name='lidar_frame_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'front_mid360',
        }],
    )

    odom_tf_broadcaster = Node(
        package='rc2026_field',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom',
            'default_parent_frame': 'odom',
            'default_child_frame': 'base_footprint',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='mmrobot',
            description='Gazebo entity name for the spawned MMRobot.',
        ),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.03'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'gz_headless',
            default_value='true',
            description='Run Gazebo server only. Set false to open Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value='rc2026_mmrobot',
            description='Gazebo Transport partition used by this simulation instance.',
        ),
        DeclareLaunchArgument(
            'pose_driver',
            default_value='false',
            description='Start the legacy /cmd_vel set-pose driver.',
        ),
        DeclareLaunchArgument(
            'robot_static',
            default_value='false',
            description='Spawn MMRobot as static. Default false enables map collision and gravity.',
        ),
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        set_field_resource_path,
        set_robot_resource_path,
        gz_sim,
        gz_sim_headless,
        clock_bridge,
        lidar_bridge,
        cmd_vel_bridge,
        odom_bridge,
        robot_state_publisher,
        spawn_robot,
        gz_pose_bridge,
        cmd_vel_pose_driver,
        lidar_frame_relay,
        odom_tf_broadcaster,
    ])
