#!/bin/bash
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

echo "Stopping ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true

echo "Killing Gazebo Harmonic / gz-sim processes..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f gz-sim 2>/dev/null

echo "Killing any lingering ROS launch processes..."
pkill -9 -f rc2026_field_sim.launch.py 2>/dev/null
pkill -9 -f rc2026_field_sim_with_controller.launch.py 2>/dev/null
pkill -9 -f rc2026_mmrobot_sim.launch.py 2>/dev/null
pkill -9 -f rc2026_mmrobot_mapping.launch.py 2>/dev/null
pkill -9 -f cmd_vel_pose_driver 2>/dev/null
pkill -9 -f gz_pose_bridge 2>/dev/null
pkill -9 -f lidar_frame_relay 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null

echo "Gazebo cleanup complete."
