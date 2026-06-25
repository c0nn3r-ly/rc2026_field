cmake_minimum_required(VERSION 3.8)
project(rc2026_field)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_executable(gz_pose_bridge src/gz_pose_bridge.cpp)
target_compile_features(gz_pose_bridge PUBLIC cxx_std_17)
ament_target_dependencies(gz_pose_bridge
  rclcpp
  ros_gz_interfaces
)
target_link_libraries(gz_pose_bridge
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

install(TARGETS
  gz_pose_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  rc2026_field/kfs_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kfs_manager
)

install(PROGRAMS
  rc2026_field/field_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME field_gui
)

install(PROGRAMS
  rc2026_field/cmd_vel_pose_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_pose_driver
)

install(PROGRAMS
  rc2026_field/lidar_frame_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_frame_relay
)

install(PROGRAMS
  rc2026_field/odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_tf_broadcaster
)

install(DIRECTORY
  config
  launch
  models
  rviz
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
