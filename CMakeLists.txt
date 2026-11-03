cmake_minimum_required(VERSION 3.22)
project(cable_insertion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(deterministic_insertion src/deterministic_insertion.cpp)
target_compile_features(deterministic_insertion PRIVATE cxx_std_17)
ament_target_dependencies(deterministic_insertion
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  tf2_msgs
  ros_gz_interfaces
)

install(TARGETS deterministic_insertion
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  sim/dh_ag95/attach_gripper.py
  sim/dh_ag95/gripper_move.py
  scripts/pose_bridge.py
  scripts/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  moveit_config
  sim
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
