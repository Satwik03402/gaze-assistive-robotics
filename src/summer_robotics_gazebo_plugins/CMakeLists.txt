cmake_minimum_required(VERSION 3.8)
project(summer_robotics_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_definitions(
  ${GAZEBO_CXX_FLAGS}
)

add_library(
  object_pose_plugin SHARED
  src/object_pose_plugin.cpp
)

ament_target_dependencies(
  object_pose_plugin
  rclcpp
  gazebo_ros
  gazebo_msgs
  geometry_msgs
)

target_link_libraries(
  object_pose_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS object_pose_plugin
  LIBRARY DESTINATION lib
)

ament_package()
