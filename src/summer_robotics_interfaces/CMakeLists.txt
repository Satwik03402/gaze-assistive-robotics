cmake_minimum_required(VERSION 3.8)
project(summer_robotics_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObjectById.srv"
  "srv/GetAvailableObjects.srv"
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/RobotCommand.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs
)

ament_package()
