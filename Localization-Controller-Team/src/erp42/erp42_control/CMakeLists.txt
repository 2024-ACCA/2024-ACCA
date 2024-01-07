cmake_minimum_required(VERSION 3.5)
project(erp42_control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ControlMessage.msg
  msg/StanleyError.msg
  DEPENDENCIES std_msgs
)

ament_package()
