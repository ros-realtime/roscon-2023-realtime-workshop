cmake_minimum_required(VERSION 3.18)
project(camera_demo_interfaces)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FakeImage.msg"
)

ament_package()
