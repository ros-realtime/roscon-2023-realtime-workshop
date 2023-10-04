cmake_minimum_required(VERSION 3.18)
project(camera_demo_4_1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# CactusRT
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/cactus-rt-integrated
  ${CMAKE_CURRENT_BINARY_DIR}/cactus-rt-build
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_demo_interfaces REQUIRED)

add_executable(camera_demo
  src/main.cc
  src/system_nodes.cc
  src/application_nodes.cc
  src/tracing.cc
)

target_include_directories(camera_demo
  PRIVATE
  include
)

target_link_libraries(camera_demo
  cactus_rt
)

target_compile_features(camera_demo
  PRIVATE
  cxx_std_17
)

ament_target_dependencies(camera_demo rclcpp camera_demo_interfaces std_msgs)

#############
## Install ##
#############

install(
  TARGETS camera_demo
)

ament_package()

