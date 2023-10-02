cmake_minimum_required(VERSION 3.8)
project(camera_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Integrated cactus-rt dependencies for the workshop
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/cactus-rt-integrated
  ${CMAKE_CURRENT_BINARY_DIR}/cactus-rt-build
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node
  rclcpp
  sensor_msgs
)

add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node
  rclcpp
  sensor_msgs
)
target_link_libraries(sensor_node
  cactus_rt
)

add_executable(actuation_node src/actuation_node.cpp)
ament_target_dependencies(actuation_node
  rclcpp
  sensor_msgs
)
target_link_libraries(actuation_node
  cactus_rt
)

install(TARGETS
  publisher_node
  sensor_node
  actuation_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
