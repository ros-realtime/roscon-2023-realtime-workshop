
cmake_minimum_required(VERSION 3.18)
project(inverted_pendulum_example)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# CactusRT
include(FetchContent)
FetchContent_Declare(
  cactus_rt
  GIT_REPOSITORY https://github.com/cactusdynamics/cactus-rt.git
  GIT_TAG        ...
)
FetchContent_MakeAvailable(cactus_rt)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  inverted_pendulum_interfaces
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/main.cc
  src/ros_pendulum_node.cc
  src/rt_thread.cc
)

target_include_directories(${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
# TODO: >  
# PRIVATE
  cactus_rt
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
  -Wall
  -Wextra
  -Wshadow
  -Wnon-virtual-dtor
  -pedantic

  -Wold-style-cast
  -Wcast-align
  -Wunused
  -Woverloaded-virtual
  -Wpedantic
  -Wconversion
  -Wsign-conversion
  -Wmisleading-indentation
  -Wduplicated-cond
  -Wduplicated-branches
  -Wlogical-op
  -Wnull-dereference
  -Wuseless-cast
  -Wdouble-promotion
  -Wformat=2
  -Wimplicit-fallthrough
)

target_compile_features(${PROJECT_NAME}
  PRIVATE
  cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
