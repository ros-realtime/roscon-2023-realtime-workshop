
cmake_minimum_required(VERSION 3.18)
project(inverted_pendulum_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# CactusRT
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/cactus-rt-integrated
  ${CMAKE_CURRENT_BINARY_DIR}/cactus-rt-build
)

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

# Example 2-1

add_executable(${PROJECT_NAME}2-1
  src/exercise2-1/main.cc
  src/exercise2-1/ros_pendulum_node.cc
  src/exercise2-1/rt_thread.cc
)

target_include_directories(${PROJECT_NAME}2-1
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}2-1
  cactus_rt
)

target_compile_options(${PROJECT_NAME}2-1
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

target_compile_features(${PROJECT_NAME}2-1
  PRIVATE
  cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}2-1 ${THIS_PACKAGE_INCLUDE_DEPENDS})


# Example 2-2

add_executable(${PROJECT_NAME}2-2
  src/exercise2-2/main.cc
  src/exercise2-2/ros_pendulum_node.cc
  src/exercise2-2/rt_thread.cc
)


target_include_directories(${PROJECT_NAME}2-2
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}2-2
  cactus_rt
)

target_compile_options(${PROJECT_NAME}2-2
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

target_compile_features(${PROJECT_NAME}2-2
  PRIVATE
  cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}2-2 ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Example 2-3

add_executable(${PROJECT_NAME}2-3
  src/exercise2-3/main.cc
  src/exercise2-3/ros_pendulum_node.cc
  src/exercise2-3/rt_thread.cc
)

target_include_directories(${PROJECT_NAME}2-3
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}2-3
  cactus_rt
)

target_compile_options(${PROJECT_NAME}2-3
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

target_compile_features(${PROJECT_NAME}2-3
  PRIVATE
  cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}2-3 ${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}2-1 ${PROJECT_NAME}2-2 ${PROJECT_NAME}2-3
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
