cmake_minimum_required(VERSION 3.8)
project(latency_tester)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

# Integrated cactus-rt dependencies for the workshop
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/cactus-rt-integrated
  ${CMAKE_CURRENT_BINARY_DIR}/cactus-rt-build
)

add_executable(latency_tester
  main.cc
)

target_link_libraries(latency_tester
  PRIVATE
  cactus_rt
)

install(
  TARGETS latency_tester
)

ament_package()
