
cmake_minimum_required(VERSION 3.18)
project(inverted_pendulum_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  cactus_rt
  GIT_REPOSITORY https://github.com/cactusdynamics/cactus-rt.git
  GIT_TAG        ...
)
FetchContent_MakeAvailable(cactus_rt)

add_executable(${PROJECT_NAME}
  main.cc
  data_logger_thread.cc
  rt_thread.cc
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
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

