cmake_minimum_required(VERSION 3.8)
project(iac_sim_time)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  include(CTest)
  add_executable(sim_step_marker_test test/sim_step_marker_test.cpp)
  target_include_directories(sim_step_marker_test PRIVATE include)
  add_test(NAME iac_sim_time_step_marker COMMAND sim_step_marker_test)
endif()

ament_export_include_directories(include)
ament_package()