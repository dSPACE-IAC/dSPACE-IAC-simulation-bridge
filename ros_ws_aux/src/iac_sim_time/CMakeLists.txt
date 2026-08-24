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
  add_executable(sim_clock_mode_test test/sim_clock_mode_test.cpp)
  target_include_directories(sim_clock_mode_test PRIVATE include)
  add_test(NAME iac_sim_time_clock_mode COMMAND sim_clock_mode_test)
endif()

ament_export_include_directories(include)
ament_package()