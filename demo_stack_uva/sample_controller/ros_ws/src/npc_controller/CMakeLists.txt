cmake_minimum_required(VERSION 3.8)
project(npc_controller)

include(CTest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(npc_controller_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autonoma_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(GEOGRAPHICLIB REQUIRED)
find_package(ALGLIB REQUIRED)

include_directories(include)

add_library(controller SHARED
  src/npc_controller.cpp
  src/dbc_structure_init.cpp
  include/npc_controller.hpp
)

set(MSG_DEPS autonoma_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_target_dependencies(controller
  rclcpp
  rclcpp_components
  vectornav_msgs
  novatel_oem7_msgs
  raptor_dbw_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  npc_controller_msgs
  autonoma_msgs
  rosgraph_msgs
)

target_link_libraries(controller ${GeographicLib_LIBRARIES} ${ALGLIB_LIBRARIES})

if(BUILD_TESTING)
  add_executable(sim_clock_control_test test/sim_clock_control_test.cpp)
  target_include_directories(sim_clock_control_test PRIVATE include)
  add_test(NAME npc_controller_sim_clock_control COMMAND sim_clock_control_test)
endif()

target_include_directories(controller PUBLIC
  ${GeographicLib_INCLUDE_DIRS}
  ${ALGLIB_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(DIRECTORY 
  include/
  DESTINATION include/
)

install(TARGETS controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(controller
  PLUGIN "controller::ControllerNode"
  EXECUTABLE controller_exec
)

ament_auto_package(INSTALL_TO_SHARE
  config
  maps
  launch
)
