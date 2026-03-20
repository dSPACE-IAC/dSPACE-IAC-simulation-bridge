cmake_minimum_required(VERSION 3.16)
project(aurelion_ros2_bridge)

set (VESIAPI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/V-ESI-API/include)
set (VESIAPI_LIB_DIR /opt/VESI/lib/)
set(exe_name AurelionRos2BridgeNode)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 99)

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

set(MSG_DEPS sensor_msgs)

include_directories(
  include
)

set(
  dependencies
  rclcpp
  rosidl_default_generators
  sensor_msgs
)

install(
  DIRECTORY include/
  DESTINATION include/
)

add_executable(
  ${exe_name}
  src/aurelion_ros2_bridge.cpp
  include/aurelion_ros2_bridge.h
  )

link_directories(${VESIAPI_LIB_DIR})

ament_target_dependencies(
  ${exe_name}
  ${dependencies}
  )

target_link_libraries(
  ${exe_name}
  ${VESIAPI_LIB_DIR}libVESIAPI.so
)

target_include_directories(
  ${exe_name}
  PUBLIC ${VESIAPI_INCLUDE_DIR}
)

install(
  TARGETS ${exe_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
  DESTINATION lib
)

set_target_properties(
  ${exe_name} PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
