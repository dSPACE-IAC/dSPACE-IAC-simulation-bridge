cmake_minimum_required(VERSION 3.16)
project(sut_te_bridge)

set (VESIAPI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/V-ESI-API)
set(exe_name SutTeBridgeNode)

set(DCMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 99)

include_directories(/opt/ros/iron/include/tf2)
include_directories(/opt/ros/iron/include/tf2_ros)

# find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonoma_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(npc_controller_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(MSG_DEPS std_msgs)
set(MSG_DEPS sensor_msgs)
set(MSG_DEPS geometry_msgs)
set(MSG_DEPS autonoma_msgs)
set(MSG_DEPS vectornav_msgs)
set(MSG_DEPS novatel_oem7_msgs)
set(MSG_DEPS npc_controller_msgs)
set(MSG_DEPS rosgraph_msgs)

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rosidl_default_generators
  autonoma_msgs
  vectornav_msgs
  novatel_oem7_msgs
  npc_controller_msgs
  foxglove_msgs
  rosgraph_msgs
  tf2
)

install(DIRECTORY include/
  DESTINATION include/
)

add_executable(${exe_name} src/bridge.cpp include/bridge.h)

link_directories(${VESIAPI_DIR}/lib/)

ament_target_dependencies(${exe_name} ${dependencies})

target_link_libraries(
  ${exe_name}
  ${VESIAPI_DIR}/lib/linux/libVESIAPI.so
)

target_include_directories(${exe_name}
  PUBLIC include/sut-te-bridge
  PUBLIC include/V-ESI-API/include
)

install(TARGETS ${exe_name}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
