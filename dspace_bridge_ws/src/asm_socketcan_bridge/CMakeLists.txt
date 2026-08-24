cmake_minimum_required(VERSION 3.16)
project(asm_socketcan_bridge)

include(CTest)

set (VESIAPI_INCLUDE_DIR /opt/VESI/include)
set (VESIAPI_LIB_DIR /opt/VESI/lib/)
set(exe_name AsmSocketCanBridgeNode)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 99)

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonoma_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)

set(MSG_DEPS std_msgs)
set(MSG_DEPS rosgraph_msgs)
set(MSG_DEPS sensor_msgs)
set(MSG_DEPS foxglove_msgs)
set(MSG_DEPS geometry_msgs)
set(MSG_DEPS autonoma_msgs)
set(MSG_DEPS vectornav_msgs)
set(MSG_DEPS novatel_oem7_msgs)

include_directories(
  include
)

set(
  dependencies
  rclcpp
  rosidl_default_generators
  tf2
  tf2_ros
  std_msgs
  rosgraph_msgs
  sensor_msgs
  foxglove_msgs
  geometry_msgs
  autonoma_msgs
  vectornav_msgs
  novatel_oem7_msgs
)

install(
  DIRECTORY include/
  DESTINATION include/
)

add_executable(
  ${exe_name}
  src/asm_socketcan_bridge.cpp
  src/asm_socketcan_bridge_main.cpp
  src/asm_socketcan_bridge_comm.cpp
  src/asm_socketcan_bridge_can_codec.cpp
  src/asm_socketcan_bridge_can_reader.cpp
  src/asm_socketcan_bridge_can_transport.cpp
  src/asm_socketcan_bridge_setup.cpp
  src/asm_socketcan_bridge_sensor_converters.cpp
  src/asm_socketcan_bridge_sensor_publishers.cpp
  src/asm_socketcan_bridge_can_publishers.cpp
  src/signal_codec.cpp
  src/dbc_structure_init.cpp
  include/signal_codec.h
  include/asm_socketcan_bridge.h
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

if(BUILD_TESTING)
  add_executable(sim_time_test test/sim_time_test.cpp)
  target_include_directories(sim_time_test PRIVATE include)
  add_test(NAME asm_socketcan_bridge_sim_time COMMAND sim_time_test)
endif()

target_include_directories(
  ${exe_name}
  PUBLIC include/asm_socketcan_bridge
  PUBLIC ${VESIAPI_INCLUDE_DIR}
)

install(
  TARGETS ${exe_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

set_target_properties(
  ${exe_name} PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
