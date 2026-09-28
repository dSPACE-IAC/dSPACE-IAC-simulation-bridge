cmake_minimum_required(VERSION 3.5)
project(raptor_dbw_can)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_executable(${PROJECT_NAME}_node
  src/node.cpp
  src/DbwNode.cpp
)

ament_target_dependencies(raptor_dbw_can_node npc_controller_msgs)

target_compile_options(${PROJECT_NAME}_node PRIVATE -Wno-unused-function)

if(BUILD_TESTING)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/report_forwarding_test.cpp")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_executable(dbw_report_forwarding_test
    test/report_forwarding_test.cpp
    src/DbwNode.cpp
  )
  target_include_directories(dbw_report_forwarding_test PRIVATE include)
  target_compile_definitions(dbw_report_forwarding_test PRIVATE
    RAPTOR_DBC_FILE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/launch/CAN1-INDY-V26.dbc\"
  )
  ament_target_dependencies(dbw_report_forwarding_test
    rclcpp
    can_msgs
    raptor_dbw_msgs
    npc_controller_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    can_dbc_parser
  )
  add_test(NAME raptor_dbw_can_report_forwarding COMMAND dbw_report_forwarding_test)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
