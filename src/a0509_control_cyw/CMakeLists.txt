cmake_minimum_required(VERSION 3.8)
project(a0509_control_cyw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

include_directories(include)
include_directories(${MODBUS_INCLUDE_DIRS})

add_executable(ee_relative_move_action src/ee_relative_move_action.cpp)
ament_target_dependencies(ee_relative_move_action
  rclcpp rclcpp_action geometry_msgs moveit_msgs yaml-cpp
  tf2 tf2_ros tf2_geometry_msgs
  moveit_core moveit_ros_planning moveit_ros_planning_interface
)

add_executable(arm_gripper_coordinator src/arm_gripper_coordinator.cpp src/gripper_controller.cpp)
ament_target_dependencies(arm_gripper_coordinator
  rclcpp rclcpp_action geometry_msgs moveit_msgs yaml-cpp
  tf2 tf2_ros tf2_geometry_msgs
  moveit_core moveit_ros_planning moveit_ros_planning_interface
)

target_link_libraries(arm_gripper_coordinator
  ${MODBUS_LIBRARIES}
)

install(TARGETS ee_relative_move_action
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS arm_gripper_coordinator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
