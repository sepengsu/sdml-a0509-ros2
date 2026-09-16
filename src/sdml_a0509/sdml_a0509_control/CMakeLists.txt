cmake_minimum_required(VERSION 3.8)
project(sdml_a0509_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(dsr_msgs2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(jogging src/jogging.cpp)
target_compile_features(jogging PUBLIC cxx_std_17)
ament_target_dependencies(jogging
  rclcpp
  dsr_msgs2
)

install(TARGETS jogging
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pick_and_place_node
  scripts/merge_node
  scripts/jog_string_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

