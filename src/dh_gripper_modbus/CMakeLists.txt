cmake_minimum_required(VERSION 3.10)
project(dh_gripper_modbus)

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 실행 파일
add_executable(gripper_controller src/dh_gripper_controller)
ament_target_dependencies(gripper_controller rclcpp std_msgs)
target_link_libraries(gripper_controller modbus)

# 설치
install(TARGETS gripper_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
