cmake_minimum_required(VERSION 3.8)
project(dh_gripper_driver)

include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# find dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED) 

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# =========================
# Library: dh_gripper_driver (hardware plugin)
# =========================
add_library(
  dh_gripper_driver
  SHARED
  src/crc_utils.cpp
  src/data_utils.cpp
  src/hardware_interface.cpp
  src/default_driver.cpp
  src/default_driver_factory.cpp
)

target_link_libraries(dh_gripper_driver atomic ${LIBMODBUS_LIBRARIES})
target_include_directories(
  dh_gripper_driver
  PRIVATE
  include
  ${LIBMODBUS_INCLUDE_DIRS}
)

ament_target_dependencies(
  dh_gripper_driver
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# =========================
# Executable: joint_state_mimic_expander
# =========================
add_executable(joint_state_mimic_expander src/joint_state_mimic_expander.cpp)
ament_target_dependencies(joint_state_mimic_expander rclcpp sensor_msgs urdf)

# =========================
# INSTALL
# =========================
install(
  TARGETS dh_gripper_driver
  DESTINATION lib
)

install(
  TARGETS joint_state_mimic_expander
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# EXPORTS (library only)
# =========================
ament_export_include_directories(
  include
)
ament_export_libraries(
  dh_gripper_driver
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# =========================
# TEST
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
