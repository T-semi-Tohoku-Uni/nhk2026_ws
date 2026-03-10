cmake_minimum_required(VERSION 3.8)
project(nhk2026_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trac_ik_lib QUIET)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(joint_state_publisher_ide
  src/joint_state_publisher_ide.cpp
)

ament_target_dependencies(joint_state_publisher_ide
  rclcpp
  std_msgs
  sensor_msgs
)

install(
  TARGETS joint_state_publisher_ide
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ide_arm_sim_bridge
  src/ide_arm_sim_bridge.cpp
)

ament_target_dependencies(ide_arm_sim_bridge
  rclcpp
  std_msgs
)

install(
  TARGETS ide_arm_sim_bridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(kdl_test
  src/kdl_test.cpp
)

ament_target_dependencies(kdl_test
  orocos_kdl
  kdl_parser
  ament_index_cpp
)

install(
  TARGETS kdl_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
