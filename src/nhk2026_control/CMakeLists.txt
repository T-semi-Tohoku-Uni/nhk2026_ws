cmake_minimum_required(VERSION 3.8)
project(nhk2026_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nhk2026_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)

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

add_executable(arm_path_plan
  src/arm_path_plan_server.cpp
)

ament_target_dependencies(arm_path_plan
  nav_msgs
  rclcpp
  nhk2026_msgs
  std_msgs
  orocos_kdl
  kdl_parser
  tf2
  urdf
)

install(
  TARGETS arm_path_plan
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
