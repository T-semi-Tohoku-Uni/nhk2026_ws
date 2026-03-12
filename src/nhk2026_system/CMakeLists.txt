cmake_minimum_required(VERSION 3.8)
project(nhk2026_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nhk2026_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trac_ik_lib QUIET)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(system_1st_video
  src/system_1st_video.cpp
)

ament_target_dependencies(system_1st_video
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  lifecycle_msgs
  nhk2026_msgs
)

install(
  TARGETS system_1st_video
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ide_arm_action_server
  src/ide_arm_action_server.cpp
)

ament_target_dependencies(ide_arm_action_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nhk2026_msgs
  orocos_kdl
  kdl_parser
  ament_index_cpp
)

install(
  TARGETS ide_arm_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# add install
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
