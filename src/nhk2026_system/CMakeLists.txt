cmake_minimum_required(VERSION 3.8)
project(nhk2026_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# find dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nhk2026_msgs REQUIRED)
find_package(inrof2025_ros_type REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# ide_arm_action_server
# -----------------------------------------------------------------------------
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
  urdf
)
install(TARGETS ide_arm_action_server DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# step_leg_action_server (追加)
# -----------------------------------------------------------------------------
add_executable(step_leg_action_server
  src/step_leg_action_server.cpp
)
ament_target_dependencies(step_leg_action_server
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
install(TARGETS step_leg_action_server DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# step_action_server
# -----------------------------------------------------------------------------
add_executable(step_action_server
  src/step_action_server.cpp
)
ament_target_dependencies(step_action_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  nhk2026_msgs
)
install(TARGETS step_action_server DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# kaneko_arm_action_server
# -----------------------------------------------------------------------------
add_executable(kaneko_arm_action_server
  src/kaneko_arm_action_server.cpp
)
ament_target_dependencies(kaneko_arm_action_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nhk2026_msgs
  orocos_kdl
  kdl_parser
)
install(TARGETS kaneko_arm_action_server DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# bt_library
# -----------------------------------------------------------------------------
add_library(bt_library
  src/bt/bt_move_arm.cpp
  src/bt/bt_vacuum.cpp
  src/bt/bt_trigger.cpp
  src/bt/bt_generate_route.cpp
  src/bt/bt_linear_path.cpp
  src/bt/bt_follow_route.cpp
  src/bt/bt_takano_hand.cpp
)
ament_target_dependencies(bt_library
  nhk2026_msgs
  std_srvs
  std_msgs
  inrof2025_ros_type
  tf2
  tf2_geometry_msgs
)
target_link_libraries(bt_library behaviortree_ros2::behaviortree_ros2)

# -----------------------------------------------------------------------------
# ide_arm_bt_node
# -----------------------------------------------------------------------------
add_executable(ide_arm_bt_node
  src/ide_arm_bt_node.cpp
)
ament_target_dependencies(ide_arm_bt_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nhk2026_msgs
  ament_index_cpp
  tf2
  tf2_geometry_msgs
)
target_link_libraries(ide_arm_bt_node
  behaviortree_ros2::behaviortree_ros2
  bt_library
)
install(TARGETS ide_arm_bt_node DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# path_bt_node
# -----------------------------------------------------------------------------
add_executable(path_bt_node
  src/path_bt_node.cpp
)
ament_target_dependencies(path_bt_node
  rclcpp
  rclcpp_action
  inrof2025_ros_type
  ament_index_cpp
)
target_link_libraries(path_bt_node
  behaviortree_ros2::behaviortree_ros2
  bt_library
)
install(TARGETS path_bt_node DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# video_2nd_bt_node
# -----------------------------------------------------------------------------
add_executable(video_2nd_bt_node
  src/video_2nd_bt_node.cpp
)
ament_target_dependencies(video_2nd_bt_node
  rclcpp
  rclcpp_action
  inrof2025_ros_type
  ament_index_cpp
)
target_link_libraries(video_2nd_bt_node
  behaviortree_ros2::behaviortree_ros2
  bt_library
)
install(TARGETS video_2nd_bt_node DESTINATION lib/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# Install other directories
# -----------------------------------------------------------------------------
install(
  DIRECTORY config launch include
  DESTINATION share/${PROJECT_NAME}
)

add_executable(takano_hand_server src/takano_hand_server.cpp)
ament_target_dependencies(takano_hand_server
  rclcpp 
  rclcpp_action 
  std_msgs 
  nhk2026_msgs
)

install(TARGETS
  takano_hand_server
  # (ここには他のノード名も並んでいるはずです)
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
