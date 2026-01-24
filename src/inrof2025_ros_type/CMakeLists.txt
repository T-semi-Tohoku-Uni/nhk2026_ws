cmake_minimum_required(VERSION 3.8)
project(inrof2025_ros_type)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#— find ament and rosidl generator
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#— find any message‐package dependencies used by your .srv/.action
#   例えば、GenRoute.srv で nav_msgs/Path を返すなら nav_msgs、
#   Follow.action のフィードバックで geometry_msgs/PoseStamped を使うなら geometry_msgs が必要です
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ライセンスチェックをスキップ
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint をスキップ
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(srv_files
  "srv/GenRoute.srv"
  "srv/Vacume.srv"
  "srv/BallPose.srv"
  "srv/Pose.srv"
  "srv/BallPath.srv"
  "srv/BallColor.srv"
  "srv/Waypoint.srv"
)
set(action_files
  "action/Follow.action"
  "action/Rotate.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    nav_msgs
    geometry_msgs
)

ament_package()