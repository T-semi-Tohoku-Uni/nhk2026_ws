cmake_minimum_required(VERSION 3.8)
project(nhk2026_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nhk2026_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)

# すべてのノードで共通して使用する依存関係
set(ALL_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
)

function(configure_nhk_target target_name)
  target_include_directories(${target_name} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_executable(ide_arm_sim_bridge
  src/ide_arm_sim_bridge.cpp
)

ament_target_dependencies(ide_arm_sim_bridge
  rclcpp
  std_msgs
)

configure_nhk_target(ide_arm_sim_bridge)

add_executable(arm_path_plan
  src/arm_path_plan_server.cpp
)

ament_target_dependencies(arm_path_plan
  rclcpp
  geometry_msgs
  nav_msgs
  nhk2026_msgs
  std_msgs
  orocos_kdl
  kdl_parser
  tf2
  urdf
)

configure_nhk_target(arm_path_plan)

function(add_nhk_node target_name source_file)
  add_executable(${target_name} ${source_file})
  ament_target_dependencies(${target_name} ${ALL_DEPENDENCIES})
  configure_nhk_target(${target_name})
endfunction()

# 実行ファイルの登録
add_nhk_node(joint_state_publisher_ide src/joint_state_publisher_ide.cpp)
add_nhk_node(joint_state_publisher_step src/joint_state_publisher_step.cpp)

install(DIRECTORY include launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
