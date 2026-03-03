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

# 依存関係をまとめる関数
function(setup_node_dependencies target)
  # このパッケージ内で使う依存関係をリンクします
  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    sensor_msgs
  )

  # インクルードディレクトリの設定
  target_include_directories(${target}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
  )
endfunction()

# ノード登録用の関数
function(ros_node_executable target_name source_file)
  add_executable(${target_name} ${source_file})
  setup_node_dependencies(${target_name})
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()


# 実行ファイルの追加（1行ずつ書くだけ！）
ros_node_executable(joint_state_publisher_ide src/joint_state_publisher_ide.cpp)
ros_node_executable(joint_state_publisher_step src/joint_state_publisher_step.cpp)



install(
  DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
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
  src/arm_path_plan.cpp
)

ament_target_dependencies(arm_path_plan
  nav_msgs
  rclcpp
  nhk2026_msgs
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