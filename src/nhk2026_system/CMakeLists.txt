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

# 依存関係をまとめる関数
function(setup_node_dependencies target)
  # このパッケージ内で使う可能性のある依存関係をすべてまとめてリンクします
  ament_target_dependencies(${target}
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    std_msgs
    geometry_msgs
    sensor_msgs
    lifecycle_msgs
    nhk2026_msgs
  )

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


# 実行ファイルの追加
ros_node_executable(system_1st_video src/system_1st_video.cpp)
ros_node_executable(ide_arm_action_server src/ide_arm_action_server.cpp)
ros_node_executable(step_leg_action_server src/step_leg_action_server.cpp)
ros_node_executable(step_action_server src/step_action_server.cpp)


# ディレクトリのインストール
install(
  DIRECTORY config launch include
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()