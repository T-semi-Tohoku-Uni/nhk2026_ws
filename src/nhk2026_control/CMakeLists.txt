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
find_package(nhk2026_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

# すべてのノードで共通して使用する依存関係を網羅しておく
set(ALL_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  nhk2026_msgs
  std_srvs
  orocos_kdl
  kdl_parser
  tf2
  tf2_ros
  urdf
)

function(configure_nhk_target target_name)
  target_include_directories(${target_name} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

# 共通ビルド関数
function(add_nhk_node target_name source_file)
  add_executable(${target_name} ${source_file})
  ament_target_dependencies(${target_name} ${ALL_DEPENDENCIES})
  target_include_directories(${target_name} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )
  # ここで install も行う（ターゲット作成後なので安全）
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

# --- 実行ファイルの登録（ここですべて一括管理） ---
add_nhk_node(joint_state_publisher_ide src/joint_state_publisher_ide.cpp)
add_nhk_node(joint_state_publisher_step src/joint_state_publisher_step.cpp)
add_nhk_node(vacuum_server src/vacuum_service.cpp)
add_nhk_node(ide_arm_sim_bridge src/ide_arm_sim_bridge.cpp)
add_nhk_node(arm_path_plan src/arm_path_plan_server.cpp)
add_nhk_node(ide_arm_teaching src/ide_arm_teaching.cpp)
add_nhk_node(robot_pos_teaching src/robot_pos_teaching.cpp)
add_nhk_node(vacuum_stack_servier src/vacuum_stack_service.cpp)

target_compile_definitions(ide_arm_teaching PRIVATE
  NHK2026_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_definitions(robot_pos_teaching PRIVATE
  NHK2026_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)


# ディレクトリのインストール
install(DIRECTORY include launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
