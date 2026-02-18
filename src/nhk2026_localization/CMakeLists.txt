cmake_minimum_required(VERSION 3.8)
project(nhk2026_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nanoflann REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HDF5 COMPONENTS C CXX REQUIRED)
find_package(OpenMP REQUIRED)

# 依存関係をまとめる関数
function(setup_node_dependencies target)
  ament_target_dependencies(${target}
    "rclcpp"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "sensor_msgs"
    "nav_msgs"
    "visualization_msgs"
    "tf2_geometry_msgs"
    "laser_geometry"
    "tf2_sensor_msgs"
  )

  target_include_directories(${target}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
      ${YAML_CPP_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
      ${HDF5_INCLUDE_DIRS}
  )

  target_link_libraries(${target} 
    ${YAML_CPP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    nanoflann::nanoflann
    Eigen3::Eigen
    ${HDF5_LIBRARIES}
    ${HDF5_CXX_LIBRARIES}
    OpenMP::OpenMP_CXX
  )
endfunction()

# ノード登録用の関数
function(ros_node_executable target_name source_file)
  add_executable(${target_name} ${source_file})
  setup_node_dependencies(${target_name})
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

# 実行ファイルの追加
ros_node_executable(mcl_node src/mcl_node.cpp)
ros_node_executable(vel_feedback_node src/vel_feedback_node.cpp)
ros_node_executable(vel_feedback_pass_through src/vel_feedback_pass_through.cpp)
ros_node_executable(joy2vel src/joy2vel.cpp)

# ディレクトリのインストール
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch worlds models urdf config map meshes
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