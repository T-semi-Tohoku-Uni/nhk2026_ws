cmake_minimum_required(VERSION 3.8)
project(nhk2026_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
# find_package(nhk2026_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(inrof2025_ros_type REQUIRED)
find_package(HDF5 COMPONENTS C CXX REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(ignition-msgs8 REQUIRED)


add_executable(pursuit src/pursuit.cpp)
add_executable(path_planner src/path_planner.cpp)
add_executable(blossom_path_planner src/blossom_path_planner.cpp)

target_link_libraries(path_planner
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES})


ament_target_dependencies(pursuit
  rclcpp
  geometry_msgs
  nav_msgs
  rclcpp_action
  tf2_geometry_msgs
  tf2
  # nhk2026_msgs
  visualization_msgs
  inrof2025_ros_type
)

ament_target_dependencies(path_planner
  rclcpp
  OpenCV
  nav_msgs
  geometry_msgs
  std_msgs
  # nhk2026_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  inrof2025_ros_type
  )

  ament_target_dependencies(blossom_path_planner
  rclcpp
  OpenCV
  nav_msgs
  geometry_msgs
  std_msgs
  # nhk2026_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  inrof2025_ros_type
  ament_index_cpp
  )
  

target_include_directories(pursuit 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${ignition-transport8_INCLUDE_DIRS}
    ${ignition-msgs8_INCLUDE_DIRS}
)

target_include_directories(path_planner
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${HDF5_INCLUDE_DIRS}

)


target_include_directories(blossom_path_planner
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${HDF5_INCLUDE_DIRS}

)


target_link_libraries(path_planner
  ${HDF5_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
)

target_link_libraries(pursuit
  ${ignition-transport8_LIBRARIES}
  ${ignition-msgs8_LIBRARIES}
)

target_link_libraries(blossom_path_planner
  ${HDF5_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
  nlohmann_json::nlohmann_json
)


install(TARGETS pursuit DESTINATION lib/${PROJECT_NAME})
install(TARGETS path_planner DESTINATION lib/${PROJECT_NAME})
install(TARGETS blossom_path_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )


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