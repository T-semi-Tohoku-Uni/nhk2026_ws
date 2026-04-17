cmake_minimum_required(VERSION 3.8)
project(nhk2026_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(IS_JETSON "Build on jetson" OFF)
# find dependencies
find_package(ament_cmake REQUIRED)


if(IS_JETSON)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(realsense2_camera_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nhk2026_msgs REQUIRED)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
  find_package(std_msgs REQUIRED)
  find_package(backward_ros REQUIRED)

  add_executable(aruco_node
    src/aruco_node.cpp
  )

  ament_target_dependencies(aruco_node
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    realsense2_camera_msgs
    cv_bridge
    geometry_msgs
    nhk2026_msgs
  )

  target_include_directories(aruco_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
  )

  install(
    TARGETS aruco_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
  )

  add_library(aruco
    src/aruco.cpp
  )

  target_include_directories(aruco PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(aruco
    ${OpenCV_LIBRARIES}
  )

  target_link_libraries(aruco_node
    aruco
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
  )
else()
  message(STATUS "Skipping aruco_node because NHK2026_BUILD_ARUCO_NODE is disabled")
endif()

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
