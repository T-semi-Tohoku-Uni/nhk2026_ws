#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "nhk2026_msgs/msg/aruco_pose.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include "aruco.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ArucoNode
: public rclcpp_lifecycle::LifecycleNode
{
public:
    ArucoNode();

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );

    void img_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr rxdata);

    std::unique_ptr<ArucoDetect> aruco_detector;
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr img_subscriber;
    rclcpp_lifecycle::LifecyclePublisher<nhk2026_msgs::msg::ArucoPose>::SharedPtr aruco_pose_publisher;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_hanle_;

    double markersize;
};