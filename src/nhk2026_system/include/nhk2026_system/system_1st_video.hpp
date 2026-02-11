#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class System1stVideo
: public rclcpp_lifecycle::LifecycleNode
{
public:
    System1stVideo();
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );
    
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_ui_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr back_arm_robstride_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr middle_arm_robstride_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr back_robomastar_subscription_;

    void cmd_vel_ui_callback(geometry_msgs::msg::Twist::SharedPtr rxdata);
    void back_arm_robstride_callback(std_msgs::msg::Float32MultiArray::SharedPtr rxdata);
    void middle_arm_robstride_callback(std_msgs::msg::Float32MultiArray::SharedPtr rxdata);
    void back_robomastar_callback(std_msgs::msg::Float32MultiArray::SharedPtr rxdata);
};