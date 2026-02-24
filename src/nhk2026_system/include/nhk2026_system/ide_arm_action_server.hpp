#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nhk2026_msgs/action/arm_move.hpp"

class IdeArmActionServer
: public rclcpp::Node
{
public:
    IdeArmActionServer();

private:
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );
};