#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nhk2026_msgs/action/arm_move.hpp"

using namespace std::placeholders;

class IdeArmActionServer
: public rclcpp::Node
{
public:
    IdeArmActionServer();

private:
    using ArmMove = nhk2026_msgs::action::ArmMove;
    using GoalHandleArmMove = rclcpp_action::ServerGoalHandle<ArmMove>;

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );

    rclcpp_action::Server<ArmMove> action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ArmMove::Goal> goal
    )
    {

    }
};