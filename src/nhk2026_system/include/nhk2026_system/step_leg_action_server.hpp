#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nhk2026_msgs/action/leg_move.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>

using namespace std::placeholders;

class StepLegActionServer
: public rclcpp::Node
{
public:
    StepLegActionServer();

private:
    using LegMove = nhk2026_msgs::action::LegMove;
    using GoalHandleLegMove = rclcpp_action::ServerGoalHandle<LegMove>;
    
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );

    rclcpp_action::Server<LegMove>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const LegMove::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLegMove> goal_handle
    );
    void handle_accepted(const std::shared_ptr<GoalHandleLegMove> goal_handle);
    void execute(const std::shared_ptr<GoalHandleLegMove> goal_handle);

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr step_legs_motor_publisher_;

    rclcpp::TimerBase::SharedPtr feedback_timer_;
    void feedback_timer_callback();

    std::atomic_bool goal_active_{false};
    geometry_msgs::msg::PoseStamped goal_pos_;
    geometry_msgs::msg::PoseStamped now_pos_;
    sensor_msgs::msg::JointState now_joint_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata);

    float kPosTolerance_;
    bool joint_subscribe_flag_{false};
};
