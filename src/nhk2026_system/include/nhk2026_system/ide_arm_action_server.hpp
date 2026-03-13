#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "nhk2026_msgs/action/arm_move.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nhk2026_msgs/srv/arm_path_plan.hpp"

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <atomic>

using namespace std::placeholders;

class IdeArmActionServer
: public rclcpp::Node
{
public:
    IdeArmActionServer();

private:
    using ArmMove = nhk2026_msgs::action::ArmMove;
    using GoalHandleArmMove = rclcpp_action::ServerGoalHandle<ArmMove>;

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ArmMove::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleArmMove> goal_handle
    );
    void handle_accepted(const std::shared_ptr<GoalHandleArmMove> goal_handle);
    void execute(const std::shared_ptr<GoalHandleArmMove> goal_handle);
    void feedback_timer_callback();
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata);
    void robot_description_callback(const std_msgs::msg::String::SharedPtr rxdata);

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rclcpp_action::Server<ArmMove>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr j1_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr j2_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr j3_motor_publisher_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
    rclcpp::Client<nhk2026_msgs::srv::ArmPathPlan>::SharedPtr path_client_;

    std::atomic_bool goal_active_{false};
    std::shared_ptr<GoalHandleArmMove> active_goal_handle_;
    float kPosTolerance_;
    bool joint_subscribe_flag_{false};
    bool robot_description_flag_{false};
    geometry_msgs::msg::PoseStamped goal_pos_;
    geometry_msgs::msg::PoseStamped now_pos_;
    sensor_msgs::msg::JointState now_joint_;
    KDL::Chain chain;
    std::string joint1_name_;
    std::string joint2_name_;
    std::string joint3_name_;

    std::vector<double> joint_positions_;
};
