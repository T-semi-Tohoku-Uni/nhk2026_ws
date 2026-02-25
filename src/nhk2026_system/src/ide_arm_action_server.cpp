#include "ide_arm_action_server.hpp"

using namespace std::chrono_literals;

IdeArmActionServer::IdeArmActionServer()
: rclcpp::Node("ide_arm_action_server")
{
    this->action_server_ = rclcpp_action::create_server<ArmMove>(
        this,
        "ide_arm",
        std::bind(&IdeArmActionServer::handle_goal, this, _1, _2),
        std::bind(&IdeArmActionServer::handle_cancel,this, _1),
        std::bind(&IdeArmActionServer::handle_accepted, this, _1)
    );

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&IdeArmActionServer::parameters_callback, this, _1)
    );
}

rcl_interfaces::msg::SetParametersResult IdeArmActionServer::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;

    if (disable_set_parameter)
    {
        result.successful = false;
        result.reason = "action is processing";
        return result;
    }

    for (const auto &param : parameters) {
        
    }

    result.successful = true;
    result.reason = "success";
    return result;
}

rclcpp_action::GoalResponse IdeArmActionServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMove::Goal> goal
)
{
    // todo アームの到達範囲をチェック
    RCLCPP_INFO(this->get_logger(), "arm start to move!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IdeArmActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleArmMove> goal_handle
)
{
    // todo 動作の終了とか？
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void IdeArmActionServer::handle_accepted(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->j1_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("j1"),
        device
    );
    this->j2_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("j2"),
        device
    );
    this->j3_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("j3"),
        device
    );
    this->feedback_timer_ = this->create_wall_timer(
        0.05s,
        std::bind(&IdeArmActionServer::feedback_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    std::thread{std::bind(&IdeArmActionServer::execute, this, _1), goal_handle}.detach();
}

void IdeArmActionServer::execute(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{

}

void IdeArmActionServer::feedback_timer_callback()
{

}

int main()
{

}