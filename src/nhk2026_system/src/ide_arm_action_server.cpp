#include "ide_arm_action_server.hpp"

IdeArmActionServer::IdeArmActionServer()
: rclcpp::Node("ide_arm_action_server")
{
    
}

rcl_interfaces::msg::SetParametersResult IdeArmActionServer::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{

}

rclcpp_action::GoalResponse IdeArmActionServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMove::Goal> goal
)
{
    
}

rclcpp_action::CancelResponse IdeArmActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleArmMove> goal_handle
)
{

}

void IdeArmActionServer::handle_accepted(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{

}

int main()
{

}