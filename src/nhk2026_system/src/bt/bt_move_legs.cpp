#include "bt/bt_move_legs.hpp"

BT::PortsList MoveLegAction::providedPorts()
{
    return {
        BT::InputPort<double>("right_angle"),
        BT::InputPort<double>("left_angle"),
        BT::InputPort<double>("back_angle")
    };
}

bool MoveLegAction::setGoal(Goal & goal)
{
    double right_angle, left_angle, back_angle;
    if (!getInput<double>("right_angle", right_angle) || !getInput<double>("left_angle", left_angle) || !getInput<double>("back_angle", back_angle))
    {
        RCLCPP_ERROR(logger(), "Failed to get input ports");
        return false;
    }
    goal.joint_states.position = {right_angle, left_angle, back_angle};
    goal.max_acc = 5.0;
    goal.max_speed = 5.0;
    return true;
}

BT::NodeStatus MoveLegAction::onResultReceived(const WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus MoveLegAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    static_cast<void>(feedback);
    // todo フィードバックを受け取る
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveLegAction::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "Action failed with error code: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}

void MoveLegAction::onHalt()
{
    RCLCPP_INFO(logger(), "Halting MoveLegAction");
    // todo アクションをキャンセルする
}