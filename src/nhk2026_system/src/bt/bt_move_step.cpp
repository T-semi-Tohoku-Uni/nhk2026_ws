#include "bt/bt_move_step.hpp"

BT::PortsList StepMoveAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("command", "step up")
  });
}

bool StepMoveAction::setGoal(Goal & goal)
{
  auto command = getInput<std::string>("command");
  if (!command) {
    RCLCPP_ERROR(logger(), "%s: missing required input: command", name().c_str());
    return false;
  }
  goal.msg = command.value();
  RCLCPP_INFO(logger(), "%s: send step move goal: %s", name().c_str(), goal.msg.c_str());
  return true;
}

BT::NodeStatus StepMoveAction::onResultReceived(const WrappedResult & result)
{
  if (result.result && result.result->success) {
    RCLCPP_INFO(logger(), "%s: step move succeeded", name().c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(logger(), "%s: step move failed", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus StepMoveAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: step move feedback: %s", name().c_str(), feedback->msg.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StepMoveAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: step move action failed with error code %d", name().c_str(), error);
  return BT::NodeStatus::FAILURE;
}

void StepMoveAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: step move action halted", name().c_str());
}