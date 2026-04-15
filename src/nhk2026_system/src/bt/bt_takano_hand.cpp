#include "bt/bt_takano_hand.hpp"

BT::PortsList TakanoHandAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>("step", 0, "which step to execute (0-6)"),
    BT::InputPort<double>("pos", 0.0f, "target position (not used)")
  });
}

bool TakanoHandAction::setGoal(Goal & goal)
{
  auto step = getInput<int>("step");
  auto pos = getInput<double>("pos");
  if (!step) {
    RCLCPP_ERROR(logger(), "%s: missing required input [step]", name().c_str());
    return false;
  }
  if (step.value() < 0 || step.value() > 6) {
    RCLCPP_ERROR(logger(), "%s: invalid step value %d", name().c_str(), step.value());
    return false;
  }
  if (!pos) {
    RCLCPP_ERROR(logger(), "%s: missing required input [pos]", name().c_str());
    return false;
  }
  goal.firststep = step.value();
  goal.finalstep = step.value();
  goal.pos = pos.value();
  RCLCPP_INFO(logger(), "%s: send hand goal for step %d", name().c_str(), step.value());
  return true;
}

BT::NodeStatus TakanoHandAction::onResultReceived(const WrappedResult & result)
{
  if (result.result && result.result->success) {
    RCLCPP_INFO(logger(), "%s: hand goal succeeded", name().c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(logger(), "%s: hand goal failed", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus TakanoHandAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  if (feedback) {
    RCLCPP_DEBUG(logger(), "%s: hand feedback:msg: %s",
                name().c_str(), feedback->msg.c_str());
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakanoHandAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: hand action failed with error code %d", name().c_str(), static_cast<int>(error));
  return BT::NodeStatus::FAILURE;
}

void TakanoHandAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halting hand action", name().c_str());
}