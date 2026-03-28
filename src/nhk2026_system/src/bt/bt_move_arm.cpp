#include "bt/bt_move_arm.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

BT::PortsList MoveArmAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("y"),
    BT::InputPort<double>("z"),
    BT::InputPort<double>("roll", 0.0, "Target roll in radians"),
    BT::InputPort<std::string>("frame_id", "arm_base", "Target frame"),
    BT::InputPort<double>("max_speed", 0.2, "Max speed"),
    BT::InputPort<double>("max_acc", 0.2, "Max acceleration")
  });
}

bool MoveArmAction::setGoal(Goal & goal)
{
  auto y = getInput<double>("y");
  auto z = getInput<double>("z");
  auto roll = getInput<double>("roll");
  auto frame_id = getInput<std::string>("frame_id");
  auto max_speed = getInput<double>("max_speed");
  auto max_acc = getInput<double>("max_acc");

  if (!y || !z || !roll || !frame_id || !max_speed || !max_acc) {
    RCLCPP_ERROR(logger(), "%s: missing required input", name().c_str());
    return false;
  }

  tf2::Quaternion q;
  q.setRPY(roll.value(), 0, 0);
  q.normalize();

  goal.goal_pos.header.stamp = now();
  goal.goal_pos.header.frame_id = frame_id.value();
  goal.goal_pos.pose.position.x = -0.182751;
  goal.goal_pos.pose.position.y = y.value();
  goal.goal_pos.pose.position.z = z.value();
  goal.goal_pos.pose.orientation = tf2::toMsg(q);
  goal.max_speed = static_cast<float>(max_speed.value());
  goal.max_acc = static_cast<float>(max_acc.value());
  goal.waypoints.clear();

  RCLCPP_INFO(
    logger(),
    "%s: send arm goal to %s (%.3f, %.3f, %.3f)",
    name().c_str(),
    goal.goal_pos.header.frame_id.c_str(),
    goal.goal_pos.pose.position.x,
    goal.goal_pos.pose.position.y,
    goal.goal_pos.pose.position.z);
  return true;
}

BT::NodeStatus MoveArmAction::onResultReceived(const WrappedResult & result)
{
  if (result.result && result.result->success) {
    RCLCPP_INFO(logger(), "%s: arm goal succeeded", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  const std::string msg = result.result ? result.result->msg : "empty result";
  RCLCPP_ERROR(logger(), "%s: arm goal failed: %s", name().c_str(), msg.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveArmAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  if (feedback) {
    RCLCPP_DEBUG(logger(), "%s: %s", name().c_str(), feedback->msg.c_str());
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArmAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: action failure: %s", name().c_str(), BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

void MoveArmAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: halted", name().c_str());
}
