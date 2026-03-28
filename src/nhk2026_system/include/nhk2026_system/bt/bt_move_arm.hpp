#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <nhk2026_msgs/action/arm_move.hpp>

class MoveArmAction : public BT::RosActionNode<nhk2026_msgs::action::ArmMove>
{
public:
  MoveArmAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosActionNode<nhk2026_msgs::action::ArmMove>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult & result) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;
};
