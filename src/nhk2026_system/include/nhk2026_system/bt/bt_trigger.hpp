#pragma once

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/bool.hpp>

class TestTrigger
: public BT::RosTopicSubNode<std_msgs::msg::Bool>
{
public:
    TestTrigger(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& msg) override;
  bool latchLastMessage() const override { return false; }

private:
  bool armed_ = false;
};