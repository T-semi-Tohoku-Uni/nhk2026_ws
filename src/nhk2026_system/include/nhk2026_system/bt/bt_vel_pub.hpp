#pragma once

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <geometry_msgs/msg/twist.hpp>

class BtVelPub : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
  BtVelPub(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params
  ) : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(name, conf, params)
  {}

  static BT::PortsList providedPorts();
  bool setMessage(geometry_msgs::msg::Twist & msg) override;
};