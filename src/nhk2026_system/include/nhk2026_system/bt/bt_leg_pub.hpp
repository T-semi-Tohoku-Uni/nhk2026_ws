#pragma once

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class BtLegPub
: public BT::RosTopicPubNode<std_msgs::msg::Float32MultiArray>
{
public:
  BtLegPub(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf,
    const BT::RosNodeParams & params
  ) : BT::RosTopicPubNode<std_msgs::msg::Float32MultiArray>(xml_tag_name, conf, params)
  {
  }

  static BT::PortsList providedPorts();
  bool setMessage(std_msgs::msg::Float32MultiArray & msg) override;
};