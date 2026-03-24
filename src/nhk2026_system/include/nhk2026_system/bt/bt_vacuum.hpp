#pragma once

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <std_msgs/msg/bool.hpp>

class PublisherVacuum
: public BT::RosTopicPubNode<std_msgs::msg::Bool>
{
public:
    PublisherVacuum(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params
    ) : BT::RosTopicPubNode<std_msgs::msg::Bool>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts();

    bool setMessage(std_msgs::msg::Bool& msg) override;
};