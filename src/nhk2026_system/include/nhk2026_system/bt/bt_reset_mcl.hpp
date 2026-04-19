#pragma once

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ResetMcl
    : public BT::RosTopicPubNode<geometry_msgs::msg::Pose>
{
public:
    ResetMcl(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const BT::RosNodeParams & params
    ) : BT::RosTopicPubNode<geometry_msgs::msg::Pose>(name, config, params)
    {
    }

    static BT::PortsList providedPorts();
    bool setMessage(geometry_msgs::msg::Pose & msg) override;
};