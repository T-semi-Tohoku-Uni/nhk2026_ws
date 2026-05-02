#pragma once

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "angles/angles.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RotateSub
: public BT::RosTopicSubNode<geometry_msgs::msg::Pose>
{
public:
    RotateSub(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params
    ) : BT::RosTopicSubNode<geometry_msgs::msg::Pose>(name, conf, params) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::Pose>& last_msg) override;
    bool latchLastMessage() const override { return true; }
};