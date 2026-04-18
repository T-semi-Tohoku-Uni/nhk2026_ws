#pragma once

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "geometry_msgs/msg/pose.hpp"

class RotateSub
: public BT::RosTopicSubNode<geometry_msgs::msg::Pose>
{
};