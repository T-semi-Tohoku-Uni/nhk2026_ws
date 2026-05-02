#pragma once

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "nhk2026_msgs/msg/aruco_pose.hpp"

class ArucoPoseSub
: public BT::RosTopicSubNode<nhk2026_msgs::msg::ArucoPose>
{
public:
    ArucoPoseSub(
        const std::string & name,
        const BT::NodeConfig & conf,
        const BT::RosNodeParams & params)
    : BT::RosTopicSubNode<nhk2026_msgs::msg::ArucoPose>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(const std::shared_ptr<nhk2026_msgs::msg::ArucoPose>& msg) override;
    bool latchLastMessage() const override { return false; }
private:
    bool armed_ = false;
};