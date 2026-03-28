#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "inrof2025_ros_type/action/follow.hpp"

class FollowRoute : public BT::RosActionNode<inrof2025_ros_type::action::Follow>
{
public:
    FollowRoute(
        const std::string& name,
        const BT::NodeConfiguration& config,
        const BT::RosNodeParams & params
    )
    : BT::RosActionNode<inrof2025_ros_type::action::Follow>(name, config, params)
    {
    }

    static BT::PortsList providedPorts();

    bool setGoal(Goal & goal) override;

    BT::NodeStatus onResultReceived(const WrappedResult & result) override;

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

    void onHalt() override;
};