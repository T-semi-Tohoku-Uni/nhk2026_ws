#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "inrof2025_ros_type/srv/ball_path.hpp"

class LinearPath
: public BT::RosServiceNode<inrof2025_ros_type::srv::BallPath>
{
public:
    LinearPath(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params
    ) : BT::RosServiceNode<inrof2025_ros_type::srv::BallPath>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts();

    bool setRequest(inrof2025_ros_type::srv::BallPath::Request::SharedPtr& request) override;
    BT::NodeStatus onResponseReceived(const inrof2025_ros_type::srv::BallPath::Response::SharedPtr& response) override;
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

};