#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "inrof2025_ros_type/srv/waypoint.hpp"

class AddWaypoint
: public BT::RosServiceNode<inrof2025_ros_type::srv::Waypoint>
{
public:
    AddWaypoint(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params
    ) : BT::RosServiceNode<inrof2025_ros_type::srv::Waypoint>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts();
    
    bool setRequest(inrof2025_ros_type::srv::Waypoint::Request::SharedPtr& request) override;
    BT::NodeStatus onResponseReceived(const inrof2025_ros_type::srv::Waypoint::Response::SharedPtr& response) override;
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};