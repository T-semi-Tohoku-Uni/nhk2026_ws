#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ServiceVacuum
: public BT::RosServiceNode<std_srvs::srv::SetBool>
{
public:
    ServiceVacuum(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params
    ) : BT::RosServiceNode<std_srvs::srv::SetBool>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts();

    bool setRequest(std_srvs::srv::SetBool::Request::SharedPtr& request) override;
    BT::NodeStatus onResponseReceived(const std_srvs::srv::SetBool::Response::SharedPtr& response) override;
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};