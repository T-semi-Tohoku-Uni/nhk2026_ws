#include "bt/bt_vacuum.hpp"

BT::PortsList ServiceVacuum::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<bool>("value")
    });
}

bool ServiceVacuum::setRequest(std_srvs::srv::SetBool::Request::SharedPtr& request)
{
    auto value = getInput<bool>("value");
    if (!value) return false;

    request->data = value.value();
    return true;
}

BT::NodeStatus ServiceVacuum::onResponseReceived(const std_srvs::srv::SetBool::Response::SharedPtr& response)
{
    if (!response || !response->success)return BT::NodeStatus::FAILURE;
    
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ServiceVacuum::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "vacuum service error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
}