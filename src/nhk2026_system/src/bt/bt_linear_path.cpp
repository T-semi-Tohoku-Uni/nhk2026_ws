#include "bt/bt_linear_path.hpp"

BT::PortsList LinearPath::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<bool>("is_return"),
    });
}

bool LinearPath::setRequest(inrof2025_ros_type::srv::BallPath::Request::SharedPtr& request)
{
    auto x = getInput<double>("x");
    auto y = getInput<double>("y");
    auto is_return = getInput<bool>("is_return");

    if (!x) {
        throw BT::RuntimeError("missing required input x: ", x.error() );
    }
    if (!y) {
        throw BT::RuntimeError("missing required input y: ", y.error() );
    }
    if (!is_return) {
        throw BT::RuntimeError("missing required input is_return: ", is_return.error() );
    }

    request->x = x.value();
    request->y = y.value();
    request->is_return = is_return.value();

    return true;
}

BT::NodeStatus LinearPath::onResponseReceived(const inrof2025_ros_type::srv::BallPath::Response::SharedPtr& response)
{
    if (!response) return BT::NodeStatus::FAILURE;
    
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LinearPath::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "ball path service error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
}