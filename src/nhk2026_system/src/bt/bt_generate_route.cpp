#include "bt/bt_generate_route.hpp"

BT::PortsList GenerateRoute::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double> ("x"),
        BT::InputPort<double> ("y"),
        BT::InputPort<double> ("theta")
    });
}

bool GenerateRoute::setRequest(inrof2025_ros_type::srv::GenRoute::Request::SharedPtr& request)
{
    BT::Expected<double> tmp_x = getInput<double>("x");
    BT::Expected<double> tmp_y = getInput<double>("y");
    BT::Expected<double> tmp_theta = getInput<double>("theta");

    if (!tmp_x) {
        throw BT::RuntimeError("missing required input x: ", tmp_x.error() );
    }
    if (!tmp_y) {
        throw BT::RuntimeError("missing required input y: ", tmp_y.error() );
    }
    if (!tmp_theta) {
        throw BT::RuntimeError("missing required input theta: ", tmp_theta.error() );
    }

    double x = tmp_x.value();
    double y = tmp_y.value();
    double theta = tmp_theta.value();

    request->x = x;
    request->y = y;
    request->theta = theta;
    return true;
}

BT::NodeStatus GenerateRoute::onResponseReceived(const inrof2025_ros_type::srv::GenRoute::Response::SharedPtr& response)
{
    if (!response) return BT::NodeStatus::FAILURE;
    
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GenerateRoute::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "generate route service error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
}