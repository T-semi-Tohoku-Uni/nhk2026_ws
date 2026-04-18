#include "bt/bt_waypoint.hpp"

BT::PortsList AddWaypoint::providedPorts() {
    return providedBasicPorts({
        BT::InputPort<double> ("x"),
        BT::InputPort<double> ("y"),
    });
}

bool AddWaypoint::setRequest(inrof2025_ros_type::srv::Waypoint::Request::SharedPtr& request) {
    BT::Expected<double> tmp_x = getInput<double>("x");
    BT::Expected<double> tmp_y = getInput<double>("y");

    if (!tmp_x) throw BT::RuntimeError("missing required input x: ", tmp_x.error());
    if (!tmp_y) throw BT::RuntimeError("missing required input y: ", tmp_y.error());

    request->x = tmp_x.value();
    request->y = tmp_y.value();

    return true;
}

BT::NodeStatus AddWaypoint::onResponseReceived(const inrof2025_ros_type::srv::Waypoint::Response::SharedPtr& response) {
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AddWaypoint::onFailure(BT::ServiceNodeErrorCode error) {
    return BT::NodeStatus::FAILURE;
}