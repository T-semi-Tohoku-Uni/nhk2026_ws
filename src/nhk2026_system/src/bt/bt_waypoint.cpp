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
    static_cast<void>(response); // response is empty, so we just ignore it
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AddWaypoint::onFailure(BT::ServiceNodeErrorCode error) {
    static_cast<void>(error); // we can log the error code here if needed
    return BT::NodeStatus::FAILURE;
}