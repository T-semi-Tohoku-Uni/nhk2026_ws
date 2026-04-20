#include "bt/bt_sub_aruco.hpp"

BT::PortsList ArucoPoseSub::providedPorts()
{
    return providedBasicPorts({
        BT::OutputPort<int>("id", "ID of the detected ArUco marker")
    });
}

BT::NodeStatus ArucoPoseSub::onTick(const std::shared_ptr<nhk2026_msgs::msg::ArucoPose>& msg)
{
    if (msg) {
        setOutput("id", msg->id);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}