#include "bt/bt_sub_aruco.hpp"

BT::PortsList ArucoPoseSub::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<int>("id", "ID of the detected ArUco marker")
    });
}

BT::NodeStatus ArucoPoseSub::onTick(const std::shared_ptr<nhk2026_msgs::msg::ArucoPose>& msg)
{
    if (!armed_)
    {
        this->armed_ = true;
        return BT::NodeStatus::FAILURE;
    }

    if (msg)
    {
        this->armed_ = false;
        auto id = this->getInput<int>("id").value_or(0);
        if (msg->id == id) {
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}