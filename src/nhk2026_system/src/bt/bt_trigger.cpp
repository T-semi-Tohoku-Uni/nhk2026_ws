#include "bt/bt_trigger.hpp"

BT::NodeStatus TestTrigger::onTick(const std::shared_ptr<std_msgs::msg::Bool>& msg)
{
    if (!armed_)
    {
        this->armed_ = true;
        return BT::NodeStatus::FAILURE;
    }

    if (msg && msg->data) {
        armed_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}