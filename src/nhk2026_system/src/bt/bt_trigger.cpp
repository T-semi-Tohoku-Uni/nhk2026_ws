#include "bt/bt_trigger.hpp"

BT::NodeStatus TriggerTopic::onTick(const std::shared_ptr<std_msgs::msg::Empty>& msg)
{
    if (!armed_)
    {
        this->armed_ = true;
        return BT::NodeStatus::FAILURE;
    }

    if (msg) {
        armed_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}