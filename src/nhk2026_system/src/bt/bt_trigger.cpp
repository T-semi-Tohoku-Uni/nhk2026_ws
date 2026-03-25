#include "bt/bt_trigger.hpp"

BT::NodeStatus TestTrigger::onTick(const std::shared_ptr<std_msgs::msg::Bool>& msg)
{
    if (msg->data) return BT::NodeStatus::SUCCESS;
    
    return BT::NodeStatus::FAILURE;
}