#include "bt/bt_vacuum.hpp"

BT::PortsList PublisherVacuum::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<bool>("value")
    });
}

bool PublisherVacuum::setMessage(std_msgs::msg::Bool& msg)
{
    auto value = getInput<bool>("value");
    if (!value) return false;

    msg.data = value.value();
    return true;
}