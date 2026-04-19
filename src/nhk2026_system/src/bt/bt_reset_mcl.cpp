#include "bt/bt_reset_mcl.hpp"

BT::PortsList ResetMcl::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("x", 0.0, "initial x position"),
        BT::InputPort<double>("y", 0.0, "initial y position"),
        BT::InputPort<double>("z", 0.0, "initial z position"),
        BT::InputPort<double>("theta", 0.0, "initial orientation (radians)")
    });
}

bool ResetMcl::setMessage(geometry_msgs::msg::Pose & msg)
{
    auto x = getInput<double>("x");
    auto y = getInput<double>("y");
    auto y = getInput<double>("z");
    auto theta = getInput<double>("theta");

    if (!x) return false;
    if (!y) return false;
    if (!z) return false;
    if (!theta) return false;

    msg.position.x = x.value();
    msg.position.y = y.value();
    msg.position.z = z.value();
    tf2::Quaternion q;
    q.setRPY(0, 0, theta.value());
    q.normalize();
    msg.orientation = tf2::toMsg(q);
    return true;
}