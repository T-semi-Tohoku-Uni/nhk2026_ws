#include "bt/bt_vel_pub.hpp"

BT::PortsList BtVelPub::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("vx", 0.0, "linear velocity in x direction"),
        BT::InputPort<double>("vy", 0.0, "linear velocity in y direction"),
        BT::InputPort<double>("omega", 0.0, "angular velocity in z direction")
    });
}

bool BtVelPub::setMessage(geometry_msgs::msg::Twist & msg)
{
    auto vx = getInput<double>("vx");
    auto vy = getInput<double>("vy");
    auto omega = getInput<double>("omega");

    if (!vx || !vy || !omega) {
        return false;
    }

    msg.linear.x = vx.value();
    msg.linear.y = vy.value();
    msg.angular.z = omega.value();

    return true;
}