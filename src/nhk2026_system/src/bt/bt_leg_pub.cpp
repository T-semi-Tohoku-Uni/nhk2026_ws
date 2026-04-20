#include "bt/bt_leg_pub.hpp"

BT::PortsList BtLegPub::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("angles", "Leg joint angles")
  });
}

bool BtLegPub::setMessage(std_msgs::msg::Float32MultiArray & msg)
{
  auto angles = getInput<double>("angles");
  if (!angles) return false;
  msg.data = {1.0, 1.0, 0, 0, static_cast<float>(angles.value())};
  return true;
}