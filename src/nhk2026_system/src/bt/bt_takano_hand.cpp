#include "bt/bt_takano_hand.hpp"

BT::PortsList TakanoHandAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>("step", false, "Whether to open the hand")
  });
}