#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "nhk2026_system/bt_move_arm.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ide_arm_bt_node");
  const auto default_xml =
    ament_index_cpp::get_package_share_directory("nhk2026_system") + "/config/ide_arm_bt.xml";
  node->declare_parameter<std::string>("bt_xml_file", default_xml);

  std::string bt_xml_file;
  node->get_parameter("bt_xml_file", bt_xml_file);

  BT::BehaviorTreeFactory factory;

  BT::RosNodeParams params;
  params.nh = node;
  params.default_port_value = "ide_arm";

  factory.registerNodeType<MoveArmAction>("MoveArm", params);

  auto tree = factory.createTreeFromFile(bt_xml_file);
  tree.tickWhileRunning();

  rclcpp::shutdown();
  return 0;
}
