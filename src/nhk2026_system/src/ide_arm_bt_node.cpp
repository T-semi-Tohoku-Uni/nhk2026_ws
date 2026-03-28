#include <memory>
#include <string>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "bt/bt_move_arm.hpp"
#include "bt/bt_vacuum.hpp"
#include "bt/bt_trigger.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ide_arm_bt_node");
  const auto default_xml =
    ament_index_cpp::get_package_share_directory("nhk2026_system") + "/config/ide_arm_bt.xml";
  node->declare_parameter<std::string>("bt_xml_file", default_xml);
  node->declare_parameter<int>("wait_for_server_timeout_ms", 5000);

  std::string bt_xml_file;
  int wait_for_server_timeout_ms;
  node->get_parameter("bt_xml_file", bt_xml_file);
  node->get_parameter("wait_for_server_timeout_ms", wait_for_server_timeout_ms);

  BT::BehaviorTreeFactory factory;

  BT::RosNodeParams action_params;
  action_params.nh = node;
  action_params.default_port_value = "ide_arm";
  action_params.wait_for_server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
  
  BT::RosNodeParams service_params;
  service_params.nh = node;
  service_params.default_port_value = "vacuum";

  BT::RosNodeParams trigger_params;
  trigger_params.nh = node;
  trigger_params.default_port_value = "trigger";

  factory.registerNodeType<MoveArmAction>("move_arm", action_params);
  factory.registerNodeType<ServiceVacuum>("service_vacuum", service_params);
  factory.registerNodeType<TriggerTopic>("trigger_topic", trigger_params);

  auto tree = factory.createTreeFromFile(bt_xml_file);
  tree.tickWhileRunning();

  rclcpp::shutdown();
  return 0;
}
