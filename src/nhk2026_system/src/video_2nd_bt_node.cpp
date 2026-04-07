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
#include "bt/bt_follow_route.hpp"
#include "bt/bt_generate_route.hpp"
#include "bt/bt_linear_path.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("video_2nd_bt_node");
    const std::string default_xml =
        ament_index_cpp::get_package_share_directory("nhk2026_system") + "/config/video_2nd_bt.xml";
    node->declare_parameter<std::string>("bt_xml_file", default_xml);
    node->declare_parameter<int>("wait_for_server_timeout_ms", 5000);

    std::string bt_xml_file;
    int wait_for_server_timeout_ms;
    node->get_parameter("bt_xml_file", bt_xml_file);
    node->get_parameter("wait_for_server_timeout_ms", wait_for_server_timeout_ms);

    BT::BehaviorTreeFactory factory;
    
    BT::RosNodeParams follow_route_params;
    follow_route_params.nh = node;
    follow_route_params.default_port_value = "follow";
    follow_route_params.server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    follow_route_params.wait_for_server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);

    BT::RosNodeParams generate_route_params;
    generate_route_params.nh = node;
    generate_route_params.default_port_value = "generate_route";
    generate_route_params.server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    generate_route_params.wait_for_server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);

    BT::RosNodeParams linear_path_params;
    linear_path_params.nh = node;
    linear_path_params.default_port_value = "generate_ball_path";
    linear_path_params.server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    linear_path_params.wait_for_server_timeout = std::chrono::milliseconds(wait_for_server_timeout_ms);
    
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

    factory.registerNodeType<FollowRoute>("follow_route", follow_route_params);
    factory.registerNodeType<GenerateRoute>("generate_route", generate_route_params);
    factory.registerNodeType<LinearPath>("linear_path", linear_path_params);
    factory.registerNodeType<MoveArmAction>("move_arm", action_params);
    factory.registerNodeType<ServiceVacuum>("service_vacuum", service_params);
    factory.registerNodeType<TriggerTopic>("trigger_topic", trigger_params);

    BT::Tree tree = factory.createTreeFromFile(bt_xml_file);
    tree.tickWhileRunning();

    rclcpp::shutdown();
    return 0;
}