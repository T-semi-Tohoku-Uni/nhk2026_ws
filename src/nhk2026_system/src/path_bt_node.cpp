#include <memory>
#include <string>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_ros2/ros_node_params.hpp"
#include "bt/bt_follow_route.hpp"
#include "bt/bt_generate_route.hpp"
#include "bt/bt_linear_path.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("path_bt_node");
    const auto default_xml =
        ament_index_cpp::get_package_share_directory("yasarobo2025_26") + "/config/blue_bt.xml";
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

    BT::RosNodeParams generate_route_params;
    generate_route_params.nh = node;
    generate_route_params.default_port_value = "generate_route";

    BT::RosNodeParams linear_path_params;
    linear_path_params.nh = node;
    linear_path_params.default_port_value = "generate_ball_path";

    factory.registerNodeType<FollowRoute>("FollowRoute", follow_route_params);
    factory.registerNodeType<GenerateRoute>("GenerateRoute", generate_route_params);
    factory.registerNodeType<LinearPath>("LinearPath", linear_path_params);

    auto tree = factory.createTreeFromFile(bt_xml_file);
    tree.tickWhileRunning();

    rclcpp::shutdown();
    return 0;
}