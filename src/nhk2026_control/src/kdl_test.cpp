#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "std_msgs/msg/string.hpp"

void callback(const std_msgs::msg::String::SharedPtr rxdata)
{
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(rxdata->data, tree)) {
        std::cerr << "Failed to parse URDF into KDL tree." << std::endl;
        return;
    }

    const std::string base_link = "arm_base";
    const std::string end_link = "tcp_link";

    KDL::Chain chain;
    if (!tree.getChain(base_link, end_link, chain)) {
        std::cerr << "Failed to extract KDL chain from "
                  << base_link << " to " << end_link << std::endl;
        return;
    }

    std::cout << "Chain joints: " << chain.getNrOfJoints() << std::endl;
    std::cout << "Chain segments: " << chain.getNrOfSegments() << std::endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node("kdl");
    auto subscriber = node.create_subscription<std_msgs::msg::String>(
        "robot_description",
        rclcpp::SystemDefaultsQoS(),
        std::bind(callback, std::placeholders::_1)
    );
    rclcpp::spin(node.get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
