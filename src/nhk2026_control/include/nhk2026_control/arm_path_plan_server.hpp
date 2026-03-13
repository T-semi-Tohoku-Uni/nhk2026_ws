#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "nhk2026_msgs/srv/arm_path_plan.hpp"

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <kdl_parser/kdl_parser.hpp>

class ArmPathPlan
: public rclcpp::Node
{
public:
    ArmPathPlan();
private:
    rclcpp::Service<nhk2026_msgs::srv::ArmPathPlan>::SharedPtr arm_path_service_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;

    void path_gen_callback(
        const std::shared_ptr<nhk2026_msgs::srv::ArmPathPlan::Request> request,
        std::shared_ptr<nhk2026_msgs::srv::ArmPathPlan::Response> response
    );
    std::vector<std::pair<double, double>> generator(
        std::pair<double, double> start_point,
        std::pair<double, double> goal_point
    );
    void robot_description_callback(const std_msgs::msg::String::SharedPtr rxdata);
};
