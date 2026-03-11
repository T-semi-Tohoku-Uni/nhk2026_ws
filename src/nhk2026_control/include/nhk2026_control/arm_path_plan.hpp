#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "nhk2026_msgs/srv/arm_path_plan.hpp"

class ArmPathPlan
: public rclcpp::Node
{
public:
    ArmPathPlan();
private:
    rclcpp::Service<nhk2026_msgs::srv::ArmPathPlan>::SharedPtr arm_path_service_;
    void path_gen_callback(
        const std::shared_ptr<nhk2026_msgs::srv::ArmPathPlan::Request> request,
        std::shared_ptr<nhk2026_msgs::srv::ArmPathPlan::Response> response
    );
    std::vector<std::pair<double, double>> generator(
        std::pair<double, double> start_point,
        std::pair<double, double> goal_point
    );
};