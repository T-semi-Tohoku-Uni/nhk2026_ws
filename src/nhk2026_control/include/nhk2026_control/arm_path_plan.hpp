#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "nhk2026_msgs/srv/arm_path_plan.hpp"

class ArmPathPlan
: public rclcpp::Node
{
public:
    ArmPathPlan();

};