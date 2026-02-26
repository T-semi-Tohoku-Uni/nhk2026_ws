#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class JointStatePublisherIde
: public rclcpp::Node
{
public:
    JointStatePublisherIde();
private:
    
};