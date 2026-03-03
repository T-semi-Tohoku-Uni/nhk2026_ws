#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class JointStatePublisherDansa
: public rclcpp::Node
{
public:
    JointStatePublisherDansa();
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    void motor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr rxdata);
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_feedback_subscriber_;
};