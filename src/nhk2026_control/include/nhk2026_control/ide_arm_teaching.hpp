#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

class IdeArmTeaching
: public rclcpp::Node
{
public:
    IdeArmTeaching();

private:
    geometry_msgs::msg::TransformStamped listen_transform();

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};