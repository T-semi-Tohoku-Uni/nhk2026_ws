#include "nhk2026_control/ide_arm_teaching.hpp"

IdeArmTeaching::IdeArmTeaching()
: rclcpp::Node("ide_arm_teaching")
{
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

geometry_msgs::msg::TransformStamped IdeArmTeaching::listen_transform()
{
    
}