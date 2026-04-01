#include "nhk2026_control/ide_arm_teaching.hpp"

IdeArmTeaching::IdeArmTeaching()
: rclcpp::Node("ide_arm_teaching")
{
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

geometry_msgs::msg::TransformStamped IdeArmTeaching::listen_transform()
{
    geometry_msgs::msg::TransformStamped t;

    if (!tf_buffer_->canTransform(
            "tcp_link", "arm_base", tf2::TimePointZero, tf2::durationFromSec(3.0))) {
        RCLCPP_WARN(this->get_logger(),
                    "Timed out waiting for transform arm_base -> tcp_link");
        return t;
    }

    try
    {
        t = tf_buffer_->lookupTransform(
            "tcp_link", "arm_base",
            tf2::TimePointZero
        );
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform base_link to dynamic_frame: %s",
            ex.what());
        return t;
    }
    
    
    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
        this->get_logger(), "base_link->dynamic_frame: %f %f %f",
        t.transform.translation.y, t.transform.translation.z, roll);
    return t;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IdeArmTeaching>();

    node->listen_transform();

    rclcpp::shutdown();
    return 0;
}