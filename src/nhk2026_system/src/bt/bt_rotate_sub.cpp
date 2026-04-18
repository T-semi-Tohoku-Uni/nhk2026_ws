#include "bt/bt_rotate_sub.hpp"

BT::PortsList RotateSub::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("target_theta"),
        BT::InputPort<double>("tolerance", 0.05, "tolerance in radians"),
    });
}

BT::NodeStatus RotateSub::onTick(const std::shared_ptr<geometry_msgs::msg::Pose>& last_msg)
{
    if (!last_msg) {
        RCLCPP_WARN(logger(), "%s: no message received yet", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    auto target_theta = getInput<double>("target_theta");
    if (!target_theta) {
        RCLCPP_ERROR(logger(), "%s: missing required input [target_theta]", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    auto tolerance = getInput<double>("tolerance").value_or(0.05);

    tf2::Quaternion q;
    tf2::fromMsg(last_msg->orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double error = std::fabs(angles::shortest_angular_distance(yaw, target_theta.value()));
    RCLCPP_INFO(logger(), "%s: current yaw=%.3f, target=%.3f, error=%.3f",
                name().c_str(), yaw, target_theta.value(), error);

    if (error < tolerance) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}
