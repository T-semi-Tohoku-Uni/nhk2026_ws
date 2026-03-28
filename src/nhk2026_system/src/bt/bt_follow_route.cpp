#include "bt/bt_follow_route.hpp"

BT::PortsList FollowRoute::providedPorts() {
    return providedBasicPorts({});
}

bool FollowRoute::setGoal(Goal & goal) {
    RCLCPP_INFO(logger(), "Sending follow goal");
    static_cast<void>(goal);
    return true;
}

BT::NodeStatus FollowRoute::onResultReceived(const WrappedResult & result) {
    if (result.result && result.result->success) {
        RCLCPP_INFO(logger(), "Follow goal succeeded");
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(logger(), "Follow goal failed");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FollowRoute::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    (void) feedback;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowRoute::onFailure(BT::ActionNodeErrorCode error) {
    RCLCPP_ERROR(logger(), "Follow goal failed with error code: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}

void FollowRoute::onHalt() {
    RCLCPP_INFO(logger(), "Halting follow goal");
}