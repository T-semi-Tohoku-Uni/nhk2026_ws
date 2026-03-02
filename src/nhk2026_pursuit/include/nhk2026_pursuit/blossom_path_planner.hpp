#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <inrof2025_ros_type/srv/ball_path.hpp>
#include <visualization_msgs/msg/marker.hpp>


namespace nhk2026_pursuit::blossom_path{
    class BlossomPathPlanner : public rclcpp::Node{
        public:
            BlossomPathPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        private:
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subPose_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_arrow_pub_;
            void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
            geometry_msgs::msg::Pose2D::SharedPtr pose_;
            void planBlossomPath(
                const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Response> response
            );
    };
}