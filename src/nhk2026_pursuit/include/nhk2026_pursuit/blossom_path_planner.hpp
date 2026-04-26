#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <inrof2025_ros_type/srv/ball_path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nhk2026_msgs/msg/path_with_box.hpp>

using PathWithBox = nhk2026_msgs::msg::PathWithBox;

namespace nhk2026_pursuit::blossom_path{
    struct GridIndex {
                int u;
                int v;
                bool flag;
    };

    struct PoseWithBox{
        geometry_msgs::msg::PoseStamped state;
        bool flag;
    };
    
    class BlossomPathPlanner : public rclcpp::Node{
        public:
            BlossomPathPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            

        private:
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subPose_;
            rclcpp::Publisher<nhk2026_msgs::msg::PathWithBox>::SharedPtr path_pub_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_rviz_pub_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_arrow_pub_;
            void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
            geometry_msgs::msg::Pose::SharedPtr pose_;
            void loadJsonFile(const std::string& json_file_path);
            std::vector<std::vector<PoseWithBox>> grid_map_;
            PathWithBox StraightPath(
                PoseWithBox start,
                PoseWithBox goal
            );
            std::vector<PoseWithBox> grid2World(const std::vector<GridIndex>& grids);
            void planBlossomPath(
                const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Response> response
            );
            rclcpp::Service<inrof2025_ros_type::srv::BallPath>::SharedPtr srv_gen_route_;
            int num_points_;
            double shorten_;
            double theta_offset_;
            const int HEIGHT_ = 6;
            const int WIDTH_ = 3;
            double start_shorten_;
            double end_shorten_;
    };
}