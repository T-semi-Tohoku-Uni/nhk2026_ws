#include <blossom_path_planner.hpp>



namespace nhk2026_pursuit::blossom_path{
    BlossomPathPlanner::BlossomPathPlanner(const rclcpp::NodeOptions & options): Node("blossom_path_planner", options){
        subPose_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&BlossomPathPlanner::poseCallback, this, std::placeholders::_1)
        );
    
        rclcpp::QoS pathQoS = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliable()
                                    .transient_local();

        path_pub_ = create_publisher<nav_msgs::msg::Path>("route", pathQoS);

        //ここをどうすればいいかわからない
        // srv_blossom_route_ = this->create_service<inrof2025_ros_type::srv::>

        
        //デバック用
        rclcpp::QoS poseArrowQos(rclcpp::KeepLast(10));
        pose_arrow_pub_= this->create_publisher<visualization_msgs::msg::Marker>("pose_arrow_marker", poseArrowQos);


    };


    void BlossomPathPlanner::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg){
        pose_ = msg;
    };


    void BlossomPathPlanner::planBlossomPath(
        const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Request> request,
        const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Response> response
    ){
        if (!pose_){
            RCLCPP_INFO(this->get_logger(), "no pose");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generating ball path to (%.2f, %.2f)", request->x, request->y);

        //generate path




    };
    

}

