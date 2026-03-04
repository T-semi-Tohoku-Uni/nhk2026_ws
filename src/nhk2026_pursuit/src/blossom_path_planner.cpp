#include "nhk2026_pursuit/blossom_path_planner.hpp"



namespace nhk2026_pursuit::blossom_path{
    BlossomPathPlanner::BlossomPathPlanner(const rclcpp::NodeOptions & options): Node("blossom_path_planner", options){
        subPose_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&BlossomPathPlanner::poseCallback, this, std::placeholders::_1)
        );
    
        rclcpp::QoS pathQoS = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliable()
                                    .transient_local();

        path_pub_ = create_publisher<nav_msgs::msg::Path>("route", pathQoS);

        srv_gen_route_ = this->create_service<inrof2025_ros_type::srv::BallPath>(
            "generate_ball_path",
            std::bind(&BlossomPathPlanner::planBlossomPath,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)
        );
                
        //デバック用
        rclcpp::QoS poseArrowQoS(rclcpp::KeepLast(10));
        pose_arrow_pub_= this->create_publisher<visualization_msgs::msg::Marker>("pose_arrow_marker", poseArrowQoS);


        this->declare_parameter<int>("num_points_", 10);
        this->declare_parameter<double>("shorten", 0.04);
        this->declare_parameter<double>("theta_offset", 0.0);
        this->get_parameter("num_points_", num_points_);
        this->get_parameter("shorten", shorten_);
        this->get_parameter("theta_offset", theta_offset_);
    };


    void BlossomPathPlanner::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg){
        pose_ = msg;
    };

    void BlossomPathPlanner::StraightPath(
        nav_msgs::msg::Path& path_msg,
        double sx, double sy, double gx, double gy
    ){        
        
        double dx = gx - sx;
        double dy = gy - sy;
        double distance = sqrt(dx*dx + dy*dy);

        if(distance < 1e-6){
            return;
        }

        double ux = dx / distance;
        double uy = dy / distance;
       
        //shorten path
        if (distance > shorten_){
            gx -= shorten_ * ux;
            gy -= shorten_ * uy;
        }

        //create path
        for (int i=0; i<=num_points_; ++i){
            double t = static_cast<double>(i) / num_points_;
            geometry_msgs::msg::PoseStamped p;
            p.header = path_msg.header;
            p.pose.position.x = sx + t * (gx - sx);
            p.pose.position.y = sy + t * (gy - sy);

            path_msg.poses.push_back(p);
        }
    };


    //グリッドの配列を入力したら、座標の配列が出力される関数
    std::vector<std::pair<double,double>> BlossomPathPlanner::grid2World(
        const std::vector<GridIndex>& grids)
    {
        std::vector<std::pair<double, double>> waypoints;

        if(!pose_){
            RCLCPP_INFO(this->get_logger(), "no pose");
            return waypoints;
        }

        //後でorigin resolutionをjsonから読み込む
        //仮にblueフィールドの値を入れる
        double origin_x   = -1.825;
        double origin_y   = 2.6;
        double resolution = 1.2;

        waypoints.push_back({pose_->x, pose_->y});

        for (size_t i = 0; i < grids.size(); ++i){
            const GridIndex& grid = grids[i];
            double world_x = origin_x - grid.v * resolution;
            double world_y = origin_y + grid.u * resolution;
            
            waypoints.push_back({world_x, world_y});
        }

        return waypoints;

    };



    void BlossomPathPlanner::planBlossomPath(
        const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Request> request,
        const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Response> response
    ){
        if (!pose_){
            RCLCPP_INFO(this->get_logger(), "no pose");
            return;
        }

        //generate path
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        
        //後で強化学習の関数からグリッドの配列をもらう
        //仮にグリッドの配列を入れる
        std::vector<GridIndex> grids = {
            {0,0},
            {1,0},
            {2,0},
            {2,1},
            {3,1},
            {4,1},
            {4,0},
            {5,0},

        };
        
        std::vector<std::pair<double,double>> waypoints = grid2World(grids);
        
        for(size_t i=0; i<waypoints.size()-1; ++i){
            StraightPath(path_msg, 
                        waypoints[i].first, waypoints[i].second, 
                        waypoints[i+1].first, waypoints[i+1].second);
        }
        
        path_pub_->publish(path_msg);




        //visualization
        visualization_msgs::msg::Marker arrow;
        geometry_msgs::msg::Pose goal_robot_pose;

        // pub waypoint pose
        arrow.header.frame_id = "map";
        arrow.ns = "goal_point_arrow";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        if (!path_msg.poses.empty()) {
            arrow.pose = path_msg.poses.back().pose;
        }
        arrow.scale.x = 0.08;
        arrow.scale.y = 0.04;
        arrow.scale.z = 0.04;
        arrow.color.r = 0.0f;
        arrow.color.g = 0.0f;
        arrow.color.b = 1.0f;
        arrow.color.a = 1.0f;
        pose_arrow_pub_ -> publish(arrow);
    };
}


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nhk2026_pursuit::blossom_path::BlossomPathPlanner>());
    rclcpp::shutdown();
    return 0;
}