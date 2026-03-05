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

        std::string pkg_path =
            ament_index_cpp::get_package_share_directory("nhk2026_core");

        std::string json_path =
        pkg_path + "/config/grid_map_blue.json";

        loadJsonFile(json_path);

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


    void BlossomPathPlanner::loadJsonFile(const std::string& json_file_path){
        std::ifstream file(json_file_path);
        nlohmann::json json_data;
        file >> json_data;

        grid_map_.resize(HEIGHT_);
        for (size_t i = 0; i < HEIGHT_; ++i){
            grid_map_[i].resize(WIDTH_);
        }

        for (const nlohmann::json& grid : json_data["grids"]){
            int u = grid["u"];
            int v = grid["v"];

            geometry_msgs::msg::Pose pose;

            pose.position.x = grid["x"];
            pose.position.y = grid["y"];
            pose.position.z = grid["z"];

            grid_map_[u][v] = pose;
        }
    };


    void BlossomPathPlanner::StraightPath(
        nav_msgs::msg::Path& path_msg,
        double sx, double sy, double sz,
        double gx, double gy, double gz
    ){        
        
        double dx = gx - sx;
        double dy = gy - sy;
        double dz = gz - sz;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        if(distance < 1e-6){
            return;
        }

        double ux = dx / distance;
        double uy = dy / distance;
        double uz = dz / distance;
       
        //shorten path
        if (distance > shorten_){
            gx -= shorten_ * ux;
            gy -= shorten_ * uy;
            gz -= shorten_ * uz;
        }

        //create path
        for (int i=0; i<=num_points_; ++i){
            double t = static_cast<double>(i) / num_points_;
            geometry_msgs::msg::PoseStamped p;
            p.header = path_msg.header;

            p.pose.position.x = sx + t * (gx - sx);
            p.pose.position.y = sy + t * (gy - sy);
            p.pose.position.z = sz + t * (gz - sz);

            path_msg.poses.push_back(p);
        }
    };


    std::vector<geometry_msgs::msg::Pose> BlossomPathPlanner::grid2World(
        const std::vector<GridIndex>& grids)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        if(!pose_){
            RCLCPP_INFO(this->get_logger(), "no pose");
            return waypoints;
        }

        geometry_msgs::msg::Pose init_pose;
        init_pose.position.x = pose_->x;
        init_pose.position.y = pose_->y;
        init_pose.position.z = 0.0;

        waypoints.push_back(init_pose);

        for (const GridIndex& grid : grids){
            geometry_msgs::msg::Pose world_pose = grid_map_[grid.u][grid.v];
            geometry_msgs::msg::Pose middle_pose1, middle_pose2;

            middle_pose1.position.x = (waypoints.back().position.x + world_pose.position.x) / 2.0;
            middle_pose1.position.y = (waypoints.back().position.y + world_pose.position.y) / 2.0;
            middle_pose1.position.z = waypoints.back().position.z;

            middle_pose2.position.x = (waypoints.back().position.x + world_pose.position.x) / 2.0;
            middle_pose2.position.y = (waypoints.back().position.y + world_pose.position.y) / 2.0;
            middle_pose2.position.z = world_pose.position.z;

            waypoints.push_back(middle_pose1);
            waypoints.push_back(middle_pose2);
            waypoints.push_back(world_pose);
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
        
        std::vector<geometry_msgs::msg::Pose> waypoints = grid2World(grids);
        
        
        for(size_t i=0; i<waypoints.size()-1; ++i){
            StraightPath(path_msg, 
                        waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z,
                        waypoints[i+1].position.x, waypoints[i+1].position.y, waypoints[i+1].position.z);
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