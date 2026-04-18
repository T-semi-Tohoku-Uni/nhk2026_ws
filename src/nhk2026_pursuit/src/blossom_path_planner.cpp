#include "nhk2026_pursuit/blossom_path_planner.hpp"



namespace nhk2026_pursuit::blossom_path{
    BlossomPathPlanner::BlossomPathPlanner(const rclcpp::NodeOptions & options): Node("blossom_path_planner", options){
        subPose_ = create_subscription<geometry_msgs::msg::Pose>(
            "pose", 10, std::bind(&BlossomPathPlanner::poseCallback, this, std::placeholders::_1)
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

        this->declare_parameter<int>("num_points_", 2);
        this->declare_parameter<double>("shorten_", 0.04);
        this->declare_parameter<double>("theta_offset_", 0.0);
        this->declare_parameter<double>("start_shorten_", 0.15);
        this->declare_parameter<double>("end_shorten_", 0.15);
        this->get_parameter("num_points_", num_points_);
        this->get_parameter("shorten_", shorten_);
        this->get_parameter("theta_offset_", theta_offset_);
        this->get_parameter("start_shorten_", start_shorten_);
        this->get_parameter("end_shorten_", end_shorten_);

         RCLCPP_INFO(this->get_logger(), "Initialize blossom path");
    };


    void BlossomPathPlanner::poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        pose_ = msg;
    };

    double getYaw(const geometry_msgs::msg::Quaternion& q_msg) {
            tf2::Quaternion q;
            tf2::fromMsg(q_msg, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return yaw;
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
        double gx, double gy, double gz,
        double yaw
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

            //add theta information in the path
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            p.pose.orientation = tf2::toMsg(q);

            path_msg.poses.push_back(p);
        }
    };


    std::vector<geometry_msgs::msg::Pose> BlossomPathPlanner::grid2World(
        const std::vector<GridIndex>& grids)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        if(!pose_){
            RCLCPP_WARN(this->get_logger(), "no pose");
            return waypoints;
        }

        geometry_msgs::msg::Pose init_pose;
        init_pose.position.x = pose_->position.x;
        init_pose.position.y = pose_->position.y;
        init_pose.position.z = pose_->position.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, getYaw(pose_->orientation));
        init_pose.orientation = tf2::toMsg(q);

        waypoints.push_back(init_pose);

        double prev_z = init_pose.position.z;

        for (int i = 0; i < static_cast<int>(grids.size()); ++i) {

            const GridIndex& grid = grids[i];
            geometry_msgs::msg::Pose world_pose = grid_map_[grid.u][grid.v];


            //ここで角度計算
            double yaw = 0.0;
            if (i == 0){
                yaw = getYaw(pose_->orientation);
            } else {
                int du, dv;
                du = grid.u - grids[i-1].u;
                dv = grid.v - grids[i-1].v;
                yaw = atan2(static_cast<double>(dv), static_cast<double>(du));
            }
            
            //角度再構築
            double z_diff = world_pose.position.z - prev_z;
            if (z_diff < 0.0){
                yaw += M_PI;
            }
            while (yaw > M_PI)  yaw -= 2.0 * M_PI;
            while (yaw < -M_PI) yaw += 2.0 * M_PI;
            
            
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);


            //ここで中間地点からのoffsetを加味して経路生成 mid_start -> mid_end -> world_pose
            double mid_x = (waypoints.back().position.x + world_pose.position.x) / 2.0;
            double mid_y = (waypoints.back().position.y + world_pose.position.y) / 2.0;
            
            double dx = world_pose.position.x - waypoints.back().position.x;
            double dy = world_pose.position.y - waypoints.back().position.y;
            double distance_start = std::hypot(dx, dy);
            double ux = (distance_start > 1e-6) ? dx / distance_start : 0.0;
            double uy = (distance_start > 1e-6) ? dy / distance_start : 0.0;

            geometry_msgs::msg::Pose mid_start, mid_end;
            mid_start.position.x = mid_x - start_shorten_ * ux;
            mid_start.position.y = mid_y - start_shorten_ * uy;
            mid_start.position.z = waypoints.back().position.z;
            mid_start.orientation = q_msg;

            mid_end.position.x = mid_x + end_shorten_ * ux;
            mid_end.position.y = mid_y + end_shorten_ * uy;
            mid_end.position.z = world_pose.position.z;
            mid_end.orientation = q_msg;

            world_pose.orientation = q_msg;

            waypoints.push_back(mid_start);
            waypoints.push_back(mid_end);
            waypoints.push_back(world_pose);

            prev_z = world_pose.position.z;
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
            // {0,1},
            // {0,0},
            // {1,0},
            // {1,1},
            // {2,1},
            // {2,2},
            // {3,2},
            // {4,2},
            // {5,2},

            {0,1},
            {1,1},
            {2,1},
            {3,1},
            {4,1},
            {5,1},
        };
        
        std::vector<geometry_msgs::msg::Pose> waypoints = grid2World(grids);
        
        
        for(size_t i=0; i<waypoints.size()-1; ++i){

            tf2::Quaternion q;
            tf2::fromMsg(waypoints[i+1].orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);


            StraightPath(path_msg, 
                        waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z,
                        waypoints[i+1].position.x, waypoints[i+1].position.y, waypoints[i+1].position.z,
                        yaw
                    );
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