#include "nhk2026_pursuit/blossom_path_planner.hpp"


namespace nhk2026_pursuit::blossom_path{
    BlossomPathPlanner::BlossomPathPlanner(const rclcpp::NodeOptions & options): Node("blossom_path_planner", options){
        subPose_ = create_subscription<geometry_msgs::msg::Pose>(
            "pose", 10, std::bind(&BlossomPathPlanner::poseCallback, this, std::placeholders::_1)
        );
    
        rclcpp::QoS pathQoS = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliable()
                                    .transient_local();

        path_pub_ = create_publisher<nhk2026_msgs::msg::PathWithBox>("route", pathQoS);
        path_rviz_pub_ = create_publisher<nav_msgs::msg::Path>("rviz_route", pathQoS);

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

            PoseWithBox pose;
            pose.state.header.frame_id = "map";
            pose.state.header.stamp = this->now();
            pose.state.pose.position.x = grid["x"];
            pose.state.pose.position.y = grid["y"];
            pose.state.pose.position.z = grid["z"];
            pose.flag = false;
            grid_map_[u][v] = pose;
        }
    };


    PathWithBox BlossomPathPlanner::StraightPath(
        PoseWithBox start,
        PoseWithBox goal
    ){        
        PathWithBox path_msg;
        path_msg.path.header = start.state.header;
        
        double dx = goal.state.pose.position.x - start.state.pose.position.x;
        double dy = goal.state.pose.position.y - start.state.pose.position.y;
        double dz = goal.state.pose.position.z - start.state.pose.position.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        if(distance < 1e-6){
            return path_msg;
        }

        double ux = dx / distance;
        double uy = dy / distance;
        double uz = dz / distance;
       
        //shorten path
        if (distance > shorten_){
            goal.state.pose.position.x -= shorten_ * ux;
            goal.state.pose.position.y -= shorten_ * uy;
            goal.state.pose.position.z -= shorten_ * uz;
        }

        double yaw = getYaw(goal.state.pose.orientation);
        bool is_box = goal.flag;

        //create path
        for (int i=0; i<=num_points_; ++i){
            double t = static_cast<double>(i) / num_points_;

            geometry_msgs::msg::PoseStamped p;
            p.header = path_msg.path.header;

            p.pose.position.x = start.state.pose.position.x + t * (goal.state.pose.position.x - start.state.pose.position.x);
            p.pose.position.y = start.state.pose.position.y + t * (goal.state.pose.position.y - start.state.pose.position.y);
            p.pose.position.z = start.state.pose.position.z + t * (goal.state.pose.position.z - start.state.pose.position.z);

            //add theta information in the path
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            p.pose.orientation = tf2::toMsg(q);

            path_msg.path.poses.push_back(p);

            //add box flag information in the path
            path_msg.flags.push_back(is_box);
        }

        return path_msg;
    };


    std::vector<PoseWithBox> BlossomPathPlanner::grid2World(
        const std::vector<GridIndex>& grids)
    {
        std::vector<PoseWithBox> waypoints;

        if(!pose_){
            RCLCPP_WARN(this->get_logger(), "no pose");
            return waypoints;
        }

        PoseWithBox init_pose;
        init_pose.state.header.frame_id = "map";
        init_pose.state.header.stamp = this->now();
        init_pose.state.pose.position.x = pose_->position.x;
        init_pose.state.pose.position.y = pose_->position.y;
        init_pose.state.pose.position.z = pose_->position.z;
        init_pose.flag = false;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, getYaw(pose_->orientation));
        init_pose.state.pose.orientation = tf2::toMsg(q);

        waypoints.push_back(init_pose);

        double prev_z = init_pose.state.pose.position.z;

        for (int i = 0; i < static_cast<int>(grids.size()); ++i) {

            const GridIndex& grid = grids[i];
            PoseWithBox world_pose = grid_map_[grid.u][grid.v];
            world_pose.flag = grid.flag;


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
            double z_diff = world_pose.state.pose.position.z - prev_z;
            if (z_diff < 0.0){
                yaw += M_PI;
            }
            while (yaw > M_PI)  yaw -= 2.0 * M_PI;
            while (yaw < -M_PI) yaw += 2.0 * M_PI;
            
            
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);


            //ここで中間地点からのoffsetを加味して経路生成 mid_start -> mid_end -> world_pose
            double mid_x = (waypoints.back().state.pose.position.x + world_pose.state.pose.position.x) / 2.0;
            double mid_y = (waypoints.back().state.pose.position.y + world_pose.state.pose.position.y) / 2.0;
            
            double dx = world_pose.state.pose.position.x - waypoints.back().state.pose.position.x;
            double dy = world_pose.state.pose.position.y - waypoints.back().state.pose.position.y;
            double distance_start = std::hypot(dx, dy);
            double ux = (distance_start > 1e-6) ? dx / distance_start : 0.0;
            double uy = (distance_start > 1e-6) ? dy / distance_start : 0.0;

            PoseWithBox mid_start, mid_end;
            mid_start.state.header.frame_id = "map";
            mid_start.state.header.stamp = this->now();
            mid_start.state.pose.position.x = mid_x - start_shorten_ * ux;
            mid_start.state.pose.position.y = mid_y - start_shorten_ * uy;
            mid_start.state.pose.position.z = waypoints.back().state.pose.position.z;
            mid_start.state.pose.orientation = q_msg;
            mid_start.flag = grid.flag;

            mid_start.state.header.frame_id = "map";
            mid_start.state.header.stamp = this->now();
            mid_end.state.pose.position.x = mid_x + end_shorten_ * ux;
            mid_end.state.pose.position.y = mid_y + end_shorten_ * uy;
            mid_end.state.pose.position.z = world_pose.state.pose.position.z;
            mid_end.state.pose.orientation = q_msg;
            mid_end.flag = grid.flag;

            world_pose.state.pose.orientation = q_msg;
            world_pose.state.header.frame_id = "map";
            world_pose.state.header.stamp = this->now();

            waypoints.push_back(mid_start);
            waypoints.push_back(mid_end);
            waypoints.push_back(world_pose);

            prev_z = world_pose.state.pose.position.z;
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
        PathWithBox path_msg;
        path_msg.path.header.frame_id = "map";
        path_msg.path.header.stamp = this->now();

        
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

            {0,0},
            {1,0},
            {2,0},
            {3,0},
            {4,0},
            {5,0},
        };
        
        std::vector<PoseWithBox> waypoints = grid2World(grids);
        
        
        for(size_t i=0; i<waypoints.size()-1; ++i){

            PathWithBox segment = StraightPath(waypoints[i], waypoints[i+1]);

            for(size_t j = 0; j < segment.path.poses.size(); ++j){
                path_msg.path.poses.push_back(segment.path.poses[j]);
                path_msg.flags.push_back(segment.flags[j]);
            }

        }
        
        path_pub_->publish(path_msg);
        path_rviz_pub_->publish(path_msg.path);

        //visualization
        visualization_msgs::msg::Marker arrow;
        geometry_msgs::msg::Pose goal_robot_pose;

        // pub waypoint pose
        arrow.header.frame_id = "map";
        arrow.ns = "goal_point_arrow";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        if (!path_msg.path.poses.empty()) {
            arrow.pose = path_msg.path.poses.back().pose;
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