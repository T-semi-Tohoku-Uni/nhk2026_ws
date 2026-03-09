#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>
#include <vector>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/msg/marker.hpp"
#include <functional>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <nhk2026_msgs/srv/reset_pose.hpp>

using namespace std::chrono_literals; 

typedef struct MotorVel {
        float v1;
        float v2;
        float v3;
    } MotorVel;



class PIDController {
    public:
        PIDController() = default;
        PIDController(double Kp, double Ki, double Kd, double dt, std::function<double(double)> normalize_func = nullptr)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0), dt_(dt), normalize_func_(normalize_func) {}

        double compute(double setpoint, double measured_value) {
            double error = setpoint - measured_value;
            if (normalize_func_) {
                error = normalize_func_(error);
            }
            integral_ += error * dt_;
            double derivative = (error - prev_error_) / dt_;
            prev_error_ = error;
            return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        }

    private:
        double Kp_;
        double Ki_;
        double Kd_;
        double dt_;
        double prev_error_;
        double integral_;
        std::function<double(double)> normalize_func_;
};


class FollowNode: public rclcpp::Node {
    public:
        explicit FollowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("follow_node", options) {
            double Kp_tan, Ki_tan, Kd_tan;
            double Kp_norm, Ki_norm, Kd_norm;
            double Kp_theta, Ki_theta, Kd_theta;
            double dt = 0.1; 
            this->declare_parameter<double>("lookahead_distance", 0.05);
            this->declare_parameter<double>("max_linear_speed", 0.2);
            this->declare_parameter<double>("max_theta_speed", 2.0);
            this->declare_parameter<double>("Kp_tan", 0.80);
            this->declare_parameter<double>("Ki_tan", 0.00);
            this->declare_parameter<double>("Kd_tan", 0.00);
            this->declare_parameter<double>("Kp_norm", 0.80);
            this->declare_parameter<double>("Ki_norm", 0.00);
            this->declare_parameter<double>("Kd_norm", 0.00);
            this->declare_parameter<double>("Kp_theta", 0.40);
            this->declare_parameter<double>("Ki_theta", 0.00);
            this->declare_parameter<double>("Kd_theta", 0.00);
            this->declare_parameter<double>("max_linear_tolerance", 0.08);
            this->declare_parameter<double>("max_reaching_distance", 0.02);
            this->declare_parameter<double>("max_theta_tolerance", 0.3);
            this->declare_parameter<double>("max_reaching_theta", 0.1);
            this->declare_parameter<int>("x", 2);
            this->declare_parameter<double>("max_rotate_speed_", 0.5);
            this->declare_parameter<double>("slow_rotate_speed_", 0.4);
            this->declare_parameter<double>("accel_angle_", M_PI / 10);
            this->declare_parameter<double>("stop_angle_", M_PI / 90);
            this->get_parameter("lookahead_distance", lookahead_distance_);
            this->get_parameter("max_linear_speed", max_linear_speed_);
            this->get_parameter("max_theta_speed", max_theta_speed_);
            this->get_parameter("Kp_tan", Kp_tan);
            this->get_parameter("Ki_tan", Ki_tan);
            this->get_parameter("Kd_tan", Kd_tan);
            this->get_parameter("Kp_norm", Kp_norm);
            this->get_parameter("Ki_norm", Ki_norm);
            this->get_parameter("Kd_norm", Kd_norm);
            this->get_parameter("Kp_theta", Kp_theta);
            this->get_parameter("Ki_theta", Ki_theta);
            this->get_parameter("Kd_theta", Kd_theta);
            this->get_parameter("max_linear_tolerance", max_linear_tolerance);
            this->get_parameter("max_reaching_distance", max_reaching_distance);
            this->get_parameter("max_theta_tolerance", max_theta_tolerance);
            this->get_parameter("max_reaching_theta", max_reaching_theta);
            this->get_parameter("x", x_);
            this->get_parameter("max_rotate_speed_", max_rotate_speed_);
            this->get_parameter("slow_rotate_speed_", slow_rotate_speed_);
            this->get_parameter("accel_angle_", accel_angle_);
            this->get_parameter("stop_angle_", stop_angle_);

            reset_pose_client_ = this->create_client<nhk2026_msgs::srv::ResetPose>("reset_pose");


            linear_PID_tan_ = PIDController(Kp_tan, Ki_tan, Kd_tan, dt);
            linear_PID_norm_ = PIDController(Kp_norm, Ki_norm, Kd_norm, dt);
            omega_PID_ = PIDController(Kp_theta, Ki_theta, Kd_theta, dt, [](double e){
                while (e > M_PI) e -= 2*M_PI;
                while (e < -M_PI) e += 2*M_PI;
                return e;
            });



            rclcpp::QoS pathQos(rclcpp::KeepLast(5));
            path_sub_ = this->create_subscription<nav_msgs::msg::Path> (
                "route", pathQos, std::bind(&FollowNode::pathCallback, this, std::placeholders::_1)
            );
            rclcpp::QoS odomQos(rclcpp::KeepLast(5));
            // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
            //     "odom", odomQos, std::bind(&FollowNode::odomCallback, this, std::placeholders::_1)
            // );
            rclcpp::QoS poseQos(rclcpp::KeepLast(5));
            pose_sub_= this->create_subscription<geometry_msgs::msg::Pose2D> (
                "pose", poseQos, std::bind(&FollowNode::odomCallback, this, std::placeholders::_1)
            );
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            timer_ = rclcpp::create_timer(
                this,
                this->get_clock(),
                100ms,
                std::bind(&FollowNode::controlLoop, this)
            );

            rclcpp::QoS markerQos(rclcpp::KeepLast(10));
            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", markerQos);
            rclcpp::QoS poseArrowQos(rclcpp::KeepLast(10));
            pose_arrow_pub_= this->create_publisher<visualization_msgs::msg::Marker>("pose_arrow_marker", poseArrowQos);
            rclcpp::QoS cmdVelArrowQos(rclcpp::KeepLast(10));
            cmd_vel_arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>("cmd_vel_arrow_marker", cmdVelArrowQos);

            target_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("target_pose", 10);
            action_server_ = rclcpp_action::create_server<inrof2025_ros_type::action::Follow>(
                this,
                "follow",
                std::bind(&FollowNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&FollowNode::handleCancel, this, std::placeholders::_1),
                std::bind(&FollowNode::handleAccepted, this, std::placeholders::_1)
            );
        }

        std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle_;
        

    private:
        // action server callback
        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID &,
            std::shared_ptr<const inrof2025_ros_type::action::Follow::Goal> goal
        ) {
            if (!goal_handle_){
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            } else {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        rclcpp_action::CancelResponse handleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
        ) {
            goal_handle_.reset();
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
        ) {
            goal_handle_ = goal_handle;
        }

        void pathCallback(nav_msgs::msg::Path msgs) {
            // std::lock_guard<std::mutex> lock(mutex_);
            path_ = msgs.poses;
            current_waypoint_index_ = 0;
        }
        void odomCallback(geometry_msgs::msg::Pose2D msgs) {
            // std::lock_guard<std::mutex> lock(mutex_);
            pose_.x = msgs.x;
            pose_.y = msgs.y;
            pose_.theta = msgs.theta;
            // RCLCPP_INFO(this->get_logger(), "%.4f %.4f", pose_.x, pose_.y);
        }

        void resetWaypointIndex(double reset_x, double reset_y) {
            if (path_.empty()) return;

            int nearest_index = 0;

            for (int i = 0; i < static_cast<int>(path_.size()); i++) {
                double dx = path_[i].pose.position.x - reset_x;
                double dy = path_[i].pose.position.y - reset_y;
                double dist = std::hypot(dx, dy);
                if (dist < max_linear_tolerance) {
                    nearest_index = i;  // 条件を満たす中で最大インデックスを更新
                }
            }
            current_waypoint_index_ = nearest_index;
        }

        void controlLoop() {
            //do nothing if if there is no goal or path
            if (!goal_handle_){
                publishZero();
                return;
            }
            if (path_.empty()){
                publishZero();
                return;
            }
            if (is_rotating_) {
                publishZero();  
                double target_yaw = getYaw(path_[current_waypoint_index_+1].pose.orientation);
                rotate(target_yaw);
                return;
            }


            // publish goal position
            geometry_msgs::msg::Pose2D target_pose;
            target_pose.x = path_[current_waypoint_index_].pose.position.x;
            target_pose.y = path_[current_waypoint_index_].pose.position.y;
            //goal_pose.theta = path_[]

            target_pub_ ->publish(target_pose);

            //error calculation linear
            double dx = path_[current_waypoint_index_].pose.position.x - pose_.x;
            double dy = path_[current_waypoint_index_].pose.position.y - pose_.y;
            double tx = path_[current_waypoint_index_+1].pose.position.x - path_[current_waypoint_index_].pose.position.x;
            double ty = path_[current_waypoint_index_+1].pose.position.y - path_[current_waypoint_index_].pose.position.y;
            double norm = std::hypot(tx, ty);
            if (norm > 0) {
                tx /= norm;
                ty /= norm;
            }
            double nx = -ty;
            double ny = tx;
            double error_tan = dx * tx + dy * ty;
            double error_norm = dx * nx + dy * ny;
            double linear_error = std::hypot(dx, dy);
            double linear_goal_x = path_[path_.size() -1].pose.position.x - pose_.x;
            double linear_goal_y = path_[path_.size() -1].pose.position.y - pose_.y;
            double linear_goal_distance = std::hypot(linear_goal_x, linear_goal_y);


            //quoternion to yaw
            tf2::Quaternion q(
                path_[current_waypoint_index_].pose.orientation.x,
                path_[current_waypoint_index_].pose.orientation.y,
                path_[current_waypoint_index_].pose.orientation.z,
                path_[current_waypoint_index_].pose.orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            double theta_error = yaw - pose_.theta;
            while (theta_error > M_PI) theta_error -= 2*M_PI;
            while (theta_error < -M_PI) theta_error += 2*M_PI;

            tf2::Quaternion q_goal(
                path_[path_.size() - 1].pose.orientation.x,
                path_[path_.size() - 1].pose.orientation.y,
                path_[path_.size() - 1].pose.orientation.z,
                path_[path_.size() - 1].pose.orientation.w
            );

            double roll_goal, pitch_goal, yaw_goal;
            tf2::Matrix3x3(q_goal).getRPY(roll_goal, pitch_goal, yaw_goal);

            double theta_goal = yaw_goal - pose_.theta;
            while (theta_goal >= M_2_PI) theta_goal -= M_2_PI;
            while (theta_goal < 0) theta_goal += M_2_PI;

            //error calculation theta
            double target_theta = yaw;
            printWayPointArrow(path_[current_waypoint_index_].pose, path_[path_.size()-1].pose);

           // --- ★ここから修正版：RVizに表示するためのMarker publish ---
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // TFに合わせる
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoint_marker";
            marker.id = 0; // ← 常に同じIDを使うことで「上書き表示」できる！
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = path_[current_waypoint_index_].pose.position.x;
            marker.pose.position.y = path_[current_waypoint_index_].pose.position.y;
            marker.pose.position.z = path_[current_waypoint_index_].pose.position.z;
            marker.pose.orientation.w = 1.0;

            // 点の大きさ
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;

            // 色：赤
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // lifetimeを少し短くして上書き更新を確実にする
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            marker_pub_->publish(marker);
            // --- ★ここまで修正版 ---

            

            while (max_linear_tolerance > linear_error) {
                if (current_waypoint_index_+1 >= static_cast<int>(path_.size())) break;
                current_waypoint_index_++;
                linear_error = std::hypot(
                    path_[current_waypoint_index_].pose.position.x - pose_.x, 
                    path_[current_waypoint_index_].pose.position.y - pose_.y
                );
            }
            while (current_waypoint_index_+1 < static_cast<int>(path_.size())) {
                double tx = path_[current_waypoint_index_+1].pose.position.x - path_[current_waypoint_index_].pose.position.x;
                double ty = path_[current_waypoint_index_+1].pose.position.y - path_[current_waypoint_index_].pose.position.y;
                if (tx == 0 && ty == 0) {
                    current_waypoint_index_++;
                } else {
                    break;
                }
            }

            ignition::transport::Node node;
            //pathの目標zが今のpose_.zから変わった場合、その目標経路のところに瞬間移動する
            //3patternあって、引き算が正、負、0のときで場合分けする。正で上がる。負で下がる。0で変わらない。
            if (path_[current_waypoint_index_].pose.position.z - /*pose_.z*/ 0.0 > 0){
                ignition::msgs::Pose req;
                ignition::msgs::Boolean rep;
                bool result;
                req.set_name("robot");
                ignition::msgs::Vector3d* position = req.mutable_position();
                ignition::msgs::Quaternion* orientation = req.mutable_orientation();
                orientation->set_x(0.0);
                orientation->set_y(0.0);
                orientation->set_z(0.0);
                orientation->set_w(1.0);
                position->set_x(-1.825);
                position->set_y(3.8);
                position->set_z(0.21);

                
                bool executed = node.Request(
                    "/world/nhk2026/set_pose",
                    req,
                    1000,
                    rep,
                    result
                );

                std::shared_ptr<nhk2026_msgs::srv::ResetPose_Request> request = 
                    std::make_shared<nhk2026_msgs::srv::ResetPose::Request>();
                request->pose.position.x = -1.825;
                request->pose.position.y = 3.8;
                request->pose.position.z = 0.21;
                request->pose.orientation.x = 0.0;
                request->pose.orientation.y = 0.0;
                request->pose.orientation.z = 0.0;
                request->pose.orientation.w = 1.0;


                reset_pose_client_->async_send_request(
                    request,
                    [this, request](rclcpp::Client<nhk2026_msgs::srv::ResetPose>::SharedFuture future)
                    {
                        auto response = future.get();
                        RCLCPP_INFO(this->get_logger(),
                                    "Service call succeeded: %s",
                                    response->success ? "true" : "false");
                        if (response->success){
                            resetWaypointIndex(request->pose.position.x, request->pose.position.y);
                        }
        
                        
                    }
                );



            }

            





            if ((linear_goal_distance < max_reaching_distance)){ //&& theta_goal < max_reaching_theta) {
                //goal reached
                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                publishZero();
                auto result_msg = std::make_shared<inrof2025_ros_type::action::Follow::Result>();
                result_msg->success = true;
                goal_handle_->succeed(result_msg);
                goal_handle_.reset();
                return;
            }

            //PID control for linear speed
            double linear_cmd_tan = linear_PID_tan_.compute(error_tan, 0.0);
            double linear_cmd_norm = linear_PID_norm_.compute(error_norm, 0.0);
            
            //PID control for theta speed
            double theta_speed_cmd = omega_PID_.compute(target_theta, pose_.theta);


            //convert to x,y speed
            double linear_speed_cmd_x = linear_cmd_tan * tx + linear_cmd_norm * nx;
            double linear_speed_cmd_y = linear_cmd_tan * ty + linear_cmd_norm * ny;


            geometry_msgs::msg::Twist linear_speed;
            linear_speed.linear.x = cos(pose_.theta) * linear_speed_cmd_x + sin(pose_.theta) * linear_speed_cmd_y;
            linear_speed.linear.y = -sin(pose_.theta) * linear_speed_cmd_x + cos(pose_.theta) * linear_speed_cmd_y;
            linear_speed.angular.z = theta_speed_cmd;

            //apply speed limits 
            geometry_msgs::msg::Twist clipped_v = clip(linear_speed);
            cmd_pub_->publish(clipped_v);


            double clipped_v_x_r = clipped_v.linear.x;
            double clipped_v_y_r = clipped_v.linear.y;


            double clipped_v_x_f = cos(pose_.theta) * clipped_v_x_r - sin(pose_.theta) * clipped_v_y_r;
            double clipped_v_y_f = sin(pose_.theta) * clipped_v_x_r + cos(pose_.theta) * clipped_v_y_r; 


            printCmdVelArrow(linear_speed_cmd_x, linear_speed_cmd_y, clipped_v_x_f, clipped_v_y_f);
            
            //publish feedback
            auto feedback_msg = std::make_shared<inrof2025_ros_type::action::Follow::Feedback>();
            feedback_msg->x = pose_.x;
            feedback_msg->y = pose_.y;
            feedback_msg->theta = pose_.theta;
            goal_handle_->publish_feedback(feedback_msg);

        }
        double getYaw(const geometry_msgs::msg::Quaternion& q_msg) {
            tf2::Quaternion q;
            tf2::fromMsg(q_msg, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return yaw;
        };

        void printWayPointArrow(geometry_msgs::msg::Pose waypoint_pose, geometry_msgs::msg::Pose goal_pose) {
            visualization_msgs::msg::Marker arrow;

            // pub waypoint pose
            arrow.header.frame_id = "map";
            arrow.ns = "way_point_arrow";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = waypoint_pose;
            arrow.scale.x = 0.08;
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 0.0f;
            arrow.color.b = 1.0f;
            arrow.color.a = 1.0f;
            pose_arrow_pub_ -> publish(arrow);

            // pub goal pose
            arrow.header.frame_id = "map";
            arrow.ns = "goal_point_arrow";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = goal_pose;
            arrow.scale.x = 0.08;
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;
            pose_arrow_pub_ -> publish(arrow);
        }

        void printCmdVelArrow(double vx, double vy, double cliped_vx, double cliped_vy) {
            // convert pose2d to pose
            geometry_msgs::msg::Pose pose;
            pose.position.x = pose_.x;
            pose.position.y = pose_.y;
            pose.position.z = 0.0;
            tf2::Quaternion q;
            double yaw;
            if (std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6) {
                yaw = 0;
            } else {
                yaw = std::atan2(vy, vx);
            }
            q.setRPY(0, 0, yaw);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            // pub raw cmd vel 
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.ns = "cmd_vel";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = pose;
            arrow.scale.x = std::hypot(vx, vy);
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 0.0f;
            arrow.color.b = 1.0f;
            arrow.color.a = 0.5f;
            cmd_vel_arrow_pub->publish(arrow);

            // pub cliped cmd_vel
            arrow.header.frame_id = "map";
            arrow.ns = "cliped_cmd_vel";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = pose;
            arrow.scale.x = std::hypot(cliped_vx, cliped_vy);
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;
            cmd_vel_arrow_pub->publish(arrow);
        }


        MotorVel forwardKinematics(float vx, float vy, float vtheta) {
                MotorVel motor_vel;
                // motor_vel.v1 = vx + r_*vtheta;
                // motor_vel.v2 = 0.5 * vx + std::sqrt(3)/2*vy - r_*vtheta;
                // motor_vel.v3 = -0.5 * vx + std::sqrt(3)/2*vy + r_*vtheta;
                motor_vel.v1 = (-vy) + r_*vtheta;
                motor_vel.v2 = 0.5 * (-vy) + std::sqrt(3)/2*vx - r_*vtheta;
                motor_vel.v3 = -0.5 * (-vy) + std::sqrt(3)/2*vx + r_*vtheta;
                return motor_vel;
            }


        geometry_msgs::msg::Twist inverseKinematics(float v1, float v2, float v3) {
                geometry_msgs::msg::Twist twist;
                // twist.linear.y = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
                // twist.linear.x = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
                // twist.angular.z = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
                twist.linear.y = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
                twist.linear.x = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
                twist.angular.z = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
                return twist;
            }


        geometry_msgs::msg::Twist clip(geometry_msgs::msg::Twist cmd){
            double linear_speed_cmd_x;
            double linear_speed_cmd_y;
            double theta_speed_cmd;
            linear_speed_cmd_x = cmd.linear.x;
            linear_speed_cmd_y = cmd.linear.y;
            theta_speed_cmd = cmd.angular.z;

            MotorVel v_motor = forwardKinematics(linear_speed_cmd_x, linear_speed_cmd_y, theta_speed_cmd);
            double v1 = v_motor.v1;
            double v2 = v_motor.v2;
            double v3 = v_motor.v3;

            double v_max = std::max({std::abs(v1), std::abs(v2), std::abs(v3)});

            if (v_max > max_linear_speed_){
                double scale = max_linear_speed_ / v_max;
                v1 *= scale;
                v2 *= scale;
                v3 *= scale; 
            }

            return inverseKinematics(v1, v2, v3);
        }
        


        void publishZero()
        {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
        }

        // action server
        rclcpp_action::Server<inrof2025_ros_type::action::Follow>::SharedPtr action_server_;

        double lookahead_distance_;
        double max_linear_speed_;
        double max_theta_speed_;
        float r_ = 0.14;

        // subscriber
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr target_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_arrow_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vel_arrow_pub;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<geometry_msgs::msg::PoseStamped> path_;
        std::mutex mutex_;
        geometry_msgs::msg::Pose2D pose_;

        rclcpp::Client<nhk2026_msgs::srv::ResetPose>::SharedPtr reset_pose_client_;

        //PID control
        PIDController linear_PID_tan_, linear_PID_norm_, omega_PID_;

        

        //tolerance and reaching distance
        double max_linear_tolerance;
        double max_theta_tolerance;
        double max_reaching_distance;
        double max_reaching_theta;

        // waypoint index
        int current_waypoint_index_;    
        int x_;

        // rotate control
        bool   is_rotating_       = false;
        double max_rotate_speed_  = 0.5;
        double slow_rotate_speed_ = 0.4;
        double accel_angle_       = M_PI / 10;
        double stop_angle_        = M_PI / 90;
        
        // rotate action server
        rclcpp_action::Server<inrof2025_ros_type::action::Rotate>::SharedPtr action_rotate_server_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Rotate>> goal_rotate_handle_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowNode>());
    rclcpp::shutdown();
    return 0;
}