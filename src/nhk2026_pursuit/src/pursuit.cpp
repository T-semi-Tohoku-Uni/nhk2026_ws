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
#include "nhk2026_msgs/action/step_move.hpp"

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
        using StepMove = nhk2026_msgs::action::StepMove;
        using GoalHandleStepMove = rclcpp_action::ClientGoalHandle<StepMove>;

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
            this->declare_parameter<double>("offset_z_", 0.2);

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
            this->get_parameter("offset_z_", offset_z_);

            reset_pose_client_ = this->create_client<nhk2026_msgs::srv::ResetPose>("reset_pose");

            linear_PID_tan_ = PIDController(Kp_tan, Ki_tan, Kd_tan, dt);
            linear_PID_norm_ = PIDController(Kp_norm, Ki_norm, Kd_norm, dt);
            omega_PID_ = PIDController(Kp_theta, Ki_theta, Kd_theta, dt, [](double e){
                while (e > M_PI) e -= 2*M_PI;
                while (e < -M_PI) e += 2*M_PI;
                return e;
            });

            rclcpp::QoS pathQoS(rclcpp::KeepLast(5));
            path_sub_ = this->create_subscription<nav_msgs::msg::Path> (
                "route", pathQoS, std::bind(&FollowNode::pathCallback, this, std::placeholders::_1)
            );
            pose_sub_= this->create_subscription<geometry_msgs::msg::Pose>(
                "pose", 10, std::bind(&FollowNode::odomCallback, this, std::placeholders::_1)
            );
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            timer_ = rclcpp::create_timer(
                this, this->get_clock(), 100ms, std::bind(&FollowNode::controlLoop, this)
            );

            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);
            pose_arrow_pub_= this->create_publisher<visualization_msgs::msg::Marker>("pose_arrow_marker", 10);
            cmd_vel_arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>("cmd_vel_arrow_marker", 10);
            target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);

            action_server_ = rclcpp_action::create_server<inrof2025_ros_type::action::Follow>(
                this, "follow",
                std::bind(&FollowNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&FollowNode::handleCancel, this, std::placeholders::_1),
                std::bind(&FollowNode::handleAccepted, this, std::placeholders::_1)
            );

            action_client_ = rclcpp_action::create_client<StepMove>(this, "step_leg_sequence");
        }

    private:
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const inrof2025_ros_type::action::Follow::Goal>) {
            return goal_handle_ ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle) {
            goal_handle_.reset();
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle) {
            goal_handle_ = goal_handle;
        }

        void pathCallback(nav_msgs::msg::Path msgs) {
            path_ = msgs.poses;
            current_waypoint_index_ = 0;
        }

        void odomCallback(geometry_msgs::msg::Pose msgs) {
            pose_ = msgs;
        }

        double getYaw(const geometry_msgs::msg::Quaternion& q_msg) {
            tf2::Quaternion q;
            tf2::fromMsg(q_msg, q);
            double r, p, y;
            tf2::Matrix3x3(q).getRPY(r, p, y);
            return y;
        }

        double normalizePi(double a) {
            a = std::fmod(a + M_PI, 2.0 * M_PI);
            if (a < 0) a += 2.0 * M_PI;
            return a - M_PI;
        }

        void publishZero() {
            geometry_msgs::msg::Twist cmd;
            cmd_pub_->publish(cmd);
        }

        void rotate(double targetTheta) {
            double yaw = getYaw(pose_.orientation);
            double err = normalizePi(targetTheta - yaw);
            double abs_err = std::abs(err);
            if (abs_err < stop_angle_) {
                publishZero();
                is_rotating_ = false;
                RCLCPP_INFO(this->get_logger(), "Rotation completed.");
                current_waypoint_index_++;
                return;
            }
            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = (err > 0 ? 1 : -1) * (abs_err > accel_angle_ ? max_rotate_speed_ : slow_rotate_speed_);
            cmd_pub_->publish(cmd);
        }

        void controlLoop() {
            if (!goal_handle_ || path_.empty()) {
                publishZero();
                return;
            }

            if (is_rotating_) {
                publishZero();
                if (current_waypoint_index_ + 1 < static_cast<int>(path_.size())) {
                    rotate(getYaw(path_[current_waypoint_index_ + 1].pose.orientation));
                } else {
                    is_rotating_ = false;
                }
                return;
            }

            if (is_jump_ || is_action_busy_) return;

            // ゴール判定
            const auto& goal_pose = path_.back().pose;
            double yaw_goal = getYaw(goal_pose.orientation);
            double linear_goal_distance = std::hypot(goal_pose.position.x - pose_.position.x, goal_pose.position.y - pose_.position.y);
            double theta_goal = normalizePi(yaw_goal - getYaw(pose_.orientation));

            if (linear_goal_distance < max_reaching_distance) {
                if (std::abs(theta_goal) < max_reaching_theta) {
                    RCLCPP_INFO(this->get_logger(), "Goal reached.");
                    publishZero();
                    auto result_msg = std::make_shared<inrof2025_ros_type::action::Follow::Result>();
                    result_msg->success = true;
                    goal_handle_->succeed(result_msg);
                    goal_handle_.reset();
                    return;
                } else {
                    publishZero();
                    geometry_msgs::msg::Twist cmd;
                    cmd.angular.z = omega_PID_.compute(yaw_goal, getYaw(pose_.orientation));
                    cmd_pub_->publish(cmd);
                    return;
                }
            }

            // 通常追従
            target_pub_->publish(path_[current_waypoint_index_].pose);
            double dx = path_[current_waypoint_index_].pose.position.x - pose_.position.x;
            double dy = path_[current_waypoint_index_].pose.position.y - pose_.position.y;
            double tx = path_[current_waypoint_index_ + 1].pose.position.x - path_[current_waypoint_index_].pose.position.x;
            double ty = path_[current_waypoint_index_ + 1].pose.position.y - path_[current_waypoint_index_].pose.position.y;
            double norm = std::hypot(tx, ty);
            if (norm > 0) { tx /= norm; ty /= norm; }
            double nx = -ty, ny = tx;
            double error_tan = dx * tx + dy * ty;
            double error_norm = dx * nx + dy * ny;
            double linear_error = std::hypot(dx, dy);

            while (max_linear_tolerance > linear_error) {
                if (current_waypoint_index_ + 1 >= static_cast<int>(path_.size())) break;
                if (std::abs(normalizePi(getYaw(path_[current_waypoint_index_ + 1].pose.orientation) - getYaw(path_[current_waypoint_index_].pose.orientation))) > 1e-2) {
                    if (linear_error < max_reaching_distance) is_rotating_ = true;
                    break;
                }
                current_waypoint_index_++;
                linear_error = std::hypot(path_[current_waypoint_index_].pose.position.x - pose_.position.x, path_[current_waypoint_index_].pose.position.y - pose_.position.y);
            }

            double cur_yaw = getYaw(pose_.orientation);
            double vx_raw = linear_PID_tan_.compute(error_tan, 0.0) * tx + linear_PID_norm_.compute(error_norm, 0.0) * nx;
            double vy_raw = linear_PID_tan_.compute(error_tan, 0.0) * ty + linear_PID_norm_.compute(error_norm, 0.0) * ny;
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = cos(cur_yaw) * vx_raw + sin(cur_yaw) * vy_raw;
            cmd.linear.y = -sin(cur_yaw) * vx_raw + cos(cur_yaw) * vy_raw;
            cmd.angular.z = omega_PID_.compute(getYaw(path_[current_waypoint_index_].pose.orientation), cur_yaw);
            cmd_pub_->publish(clip(cmd));
        }

        MotorVel forwardKinematics(float vx, float vy, float vtheta) {
            MotorVel m;
            m.v1 = (-vy) + r_ * vtheta;
            m.v2 = 0.5 * (-vy) + std::sqrt(3)/2 * vx - r_ * vtheta;
            m.v3 = -0.5 * (-vy) + std::sqrt(3)/2 * vx + r_ * vtheta;
            return m;
        }

        geometry_msgs::msg::Twist inverseKinematics(float v1, float v2, float v3) {
            geometry_msgs::msg::Twist t;
            t.linear.y = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
            t.linear.x = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
            t.angular.z = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
            return t;
        }

        geometry_msgs::msg::Twist clip(geometry_msgs::msg::Twist cmd) {
            MotorVel v = forwardKinematics(cmd.linear.x, cmd.linear.y, cmd.angular.z);
            double v_max = std::max({std::abs(v.v1), std::abs(v.v2), std::abs(v.v3)});
            if (v_max > max_linear_speed_) {
                double s = max_linear_speed_ / v_max;
                v.v1 *= s; v.v2 *= s; v.v3 *= s;
            }
            return inverseKinematics(v.v1, v.v2, v.v3);
        }

        // メンバ変数
        std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle_;
        double lookahead_distance_, max_linear_speed_, max_theta_speed_, r_ = 0.14;
        double max_linear_tolerance, max_theta_tolerance, max_reaching_distance, max_reaching_theta;
        double max_rotate_speed_, slow_rotate_speed_, accel_angle_, stop_angle_, offset_z_;
        int current_waypoint_index_ = 0, x_;
        bool is_rotating_ = false, is_jump_ = false, is_action_busy_ = false;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_, pose_arrow_pub_, cmd_vel_arrow_pub;
        rclcpp::TimerBase::SharedPtr timer_;
        
        std::vector<geometry_msgs::msg::PoseStamped> path_;
        geometry_msgs::msg::Pose pose_;
        PIDController linear_PID_tan_, linear_PID_norm_, omega_PID_;
        
        rclcpp_action::Server<inrof2025_ros_type::action::Follow>::SharedPtr action_server_;
        rclcpp_action::Client<StepMove>::SharedPtr action_client_;
        rclcpp::Client<nhk2026_msgs::srv::ResetPose>::SharedPtr reset_pose_client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowNode>());
    rclcpp::shutdown();
    return 0;
}