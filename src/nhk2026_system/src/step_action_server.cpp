#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm> // std::min, std::max用

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nhk2026_msgs/action/step_move.hpp" 
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp" 
#include <mutex> 

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2;

class StepActionServer : public rclcpp::Node {
public:
    using StepMove = nhk2026_msgs::action::StepMove;
    using GoalHandleStepMove = rclcpp_action::ServerGoalHandle<StepMove>;

    StepActionServer() : Node("step_leg_sequencer") { 
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        this->action_server_ = rclcpp_action::create_server<StepMove>(
            this, "step_leg_sequence", 
            std::bind(&StepActionServer::handle_goal, this, _1, _2),
            std::bind(&StepActionServer::handle_cancel, this, _1),
            std::bind(&StepActionServer::handle_accepted, this, _1),
            rcl_action_server_get_default_options(),
            callback_group_
        );

        rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);
        
        zaxis_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("zaxis", 10);
        robomas_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("leg_robomas", reliable_qos);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", reliable_qos);
        legs_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("step_legs", reliable_qos);

        count = 0;
        
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;
        
        lidar_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "onedlidar", 10, std::bind(&StepActionServer::lidar_callback, this, _1), sub_opt
        );
        
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_step", reliable_qos, std::bind(&StepActionServer::joint_state_callback, this, _1), sub_opt
        );

        zaxis_timer_ = this->create_wall_timer(
            10ms, std::bind(&StepActionServer::publish_zaxis_periodic, this));
            
        this->declare_parameter<double>("kPosTolerance", 0.05);
        this->declare_parameter<double>("max_leg_rad_per_sec", 2.0); // 補間速度のパラメータ化
        kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();
        max_leg_vel_ = this->get_parameter("max_leg_rad_per_sec").as_double();
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const StepMove::Goal> goal) {
        if (!joint_subscribe_flag_) {
            RCLCPP_ERROR(this->get_logger(), "Joint state (/joint_states_step) not received yet.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleStepMove>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleStepMove> goal_handle) {
        std::thread{std::bind(&StepActionServer::execute, this, _1), goal_handle}.detach();
    }

    // 同期補間ロジック：左右の脚を同じ値で更新する
    void update_interpolation(double dt) {
        for (size_t i = 0; i < 3; ++i) {
            double diff = target_leg_pos_[i] - current_cmd_pos_[i];
            if (std::fabs(diff) > 0.001) {
                // 最大速度制限に基づいたステップ量
                double step = std::copysign(std::min(std::fabs(diff), max_leg_vel_ * dt), diff);
                current_cmd_pos_[i] += step;
            }
        }
        // 強制同期：左右（0と1）を平均または片方に合わせる
        double sync_val = (current_cmd_pos_[0] + current_cmd_pos_[1]) / 2.0;
        current_cmd_pos_[0] = sync_val;
        current_cmd_pos_[1] = sync_val;
    }

    void execute(const std::shared_ptr<GoalHandleStepMove> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<StepMove::Result>();
        
        double hz = 100.0; // 制御周期を少し上げると滑らかになります
        rclcpp::Rate loop_rate(hz);
        double dt = 1.0 / hz;

        int step = 0;
        auto state_start_time = this->now();
        
        // 現在の関節位置から補間を開始する
        {
            std::lock_guard<std::mutex> lock(joint_mutex_);
            current_cmd_pos_ = {now_joint_.position[0], now_joint_.position[1], now_joint_.position[2]};
            target_leg_pos_ = current_cmd_pos_;
        }

        float target_robomas = 0.0f;
        geometry_msgs::msg::Twist target_cmd_vel;

        if (goal->msg == "step up" && count < 2) {
            RCLCPP_INFO(this->get_logger(), "=== 段上りシーケンス開始 (count: %d) ===", count);
            while (rclcpp::ok()) {
                if (goal_handle->is_canceling()) {
                    stop_all();
                    result->success = false; result->msg = "Canceled";
                    goal_handle->canceled(result);
                    return;
                }

                switch (step) {
                    case 0:
                        zaxics_count = nowzaxices;
                        target_leg_pos_ = {3.14 + count * 6.28, 3.14 + count * 6.28, 0.0};
                        if (leg_reached()) { zaxics_count++; next_step(step, state_start_time); }
                        break;
                    case 1:
                        target_leg_pos_ = {4.57 + count * 6.28, 4.57 + count * 6.28, -1.57};
                        target_robomas = 0.3f;
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 2:
                        target_robomas = 0.3f; 
                        if (check_lidar(0, 0)) next_step(step, state_start_time);
                        break;
                    case 3:
                        target_robomas = 0.3f; target_cmd_vel.linear.y = 0.3;
                        target_leg_pos_ = {6.1 + count * 6.28, 6.1 + count * 6.28, -1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 4:
                        target_robomas = 0.3f; target_cmd_vel.linear.y = 0.3;
                        if (elapsed_sec(state_start_time) > 0.5) next_step(step, state_start_time);
                        break;
                    case 5:
                        target_robomas = 0.3f; target_cmd_vel.linear.y = 0.5;
                        if (check_lidar(1, 0)) next_step(step, state_start_time);
                        break;
                    case 6:
                        target_leg_pos_ = {6.1 + count * 6.28, 6.1 + count * 6.28, 0.0};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 7:
                        target_robomas = 0.0f; target_cmd_vel.linear.y = 0.5;
                        if (elapsed_sec(state_start_time) > 0.5) next_step(step, state_start_time);
                        break;
                    case 8:
                        target_robomas = 0.0f; target_cmd_vel.linear.y = 0.0;
                        target_leg_pos_ = {6.28 + count * 6.28, 6.28 + count * 6.28, 1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 9:
                        nowzaxices = zaxics_count;
                        zaxics_count = 10;
                        if (elapsed_sec(state_start_time) > 2.0) next_step(step, state_start_time);
                    default:
                        count++;
                        stop_all();
                        result->success = true; result->msg = "Step up Completed!";
                        goal_handle->succeed(result);
                        return;
                }

                update_interpolation(dt);
                publish_all(target_robomas, target_cmd_vel);
                loop_rate.sleep();
            }

        } else if (goal->msg == "step down" && count > -2) {
            RCLCPP_INFO(this->get_logger(), "=== 段降りシーケンス開始 (count: %d) ===", count);
            count--;
            while (rclcpp::ok()) {
                if (goal_handle->is_canceling()) {
                    stop_all();
                    result->success = false; result->msg = "Canceled";
                    goal_handle->canceled(result);
                    return;
                }

                switch (step) {
                    case 0:
                        zaxics_count = nowzaxices;
                        target_leg_pos_ = {6.1 + count * 6.28, 6.1 + count * 6.28, 0.0};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 1:
                        target_robomas = -0.3f; target_cmd_vel.linear.y = -0.5;
                        if (check_lidar(1, 1)) next_step(step, state_start_time);
                        break;
                    case 2:
                        target_leg_pos_ = {6.1 + count * 6.28, 6.1 + count * 6.28, -1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 3:
                        target_robomas = -0.3f; target_cmd_vel.linear.y = -0.5;
                        if (check_lidar(0, 1)) next_step(step, state_start_time);
                        break;
                    case 4:
                        target_leg_pos_ = {4.57 + count * 6.28, 4.57 + count * 6.28, -1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 5:
                        target_robomas = -0.3f; target_cmd_vel.linear.y = 0.0;
                        if (elapsed_sec(state_start_time) > 0.8) next_step(step, state_start_time);
                        break;
                    case 6:
                        target_robomas = 0.0f; leg_max_speed_ = 1.0; leg_max_acc_ = 1.0;
                        target_leg_pos_ = {3.66 + count * 6.28, 3.66 + count * 6.28, -0.52};
                        if (leg_reached()) {next_step(step, state_start_time); }
                        break;
                    case 7:
                        if (elapsed_sec(state_start_time) > 0.1) next_step(step, state_start_time);
                        break;
                    case 8:
                        target_robomas = 0.0f;
                        target_leg_pos_ = {3.14 + count * 6.28, 3.14 + count * 6.28, 0.0};
                        if (leg_reached()) { zaxics_count--; next_step(step, state_start_time); }
                        break;
                    case 9:
                        leg_max_speed_ = 50.0; leg_max_acc_ = 20.0;
                        target_leg_pos_ = {0.0 + count * 6.28, 0.0 + count * 6.28, 1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 10:
                        nowzaxices = zaxics_count;
                        zaxics_count = 10;
                        if (elapsed_sec(state_start_time) > 2.0) next_step(step, state_start_time);
                    default:
                        stop_all();
                        result->success = true; result->msg = "Step down Completed!";
                        goal_handle->succeed(result);
                        return;
                }
                update_interpolation(dt);
                publish_all(target_robomas, target_cmd_vel);
                loop_rate.sleep();
            }
        } else {
            result->success = false; result->msg = "Invalid request or count limit";
            goal_handle->abort(result);
        }
    }

    void publish_all(float target_robomas, geometry_msgs::msg::Twist target_cmd_vel) {
        std_msgs::msg::Float32MultiArray rb_msg;
        rb_msg.data = {target_robomas};
        robomas_pub_->publish(rb_msg);
        cmd_vel_pub_->publish(target_cmd_vel);

        if (joint_subscribe_flag_) {
            std_msgs::msg::Float32MultiArray legs_cmd;
            // 補間された current_cmd_pos_ をパブリッシュ
            legs_cmd.data = {
                leg_max_speed_,
                leg_max_acc_,
                static_cast<float>(current_cmd_pos_[0]),
                static_cast<float>(current_cmd_pos_[1]),
                static_cast<float>(current_cmd_pos_[2])
            };
            legs_pub_->publish(legs_cmd);
        }
    }

    void publish_zaxis_periodic() {
        std_msgs::msg::Int32MultiArray msg;
        msg.data.push_back(this->zaxics_count);
        zaxis_pub_->publish(msg);
    }

    // 判定は「最終目標値」に到達したかで見る
    bool leg_reached() {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (!joint_subscribe_flag_ || now_joint_.position.size() < 3) return false;
        bool right_reached = std::fabs(now_joint_.position[0] - target_leg_pos_[0]) <= kPosTolerance_;
        bool left_reached  = std::fabs(now_joint_.position[1] - target_leg_pos_[1]) <= kPosTolerance_;
        bool back_reached  = std::fabs(now_joint_.position[2] - target_leg_pos_[2]) <= kPosTolerance_;
        return right_reached && left_reached && back_reached;
    }

    void next_step(int& step, rclcpp::Time& start_time) {
        step++;
        start_time = this->now();
    }

    double elapsed_sec(const rclcpp::Time& start_time) {
        return (this->now() - start_time).seconds();
    }

    bool check_lidar(size_t target_index, int target_value) {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        if (target_index < lidar_data_.size() && lidar_data_[target_index] == target_value) {
            return true;
        }
        return false;
    }

    void stop_all() {
        std_msgs::msg::Float32MultiArray stop_rb;
        stop_rb.data = {0.0f};
        robomas_pub_->publish(stop_rb);
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        // 現在の補間位置を最終目標にして停止
        target_leg_pos_ = current_cmd_pos_;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (msg->position.size() >= 3) {
            now_joint_ = *msg;
            joint_subscribe_flag_ = true;
        }
    }

    void lidar_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(lidar_mutex_); 
        lidar_data_ = msg->data;
    }

    // メンバ変数
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Server<StepMove>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robomas_pub_, legs_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr zaxis_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr zaxis_timer_;

    std::vector<int> lidar_data_;
    std::mutex lidar_mutex_, joint_mutex_;
    int count, zaxics_count = 0;
    int nowzaxices = 0;
    sensor_msgs::msg::JointState now_joint_;
    bool joint_subscribe_flag_ = false;
    double kPosTolerance_, max_leg_vel_;
    
    std::vector<double> target_leg_pos_ = {0.0, 0.0, 0.0};  // 最終目標
    std::vector<double> current_cmd_pos_ = {0.0, 0.0, 0.0}; // 補間中の指令値
    float leg_max_speed_ = 50.0f;
    float leg_max_acc_ = 20.0f;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StepActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}