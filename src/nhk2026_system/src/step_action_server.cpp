#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nhk2026_msgs/action/step_move.hpp" 
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // 脚のジョイント用に追加
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
        
        // パブリッシャーの初期化
        zaxis_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("zaxis", 10);
        robomas_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("leg_robomas", reliable_qos);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", reliable_qos);
        right_leg_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("right_leg", reliable_qos);
        left_leg_pub_  = this->create_publisher<std_msgs::msg::Float32MultiArray>("left_leg", reliable_qos);
        back_leg_pub_  = this->create_publisher<std_msgs::msg::Float32MultiArray>("back_leg", reliable_qos);

        count = 0;
        
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;
        
        // サブスクライバーの初期化
        lidar_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "onedlidar", 10, std::bind(&StepActionServer::lidar_callback, this, _1), sub_opt
        );
        
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_step", reliable_qos, std::bind(&StepActionServer::joint_state_callback, this, _1), sub_opt
        );

        zaxis_timer_ = this->create_wall_timer(
            10ms, std::bind(&StepActionServer::publish_zaxis_periodic, this));
            
        this->declare_parameter<double>("kPosTolerance", 0.05);
        this->kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();
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

    void publish_zaxis_periodic() {
        std_msgs::msg::Int32MultiArray msg;
        msg.data.push_back(this->zaxics_count);
        zaxis_pub_->publish(msg);
    }

    void execute(const std::shared_ptr<GoalHandleStepMove> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<StepMove::Result>();

        rclcpp::Rate loop_rate(10.0); // 50Hzで全体を回す

        int step = 0;
        auto state_start_time = this->now();
        
        target_leg_pos_ = {now_joint_.position[0], now_joint_.position[1], now_joint_.position[2]};
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
                        target_robomas = 0.3f; target_cmd_vel.linear.y = 0.5;
                        if (check_lidar(1, 0)) next_step(step, state_start_time);
                        break;
                    case 5:
                        target_leg_pos_ = {6.1 + count * 6.28, 6.1 + count * 6.28, 0.0};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 6:
                        target_robomas = 0.0f; target_cmd_vel.linear.y = 0.5;
                        if (elapsed_sec(state_start_time) > 1.0) next_step(step, state_start_time);
                        break;
                    case 7:
                        target_robomas = 0.0f; target_cmd_vel.linear.y = 0.0;
                        target_leg_pos_ = {6.28 + count * 6.28, 6.28 + count * 6.28, 1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    default:
                        // 全ステップ完了
                        count++;
                        stop_all();
                        result->success = true; result->msg = "Step up Completed!";
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "=== 段上りシーケンス正常終了 ===");
                        return;
                }

               
                publish_all(target_robomas, target_cmd_vel);
                loop_rate.sleep();
                RCLCPP_INFO(this->get_logger(), "step: %d", step);
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
                        target_leg_pos_ = {6.1 + count * 6.28, 6.1 + count * 6.28, 0.0};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 1:
                        target_robomas = 0.0f; target_cmd_vel.linear.y = -0.5;
                        if (check_lidar(1, 1)) next_step(step, state_start_time);
                        break;
                    case 2:
                        target_cmd_vel.linear.y = 0.0;
                        if (elapsed_sec(state_start_time) > 0.2) next_step(step, state_start_time);
                        break;
                    case 3:
                        target_leg_pos_ = {6.1 + count * 6.28, 6.1 + count * 6.28, -1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 4:
                        target_robomas = -0.3f; target_cmd_vel.linear.y = -0.5;
                        if (check_lidar(0, 1)) next_step(step, state_start_time);
                        break;
                    case 5:
                        target_robomas = 0.0f; target_cmd_vel.linear.y = 0.0;
                        if (elapsed_sec(state_start_time) > 0.2) next_step(step, state_start_time);
                        break;
                    case 6:
                        target_leg_pos_ = {4.57 + count * 6.28, 4.57 + count * 6.28, -1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    case 7:
                        target_robomas = -0.3f; target_cmd_vel.linear.y = 0.0;
                        if (elapsed_sec(state_start_time) > 1.0) next_step(step, state_start_time);
                        break;
                    case 8:
                        target_robomas = 0.0f;
                        if (elapsed_sec(state_start_time) > 0.2) next_step(step, state_start_time);
                        break;
                    case 9:
                        target_leg_pos_ = {3.14 + count * 6.28, 3.14 + count * 6.28, 0.0};
                        if (leg_reached()) { zaxics_count--; next_step(step, state_start_time); }
                        break;
                    case 10:
                        target_cmd_vel.linear.y = 0.0;
                        if (elapsed_sec(state_start_time) > 0.2) next_step(step, state_start_time);
                        break;
                    case 11:
                        target_robomas = 0.0f; // 元コードが publish_robomas_for_duration(0.0, 0.5) だったので0でキープ
                        if (elapsed_sec(state_start_time) > 0.5) next_step(step, state_start_time);
                        break;
                    case 12:
                        target_cmd_vel.linear.y = 0.0;
                        if (elapsed_sec(state_start_time) > 0.2) next_step(step, state_start_time);
                        break;
                    case 13:
                        target_leg_pos_ = {0.0 + count * 6.28, 0.0 + count * 6.28, 1.57};
                        if (leg_reached()) next_step(step, state_start_time);
                        break;
                    default:
                        stop_all();
                        result->success = true; result->msg = "Step down Completed!";
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "=== 段降りシーケンス正常終了 ===");
                        return;
                }

                publish_all(target_robomas, target_cmd_vel);
                loop_rate.sleep();
            }
        } else {
            result->success = false; result->msg = "Invalid request or count limit";
            goal_handle->abort(result);
        }
    }

 
    // 全トピックを一斉にパブリッシュする
    void publish_all(float target_robomas, geometry_msgs::msg::Twist target_cmd_vel) {
        std_msgs::msg::Float32MultiArray rb_msg;
        rb_msg.data = {target_robomas};
        robomas_pub_->publish(rb_msg);
        cmd_vel_pub_->publish(target_cmd_vel);

        if (joint_subscribe_flag_) {
            bool right_reached = std::fabs(now_joint_.position[0] - target_leg_pos_[0]) <= kPosTolerance_;
            bool left_reached  = std::fabs(now_joint_.position[1] - target_leg_pos_[1]) <= kPosTolerance_;
            bool back_reached  = std::fabs(now_joint_.position[2] - target_leg_pos_[2]) <= kPosTolerance_;

            if (!right_reached) {
                std_msgs::msg::Float32MultiArray cmd;
                cmd.data = {static_cast<float>(target_leg_pos_[0]), leg_max_speed_, leg_max_acc_};
                right_leg_pub_->publish(cmd);
            }
            if (!left_reached) {
                std_msgs::msg::Float32MultiArray cmd;
                cmd.data = {static_cast<float>(target_leg_pos_[1]), leg_max_speed_, leg_max_acc_};
                left_leg_pub_->publish(cmd);
            }
            if (!back_reached) {
                std_msgs::msg::Float32MultiArray cmd;
                cmd.data = {static_cast<float>(target_leg_pos_[2]), leg_max_speed_, leg_max_acc_};
                back_leg_pub_->publish(cmd);
            }
        }
    }

    bool leg_reached() {
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
        // 脚は現在位置を目標に書き換えて停止させる
        target_leg_pos_ = {now_joint_.position[0], now_joint_.position[1], now_joint_.position[2]};
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() >= 3) {
            now_joint_ = *msg;
            joint_subscribe_flag_ = true;
        }
    }

    void lidar_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(lidar_mutex_); 
        lidar_data_ = msg->data;
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Server<StepMove>::SharedPtr action_server_;
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robomas_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr zaxis_pub_;
    rclcpp::TimerBase::SharedPtr zaxis_timer_;

    // 脚用トピック
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr right_leg_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr left_leg_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr back_leg_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr lidar_sub_;
    
    std::vector<int> lidar_data_;
    std::mutex lidar_mutex_;
    
    int count; 
    int zaxics_count = 0;

    // 脚制御用の変数
    sensor_msgs::msg::JointState now_joint_;
    bool joint_subscribe_flag_ = false;
    double kPosTolerance_;
    std::vector<double> target_leg_pos_ = {0.0, 0.0, 0.0};
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