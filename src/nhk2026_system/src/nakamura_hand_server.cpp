#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nhk2026_msgs/action/nakamura_hand.hpp" 
#include "std_msgs/msg/float32_multi_array.hpp"
#include <mutex> 

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2;

class NakamuraHandServer : public rclcpp::Node {
public:
    using NakamuraHand = nhk2026_msgs::action::NakamuraHand;
    using GoalHandleNakamuraHand = rclcpp_action::ServerGoalHandle<NakamuraHand>;

    NakamuraHandServer() : Node("nakamura_hand_sequencer") { 
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        this->action_server_ = rclcpp_action::create_server<NakamuraHand>(
            this, "nakamura_hand_sequence", 
            std::bind(&NakamuraHandServer::handle_goal, this, _1, _2),
            std::bind(&NakamuraHandServer::handle_cancel, this, _1),
            std::bind(&NakamuraHandServer::handle_accepted, this, _1),
            rcl_action_server_get_default_options(),
            callback_group_
        );

        rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);
        
        robomas_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("nakamura_hand", reliable_qos);

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;
        
        joint_states_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "nakamura_hand_feedback", reliable_qos, std::bind(&NakamuraHandServer::joint_state_callback, this, _1), sub_opt
        );


        this->declare_parameter<double>("kPosTolerance", 0.05);
        this->kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const NakamuraHand::Goal> goal) {
        if (!joint_subscribe_flag_) {
            RCLCPP_ERROR(this->get_logger(), "Feedback not received yet.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // ステップの妥当性チェック
        if (goal->firststep < 0 || goal->firststep > goal->finalstep) {
            RCLCPP_ERROR(this->get_logger(), "Invalid step range: %d to %d", goal->firststep, goal->finalstep);
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNakamuraHand>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNakamuraHand> goal_handle) {
        std::thread{std::bind(&NakamuraHandServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNakamuraHand> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<NakamuraHand::Result>();
        auto feedback = std::make_shared<NakamuraHand::Feedback>();

        rclcpp::Rate loop_rate(10.0);
        
        // 開始ステップを代入
        int step = goal->firststep;
        auto state_start_time = this->now();
        
        target_hand_pos_ = now_joint_;
        // 引数のposをホルダーの目標位置に反映

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                stop_all();
                result->success = false;
                result->msg = "Goal canceled at step " + std::to_string(step);
                goal_handle->canceled(result);
                return;
            }

            // --- ステートマシン: finalstep を完了するまで実行 ---
            if (step <= goal->finalstep) {
                switch (step) {
                    case 0:
                        target_hand_pos_ = {0.0,0.0 ,0.0,0.0 ,1.57};
                        if (hand_reached()) next_step(step, state_start_time);
                        break;
                    case 1:
                        target_hand_pos_ = {1.0, 0.0,0.0, 1.57};
                        if (hand_reached()) next_step(step, state_start_time);
                        break;
                    case 2:
                        target_hand_pos_ = {0.0, 0.0,0.0, 1.57};
                        if (hand_reached()) next_step(step, state_start_time);
                        break;
                    case 3:
                        target_hand_pos_ = {0.0, 1.0,0.0, 1.57};
                        if (hand_reached()) next_step(step, state_start_time);
                        break;
                    case 4:
                        target_hand_pos_ = {0.0, 1.0,0.0, 3.14};
                        if (hand_reached()) next_step(step, state_start_time);
                        break;
                    default:
                        RCLCPP_WARN(this->get_logger(), "Step %d is not defined. Terminating.", step);
                        step = goal->finalstep + 1; // ループを抜ける
                        break;
                }
                feedback->msg = "Executing step " + std::to_string(step);
                goal_handle->publish_feedback(feedback);
            } else {
                // 全ての指定ステップが完了
                stop_all();
                result->success = true;
                result->msg = "Sequence completed from " + std::to_string(goal->firststep) + " to " + std::to_string(goal->finalstep);
                goal_handle->succeed(result);
                return;
            }

            publish_all();
            loop_rate.sleep();
        }
    }

    // --- (以下のメソッドは変更なし) ---
    void publish_all() {
        if (joint_subscribe_flag_) {
            std_msgs::msg::Float32MultiArray hand_msg;
            hand_msg.data = {(float)target_hand_pos_[0], (float)target_hand_pos_[1], (float)target_hand_pos_[2],(float)target_hand_pos_[3]};
            robomas_pub_->publish(hand_msg);
        }
    }

    bool hand_reached() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < 3; ++i) {
            if (std::fabs(now_joint_[i] - target_hand_pos_[i]) > kPosTolerance_) return false;
        }
        return true;
    }

    void next_step(int& step, rclcpp::Time& start_time) {
        step++;
        start_time = this->now();
    }

    double elapsed_sec(const rclcpp::Time& start_time) {
        return (this->now() - start_time).seconds();
    }

    void stop_all() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_hand_pos_ = now_joint_;
        publish_all();
    }

    void joint_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            now_joint_ = {msg->data[0], msg->data[1], msg->data[2],msg->data[3]};
            joint_subscribe_flag_ = true;
        }
    }


    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Server<NakamuraHand>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robomas_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr holder_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr holder_sub_;

    std::mutex data_mutex_;
    std::vector<double> now_joint_ = {0.0, 0.0, 0.0,0.0};
    std::vector<double> target_hand_pos_ = {0.0, 0.0, 0.0,0.0};
    double now_holder_value_ = 0.0;


    bool joint_subscribe_flag_ = false;
    double kPosTolerance_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NakamuraHandServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}