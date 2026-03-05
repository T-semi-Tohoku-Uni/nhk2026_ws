#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nhk2026_msgs/action/step_move.hpp" 
#include "nhk2026_msgs/action/leg_move.hpp" 
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2;

class StepActionServer : public rclcpp::Node {
public:
    using StepMove = nhk2026_msgs::action::StepMove;
    using LegMove = nhk2026_msgs::action::LegMove;
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

        this->leg_client_ = rclcpp_action::create_client<LegMove>(
            this, "step_leg", callback_group_
        );

        robomas_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/leg_robomas", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        count = 0;
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const StepMove::Goal> goal) {
        (void)goal; 
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleStepMove>) {
        RCLCPP_INFO(this->get_logger(), "=== キャンセルリクエストを受信 ===");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleStepMove> goal_handle) {
        std::thread{std::bind(&StepActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleStepMove> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<StepMove::Result>();

        if(goal->msg == "step up" && count < 2) {
            RCLCPP_INFO(this->get_logger(), "=== 段上りシーケンス開始 (count: %d) ===", count);
            
            // 各関数が false を返した場合は、内部ですでに中断処理(canceled/abort)が呼ばれているのでそのまま抜ける
            if (!send_leg_goal_sync({3.14 + count * 6.28, 3.14 + count * 6.28, 0.0}, goal_handle)) return;
            if (!send_leg_goal_sync({4.57 + count * 6.28, 4.57 + count * 6.28, -1.57}, goal_handle)) return;
            
            if (!publish_robomas_for_duration(10.0f, 2.0, goal_handle)) return;
            if (!publish_robomas_for_duration(0.0f, 0.5, goal_handle)) return;
            
            if (!send_leg_goal_sync({6.1 + count * 6.28, 6.1 + count * 6.28, -1.57}, goal_handle)) return;
            
            if (!publish_cmd_vel_for_duration(0.5, 0.0, 5.0, goal_handle)) return;
            if (!publish_cmd_vel_for_duration(0.0, 0.0, 0.5, goal_handle)) return;
            
            if (!send_leg_goal_sync({6.1 + count * 6.28, 6.1 + count * 6.28, 0.0}, goal_handle)) return;

            count++; 
            result->success = true;
            result->msg = "Step up Completed!";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "=== 段上りシーケンス正常終了 ===");

        } else if(goal->msg == "step down" && count > 0) {
            RCLCPP_INFO(this->get_logger(), "=== 段降りシーケンス開始 (count: %d) ===", count);
            
            if (!send_leg_goal_sync({6.1 + count * 6.28, 6.1 + count * 6.28, 0.0}, goal_handle)) return;
            if (!publish_cmd_vel_for_duration(-0.5, 0.0, 5.0, goal_handle)) return;
            if (!send_leg_goal_sync({6.1 + count * 6.28, 6.1 + count * 6.28, -1.57}, goal_handle)) return;
            if (!publish_cmd_vel_for_duration(-0.5, 0.0, 5.0, goal_handle)) return;
            if (!send_leg_goal_sync({4.57 + count * 6.28, 4.57 + count * 6.28, -1.57}, goal_handle)) return;
            if (!publish_robomas_for_duration(10.0f, 2.0, goal_handle)) return;
            if (!send_leg_goal_sync({3.14 + count * 6.28, 3.14 + count * 6.28, 0.0}, goal_handle)) return;

            count--;
            result->success = true;
            result->msg = "Step down Completed!";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "=== 段降りシーケンス正常終了 ===");
            
        } else {
            result->success = false;
            result->msg = "Invalid request or count limit";
            goal_handle->abort(result);
        }
    }

    // --- ヘルパー関数 ---

    bool send_leg_goal_sync(std::vector<double> positions, const std::shared_ptr<GoalHandleStepMove>& goal_handle) {
        if (!leg_client_->wait_for_action_server(5s)) {
            abort_action(goal_handle, "LegMove Action Server not available");
            return false;
        }

        auto goal_msg = LegMove::Goal();
        goal_msg.joint_states.position = positions;
        goal_msg.max_speed = 1.0f;
        goal_msg.max_acc = 1.0f;

        auto goal_handle_future = leg_client_->async_send_goal(goal_msg);
        
        // ゴール受付完了を待つ
        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            abort_action(goal_handle, "LegMove goal rejected or timed out");
            return false;
        }
        auto child_goal_handle = goal_handle_future.get(); 
        if (!child_goal_handle) {
            abort_action(goal_handle, "LegMove goal was rejected by server");
            return false;
        }

        auto result_future = leg_client_->async_get_result(child_goal_handle);
        
        // ★ キャンセル監視付きの待機ループ ★
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "中断: 下位のLegMoveアクションもキャンセルします");
                leg_client_->async_cancel_goal(child_goal_handle); // 下請けのアクションもキャンセル！
                cancel_action(goal_handle, "Canceled during LegMove");
                return false;
            }
            // 50msごとに完了したかチェック
            if (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
                break;
            }
        }

        auto wrapped_result = result_future.get();
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            abort_action(goal_handle, "LegMove failed");
            return false;
        }
        return true;
    }

    bool publish_robomas_for_duration(float value, double duration_sec, const std::shared_ptr<GoalHandleStepMove>& goal_handle) {
        auto start_time = this->now();
        rclcpp::Rate loop_rate(20);
        while (rclcpp::ok() && (this->now() - start_time).seconds() < duration_sec) {
            if (goal_handle->is_canceling()) {
                cancel_action(goal_handle, "Canceled during robomas publish");
                return false;
            }
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {value}; 
            robomas_pub_->publish(msg);
            loop_rate.sleep();
        }
        return true;
    }

    bool publish_cmd_vel_for_duration(double linear_y, double angular_z, double duration_sec, const std::shared_ptr<GoalHandleStepMove>& goal_handle) {
        auto start_time = this->now();
        rclcpp::Rate loop_rate(20);
        while (rclcpp::ok() && (this->now() - start_time).seconds() < duration_sec) {
            if (goal_handle->is_canceling()) {
                cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); // ★中断時は安全のためにすぐ停止！
                cancel_action(goal_handle, "Canceled during cmd_vel publish");
                return false;
            }
            geometry_msgs::msg::Twist msg;
            msg.linear.y = linear_y; 
            msg.angular.z = angular_z;
            cmd_vel_pub_->publish(msg);
            loop_rate.sleep();
        }
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        return true;
    }

    // エラー終了用
    void abort_action(const std::shared_ptr<GoalHandleStepMove>& goal_handle, const std::string& msg = "Action Failed") {
        auto result = std::make_shared<StepMove::Result>();
        result->success = false;
        result->msg = msg;
        goal_handle->abort(result);
    }

    // キャンセル終了用
    void cancel_action(const std::shared_ptr<GoalHandleStepMove>& goal_handle, const std::string& msg = "Action Canceled") {
        auto result = std::make_shared<StepMove::Result>();
        result->success = false;
        result->msg = msg;
        goal_handle->canceled(result);
    }
       
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Server<StepMove>::SharedPtr action_server_;
    rclcpp_action::Client<LegMove>::SharedPtr leg_client_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robomas_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    int count; 
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