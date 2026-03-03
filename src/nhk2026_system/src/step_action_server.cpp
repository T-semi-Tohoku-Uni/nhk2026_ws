#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nhk2026_msgs/action/leg_move.hpp" 
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2;

class StepActionServer : public rclcpp::Node {
public:
    using LegMove = nhk2026_msgs::action::LegMove;
    using GoalHandleLegMove = rclcpp_action::ServerGoalHandle<LegMove>;

    StepActionServer() : Node("step_leg_sequencer") { 
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        this->action_server_ = rclcpp_action::create_server<LegMove>(
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

        robomas_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("leg_robomas", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const LegMove::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLegMove>) {
        RCLCPP_INFO(this->get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLegMove> goal_handle) {
        std::thread{std::bind(&StepActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLegMove> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "=== 段超えシーケンス開始 ===");
        auto result = std::make_shared<LegMove::Result>();

        // 各手順
        if (!send_leg_goal_sync({3.14, 3.14, 0.0})) { abort_action(goal_handle); return; }
        if (!send_leg_goal_sync({4.57, 4.57, -1.57})) { abort_action(goal_handle); return; }

        publish_robomas_for_duration(10.0, 2.0);
        publish_robomas_for_duration(0.0, 0.5);

        if (!send_leg_goal_sync({6.28, 6.28, -1.57})) { abort_action(goal_handle); return; }

        publish_cmd_vel_for_duration(0.5, 0.0, 5.0);
        publish_cmd_vel_for_duration(0.0, 0.0, 0.5);

        if (!send_leg_goal_sync({6.28, 6.28, 0.0})) { abort_action(goal_handle); return; }

        result->success = true;
        result->msg = "StepLeg Completed!";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "=== 段超えシーケンス正常終了 ===");
    }

    bool send_leg_goal_sync(std::vector<double> positions) {
        if (!leg_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "LegMove Action Server not available");
            return false;
        }

        auto goal_msg = LegMove::Goal();
        goal_msg.joint_states.position = positions;
        goal_msg.max_speed = 1.0;
        goal_msg.max_acc = 1.0;

        auto goal_handle_future = leg_client_->async_send_goal(goal_msg);
        
        // 2. spin_until_future_complete を削除し、直感的な待機に変更
        auto goal_handle = goal_handle_future.get(); 
        if (!goal_handle) return false;

        auto result_future = leg_client_->async_get_result(goal_handle);
        auto wrapped_result = result_future.get();

        return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
    }

    void publish_robomas_for_duration(float value, double duration_sec) {
        auto start_time = this->now();
        rclcpp::Rate loop_rate(20);
        while (rclcpp::ok() && (this->now() - start_time).seconds() < duration_sec) {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {value}; 
            robomas_pub_->publish(msg);
            loop_rate.sleep();
        }
    }

    void publish_cmd_vel_for_duration(double linear_y, double angular_z, double duration_sec) {
        auto start_time = this->now();
        rclcpp::Rate loop_rate(20);
        while (rclcpp::ok() && (this->now() - start_time).seconds() < duration_sec) {
            geometry_msgs::msg::Twist msg;
            msg.linear.y = linear_y; 
            msg.angular.z = angular_z;
            cmd_vel_pub_->publish(msg);
            loop_rate.sleep();
        }
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    }

    void abort_action(const std::shared_ptr<GoalHandleLegMove> goal_handle) {
        auto result = std::make_shared<LegMove::Result>();
        result->success = false;
        result->msg = "Action Failed during nested step";
        goal_handle->abort(result);
    }
       
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Server<LegMove>::SharedPtr action_server_;
    rclcpp_action::Client<LegMove>::SharedPtr leg_client_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robomas_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
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