#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nhk2026_msgs/action/kneko_arm.hpp" 

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class RobstrideRobomasActionServer : public rclcpp::Node {
public:
    using KnekoArm = nhk2026_msgs::action::KnekoArm;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<KnekoArm>;

    RobstrideRobomasActionServer() : Node("robstride_robomas_action_server") {
        rclcpp::QoS device_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // アクションサーバーの作成
        this->action_server_ = rclcpp_action::create_server<KnekoArm>(
            this,
            "robstride_robomas_move",
            std::bind(&RobstrideRobomasActionServer::handle_goal, this, _1, _2),
            std::bind(&RobstrideRobomasActionServer::handle_cancel, this, _1),
            std::bind(&RobstrideRobomasActionServer::handle_accepted, this, _1)
        );

        // 状態購読
        this->joint_states_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "kaneko_arm", // 必要に応じてトピック名を変更
            device_qos,
            std::bind(&RobstrideRobomasActionServer::joint_state_callback, this, _1)
        );

        // モーター指令用パブリッシャー
        this->robstride_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("robstride_angle", device_qos);
        this->robomas_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("robomas_position", device_qos);

        this->declare_parameter<double>("kPosTolerance", 0.05);
        this->kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();
    }

private:
    // ゴールの受付判断
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const KnekoArm::Goal> goal) {
        (void)goal;
        if (!this->joint_subscribe_flag_) {
            RCLCPP_ERROR(this->get_logger(), "Joint state not received yet");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Goal received");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // キャンセル処理
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 実行開始
    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
        std::thread{std::bind(&RobstrideRobomasActionServer::execute, this, _1), goal_handle}.detach();
    }

    // メイン実行ループ
    void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<KnekoArm::Feedback>();
        auto result = std::make_shared<KnekoArm::Result>();

        rclcpp::Rate loop_rate(50.0);
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->msg = "Goal canceled";
                goal_handle->canceled(result);
                return;
            }

            double robstride_diff = std::fabs(this->now_joint_.position[0] - goal->robstride_angle);
            double robomas_diff = std::fabs(this->now_joint_.position[1] - goal->robomas_position);

            std_msgs::msg::Float32MultiArray rs_msg;
            rs_msg.data = {goal->robstride_angle, goal->max_speed, goal->max_acc};
            robstride_pub_->publish(rs_msg);

            std_msgs::msg::Float32MultiArray rm_msg;
            rm_msg.data = {goal->robomas_position};
            robomas_pub_->publish(rm_msg);

            feedback->current.header.stamp = this->now();
            feedback->current.header.frame_id = "base_link";
            feedback->current.pose.position.x = this->now_joint_.position[0]; 
            feedback->pos_error = static_cast<float>(std::hypot(robstride_diff, robomas_diff));
            feedback->msg = "Moving to target...";
            goal_handle->publish_feedback(feedback);

            if (robstride_diff <= kPosTolerance_ && robomas_diff <= kPosTolerance_) {
                result->success = true;
                result->msg = "Reached target position";
                goal_handle->succeed(result);
                return;
            }

            loop_rate.sleep();
        }
    }

    void joint_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received joint state data is too short (size: %zu)", msg->data.size());
            return;
        }

        if (this->now_joint_.position.size() < 2) {
            this->now_joint_.position.resize(2);
        }

        this->now_joint_.position[0] = msg->data[0];
        this->now_joint_.position[1] = msg->data[1];

        this->joint_subscribe_flag_ = true;
    }

    // メンバ変数
    rclcpp_action::Server<KnekoArm>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_states_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robstride_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robomas_pub_;

    sensor_msgs::msg::JointState now_joint_;
    bool joint_subscribe_flag_ = false;
    double kPosTolerance_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobstrideRobomasActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}