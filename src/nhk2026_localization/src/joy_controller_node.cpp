#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nhk2026_msgs/action/step_move.hpp"

class JoyControllerNode : public rclcpp::Node {
public:
    using StepMove = nhk2026_msgs::action::StepMove;
    using GoalHandleStepMove = rclcpp_action::ClientGoalHandle<StepMove>;

    JoyControllerNode() : Node("joy_controller_node") {
        // --- パブリッシャーの設定 ---
        // cmd_velをパブリッシュして足回りを動かす
        this->vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // --- アクションクライアントの設定 ---
        // step_action_server.cpp で定義されたサーバーを呼び出す
        this->action_client_ = rclcpp_action::create_client<StepMove>(this, "step_leg_sequence");

        // --- サブスクライバーの設定 ---
        this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started. (L-Stick: Move, Circle: Step Up, Cross: Step Down)");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // 1. 段差昇降アクションのトリガーチェック (○ボタン: index 1, ×ボタン: index 0)
        bool circle_pressed = msg->buttons[1];
        bool cross_pressed = msg->buttons[0];

        if (!is_action_busy_) {
            if (circle_pressed && !prev_circle_) {
                send_step_goal("step up");
            } else if (cross_pressed && !prev_cross_) {
                send_step_goal("step down");
            }
        }

        // 2. 速度(vel)の計算と出力
        // アクション実行中(is_action_busy_ == true)は強制的に速度を0にする
        geometry_msgs::msg::Twist twist;
        if (!is_action_busy_) {
            // joy2Vel のロジックを参考
            twist.linear.x = msg->axes[1] * 2.0;  // 前後
            twist.linear.y = msg->axes[0] * 2.0;  // 左右
            twist.angular.z = msg->axes[3] * 1.0; // 旋回
            vel_pub_->publish(twist);
        } else {
            // アクション中は全項目 0.0 (停止)
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.angular.z = 0.0;
        }
        

        // ボタン状態の保存（チャタリング防止）
        prev_circle_ = circle_pressed;
        prev_cross_ = cross_pressed;
    }

    void send_step_goal(const std::string & command) {
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Step Action Server not available");
            return;
        }

        is_action_busy_ = true; // アクション開始フラグを立てる
        auto goal_msg = StepMove::Goal();
        goal_msg.msg = command;

        auto send_goal_options = rclcpp_action::Client<StepMove>::SendGoalOptions();
        
        // アクション完了時のコールバックを登録
        send_goal_options.result_callback = [this](const GoalHandleStepMove::WrappedResult & result) {
            is_action_busy_ = false; // アクションが終わったらフラグを下ろす
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Step sequence completed.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Step sequence failed or canceled.");
            }
        };

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<StepMove>::SharedPtr action_client_;

    bool is_action_busy_ = false; // アクション実行中かどうか
    bool prev_circle_ = false;
    bool prev_cross_ = false;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}