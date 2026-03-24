#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nhk2026_msgs/action/step_move.hpp"
#include "nhk2026_msgs/action/takano_hand.hpp" // TakanoHand用に追加

class JoyControllerNode : public rclcpp::Node {
public:
    using StepMove = nhk2026_msgs::action::StepMove;
    using GoalHandleStepMove = rclcpp_action::ClientGoalHandle<StepMove>;
    using TakanoHand = nhk2026_msgs::action::TakanoHand; // 追加
    using GoalHandleTakanoHand = rclcpp_action::ClientGoalHandle<TakanoHand>; // 追加

    JoyControllerNode() : Node("joy_controller_node") {
        this->vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // アクションクライアントの設定
        this->step_client_ = rclcpp_action::create_client<StepMove>(this, "/step_leg_sequence");
        this->hand_client_ = rclcpp_action::create_client<TakanoHand>(this, "takano_hand_sequence"); //

        this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started.");
        RCLCPP_INFO(this->get_logger(), "L-Stick: Move, Circle: Step Up, Cross: Step Down, Triangle: Takano Hand");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        bool circle_pressed = msg->buttons[1];   // 〇
        bool cross_pressed = msg->buttons[0];    // ×
        bool triangle_pressed = msg->buttons[2]; // △

        // --- TakanoHand アクション開始判定 (非ブロッキング) ---
        if (triangle_pressed && !prev_triangle_ && !is_hand_busy_) {
            send_hand_goal(7); // ステップ数5で実行
        }

        // --- StepMove アクション中（足回り自動制御中）の処理 ---
        if (is_step_busy_) {
            prev_circle_ = circle_pressed;
            prev_cross_ = cross_pressed;
            prev_triangle_ = triangle_pressed;
            return; 
        }

        // --- StepMove 開始判定 ---
        if (circle_pressed && !prev_circle_) {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            send_step_goal("step up");
        } else if (cross_pressed && !prev_cross_) {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            send_step_goal("step down");
        } else {
            // --- 通常時の速度出力 (StepMove中でなければ常にパブリッシュ) ---
            geometry_msgs::msg::Twist twist;
            twist.linear.x = msg->axes[1] * 2.0;  
            twist.linear.y = msg->axes[0] * 2.0;  
            twist.angular.z = msg->axes[3] * 1.0; 
            vel_pub_->publish(twist);
        }

        prev_circle_ = circle_pressed;
        prev_cross_ = cross_pressed;
        prev_triangle_ = triangle_pressed;
    }

    void send_step_goal(const std::string & command) {
        if (!this->step_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Step Action Server not available");
            return;
        }

        is_step_busy_ = true;
        auto goal_msg = StepMove::Goal();
        goal_msg.msg = command;

        auto send_goal_options = rclcpp_action::Client<StepMove>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleStepMove::WrappedResult & result) {
            is_step_busy_ = false;
            RCLCPP_INFO(this->get_logger(), "Step sequence finished.");
        };
        this->step_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void send_hand_goal(int count) {
        if (!this->hand_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Takano Hand Server not available");
            return;
        }

        is_hand_busy_ = true; //
        auto goal_msg = TakanoHand::Goal();
        goal_msg.count = count;

        auto send_goal_options = rclcpp_action::Client<TakanoHand>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleTakanoHand::WrappedResult & result) {
            is_hand_busy_ = false;
            RCLCPP_INFO(this->get_logger(), "Takano Hand sequence finished.");
        };
        this->hand_client_->async_send_goal(goal_msg, send_goal_options); //
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<StepMove>::SharedPtr step_client_;
    rclcpp_action::Client<TakanoHand>::SharedPtr hand_client_; //

    bool is_step_busy_ = false;
    bool is_hand_busy_ = false; //
    bool prev_circle_ = false;
    bool prev_cross_ = false;
    bool prev_triangle_ = false; //
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}