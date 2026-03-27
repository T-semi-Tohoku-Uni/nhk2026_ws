#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp" // 追加
#include "nhk2026_msgs/action/step_move.hpp"
#include "nhk2026_msgs/action/takano_hand.hpp"

class JoyControllerNode : public rclcpp::Node {
public:
    using StepMove = nhk2026_msgs::action::StepMove;
    using GoalHandleStepMove = rclcpp_action::ClientGoalHandle<StepMove>;
    using TakanoHand = nhk2026_msgs::action::TakanoHand;
    using GoalHandleTakanoHand = rclcpp_action::ClientGoalHandle<TakanoHand>;

    JoyControllerNode() : Node("joy_controller_node") {
        this->vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // vacuumトピックのパブリッシャー
        this->vacuum_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("vacuum", 10);

        this->step_client_ = rclcpp_action::create_client<StepMove>(this, "/step_leg_sequence");
        this->hand_client_ = rclcpp_action::create_client<TakanoHand>(this, "takano_hand_sequence");

        this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // ボタン割り当て（一般的なコントローラ配置を想定）
        bool cross_pressed    = msg->buttons[0]; // ×
        bool circle_pressed   = msg->buttons[1]; // ○
        bool triangle_pressed = msg->buttons[2]; // △
        bool square_pressed   = msg->buttons[3]; // □ (真空吸着用)

        // --- 真空吸着（Vacuum）制御：□ボタン ---
        auto vacuum_msg = std_msgs::msg::Int32MultiArray();
        if (square_pressed) {
            vacuum_msg.data.push_back(800);
        } else {
            vacuum_msg.data.push_back(0);
        }
        vacuum_pub_->publish(vacuum_msg);

        // --- TakanoHand 1ステップ実行：△ボタン ---
        if (triangle_pressed && !prev_triangle_ && !is_hand_busy_) {
            send_hand_goal(current_hand_step_, current_hand_step_, 5.0f);
        }

        // 足のステップ動作中（Action実行中）は移動入力を受け付けない
        if (is_step_busy_) {
            update_prev_buttons(circle_pressed, cross_pressed, triangle_pressed, square_pressed);
            return; 
        }

        // --- 足のステップ動作 / 移動制御 ---
        if (circle_pressed && !prev_circle_) {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            send_step_goal("step up");
        } 
        else if (cross_pressed && !prev_cross_) {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            send_step_goal("step down");
        } 
        else {
            // 通常移動
            geometry_msgs::msg::Twist twist;
            twist.linear.x = msg->axes[1] * 2.0;  
            twist.linear.y = msg->axes[0] * 2.0;  
            twist.angular.z = msg->axes[3] * 1.0; 
            vel_pub_->publish(twist);
        }

        update_prev_buttons(circle_pressed, cross_pressed, triangle_pressed, square_pressed);
    }

    void update_prev_buttons(bool circle, bool cross, bool triangle, bool square) {
        prev_circle_ = circle;
        prev_cross_ = cross;
        prev_triangle_ = triangle;
        prev_square_ = square;
    }

    // --- Action送信関数 (変更なし) ---
    void send_hand_goal(int first, int final, float pos) {
        if (!this->hand_client_->wait_for_action_server(std::chrono::seconds(2))) return;
        is_hand_busy_ = true;
        auto goal_msg = TakanoHand::Goal();
        goal_msg.firststep = first;
        goal_msg.finalstep = final;
        goal_msg.pos = pos;
        auto opts = rclcpp_action::Client<TakanoHand>::SendGoalOptions();
        opts.result_callback = [this](const GoalHandleTakanoHand::WrappedResult & result) {
            is_hand_busy_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                current_hand_step_ = (current_hand_step_ >= 6) ? 0 : current_hand_step_ + 1;
            }
        };
        this->hand_client_->async_send_goal(goal_msg, opts);
    }

    void send_step_goal(const std::string & command) {
        if (!this->step_client_->wait_for_action_server(std::chrono::seconds(5))) return;
        is_step_busy_ = true;
        auto goal_msg = StepMove::Goal();
        goal_msg.msg = command;
        auto opts = rclcpp_action::Client<StepMove>::SendGoalOptions();
        opts.result_callback = [this](const GoalHandleStepMove::WrappedResult &) { is_step_busy_ = false; };
        this->step_client_->async_send_goal(goal_msg, opts);
    }

    // --- 変数定義 ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vacuum_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<StepMove>::SharedPtr step_client_;
    rclcpp_action::Client<TakanoHand>::SharedPtr hand_client_;

    bool is_step_busy_ = false;
    bool is_hand_busy_ = false;
    bool prev_circle_ = false;
    bool prev_cross_ = false;
    bool prev_triangle_ = false;
    bool prev_square_ = false; // □ボタン用
    int current_hand_step_ = 0; 
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}