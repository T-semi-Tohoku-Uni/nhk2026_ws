#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

        this->step_client_ = rclcpp_action::create_client<StepMove>(this, "/step_leg_sequence");
        this->hand_client_ = rclcpp_action::create_client<TakanoHand>(this, "takano_hand_sequence");

        this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started.");
        RCLCPP_INFO(this->get_logger(), "Triangle: Execute next Takano Hand step (Current: 0)");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        bool circle_pressed = msg->buttons[1];
        bool cross_pressed = msg->buttons[0];
        bool triangle_pressed = msg->buttons[2];

        // --- TakanoHand 1ステップ実行 ---
        if (triangle_pressed && !prev_triangle_ && !is_hand_busy_) {
            // 現在のステップのみを実行し、pos（例として1.0）を渡す
            send_hand_goal(current_hand_step_, current_hand_step_, 5.0f);
        }

        if (is_step_busy_) {
            update_prev_buttons(circle_pressed, cross_pressed, triangle_pressed);
            return; 
        }

        if (circle_pressed && !prev_circle_) {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            send_step_goal("step up");
        } else if (cross_pressed && !prev_cross_) {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            send_step_goal("step down");
        } else {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = msg->axes[1] * 2.0;  
            twist.linear.y = msg->axes[0] * 2.0;  
            twist.angular.z = msg->axes[3] * 1.0; 
            vel_pub_->publish(twist);
        }

        update_prev_buttons(circle_pressed, cross_pressed, triangle_pressed);
    }

    void update_prev_buttons(bool circle, bool cross, bool triangle) {
        prev_circle_ = circle;
        prev_cross_ = cross;
        prev_triangle_ = triangle;
    }

    void send_hand_goal(int first, int final, float pos) {
        if (!this->hand_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Takano Hand Server not available");
            return;
        }

        is_hand_busy_ = true;
        auto goal_msg = TakanoHand::Goal();
        goal_msg.firststep = first;
        goal_msg.finalstep = final;
        goal_msg.pos = pos;

        RCLCPP_INFO(this->get_logger(), "Sending goal: Step %d", first);

        auto send_goal_options = rclcpp_action::Client<TakanoHand>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const GoalHandleTakanoHand::WrappedResult & result) {
            is_hand_busy_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Step %d finished successfully.", current_hand_step_);
                
                // 次のステップへ（最大5まで。5を超えたら0に戻る例）
                current_hand_step_++;
                if (current_hand_step_ > 6) {
                    current_hand_step_ = 0;
                    RCLCPP_INFO(this->get_logger(), "Sequence reset to Step 0.");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Step failed. Current step remains at %d", current_hand_step_);
            }
        };

        this->hand_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // (send_step_goal 等、他の関数は変更なしのため省略)
    void send_step_goal(const std::string & command) {
        if (!this->step_client_->wait_for_action_server(std::chrono::seconds(5))) return;
        is_step_busy_ = true;
        auto goal_msg = StepMove::Goal();
        goal_msg.msg = command;
        auto opts = rclcpp_action::Client<StepMove>::SendGoalOptions();
        opts.result_callback = [this](const GoalHandleStepMove::WrappedResult &) { is_step_busy_ = false; };
        this->step_client_->async_send_goal(goal_msg, opts);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<StepMove>::SharedPtr step_client_;
    rclcpp_action::Client<TakanoHand>::SharedPtr hand_client_;

    bool is_step_busy_ = false;
    bool is_hand_busy_ = false;
    bool prev_circle_ = false;
    bool prev_cross_ = false;
    bool prev_triangle_ = false;

    // 現在のステップを保持する変数（追加）
    int current_hand_step_ = 0; 
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}