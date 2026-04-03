#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nhk2026_msgs/action/nakamura_hand.hpp"

class JoyControllerNode : public rclcpp::Node {
public:
    using NakamuraHand = nhk2026_msgs::action::NakamuraHand;
    using GoalHandleNakamuraHand = rclcpp_action::ClientGoalHandle<NakamuraHand>;

    JoyControllerNode() : Node("joy_r1") {
        // パブリッシャーの設定
        this->vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        this->vacuum_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("vacuum", 10);
        this->kaneko_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("kaneko_arm", 10);
        
        // nakamura_vertical用のパブリッシャーを追加
        this->nakamura_vertical_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("nakamura_vertical", 10);

        this->hand_client_ = rclcpp_action::create_client<NakamuraHand>(this, "nakamura_hand_sequence");

        this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started. (Kaneko: D-pad, NakamuraVertical: RightStick-V)");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons.size() < 6 || msg->axes.size() < 8) return;

        bool l1_pressed       = msg->buttons[4];
        bool r1_pressed       = msg->buttons[5];
        bool triangle_pressed = msg->buttons[2];
        bool square_pressed   = msg->buttons[3];

        double dpad_h = msg->axes[6]; 
        double dpad_v = msg->axes[7];
        
        double rs_h = msg->axes[5];
        double rs_v = msg->axes[4];

        // 真空吸着
        auto vacuum_msg = std_msgs::msg::Int32MultiArray();
        vacuum_msg.data.push_back(square_pressed ? 800 : 0);
        vacuum_pub_->publish(vacuum_msg);

        // NakamuraHand シーケンス
        if (triangle_pressed && !prev_triangle_ && !is_hand_busy_) {
            send_hand_goal(current_hand_step_, current_hand_step_);
        }

        // 移動制御
        geometry_msgs::msg::Twist twist;
        twist.linear.x = msg->axes[1] * 2.0;
        twist.linear.y = msg->axes[0] * 2.0;
        twist.angular.z = l1_pressed ? 1.5 : (r1_pressed ? -1.5 : 0.0);
        vel_pub_->publish(twist);

        // 感度の設定
        const double sensitivity = 0.05;
        const double sen = 5.0;
        const double vertical_sensitivity = 10.0; 

        // kaneko_arm 制御
        kaneko_values_[0] += dpad_h * sensitivity;
        kaneko_values_[1] += dpad_v * sen;
        auto kaneko_msg = std_msgs::msg::Float32MultiArray();
        kaneko_msg.data = {static_cast<float>(kaneko_values_[0]), 0.0f, 0.0f, static_cast<float>(kaneko_values_[1])};
        kaneko_pub_->publish(kaneko_msg);

        // nakamura_vertical 制御 (修正済み)
        nakamura_vertical_value_[0] += rs_v * vertical_sensitivity;
        nakamura_vertical_value_[1] += rs_h * 0.1; // 横方向の感度

        auto nv_msg = std_msgs::msg::Float32MultiArray();
        // 修正: 個別にキャストして2つの要素を格納
        nv_msg.data = {
            static_cast<float>(nakamura_vertical_value_[0]), 
            static_cast<float>(nakamura_vertical_value_[1])
        };
        nakamura_vertical_pub_->publish(nv_msg);

        prev_triangle_ = triangle_pressed;
    }

    void send_hand_goal(int first, int final) {
        if (!this->hand_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }
        is_hand_busy_ = true;
        auto goal_msg = NakamuraHand::Goal();
        goal_msg.firststep = first;
        goal_msg.finalstep = final;

        auto opts = rclcpp_action::Client<NakamuraHand>::SendGoalOptions();
        opts.result_callback = [this](const GoalHandleNakamuraHand::WrappedResult & result) {
            is_hand_busy_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                current_hand_step_ = (current_hand_step_ >= 4) ? 0 : current_hand_step_ + 1;
            }
        };
        this->hand_client_->async_send_goal(goal_msg, opts);
    }

    // パブリッシャー・クライアント
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vacuum_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr kaneko_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr nakamura_vertical_pub_; // 追加
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp_action::Client<NakamuraHand>::SharedPtr hand_client_;

    // 内部状態
    bool is_hand_busy_ = false;
    bool prev_triangle_ = false;
    int current_hand_step_ = 0;
    std::vector<double> kaneko_values_ = {0.0, 0.0}; 
    std::vector<double> nakamura_vertical_value_ = {0.0, 0.0}; // 追加
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}