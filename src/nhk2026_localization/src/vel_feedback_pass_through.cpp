#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace simulation {

    class VelFeedbackPassThrough : public rclcpp::Node {
        public:
            explicit VelFeedbackPassThrough(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("vel_feedback_pass_through", options) {
                
                // QoS設定（実機や他のノードに合わせる）
                rclcpp::QoS qos(rclcpp::KeepLast(10));

                // Publisher
                pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_feedback", qos);
                
                // Subscriber
                sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel", qos, 
                    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                        // 受け取ったメッセージをそのままPublish
                        pub_->publish(*msg);
                    }
                );
            }

        private:
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simulation::VelFeedbackPassThrough>());
    rclcpp::shutdown();
    return 0;
}