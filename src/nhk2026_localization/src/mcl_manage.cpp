#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace mcl_manage {

class MclManage : public rclcpp::Node {
    public:
        explicit MclManage(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("mcl_manage_node", options), current_zaxis_level_(0), has_pose_(false) {
            
            pub_mcl_select_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("mcl_select", 10);
            pub_initial_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("initial_pose", 10);
            
            sub_zaxis_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
                "zaxis", 10, std::bind(&MclManage::zaxisCallback, this, std::placeholders::_1)
            );
            
            sub_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "pose", 10, std::bind(&MclManage::poseCallback, this, std::placeholders::_1)
            );

            // 1. タイマーの初期化 (例: 100ms = 10Hz でパブリッシュ)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&MclManage::timerCallback, this)
            );

            RCLCPP_INFO(this->get_logger(), "MCL Manage Node initialized. Default Z-axis level: 0");
        }

    private:
        // 2. タイマーから呼ばれるコールバック関数
        void timerCallback() {
            publishMclSelect(current_zaxis_level_);
        }

        void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
            current_pose_ = *msg;
            has_pose_ = true;
        }

        void zaxisCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            if (msg->data.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty array on /zaxis topic.");
                return;
            }
            int new_level = msg->data[0];

            if(new_level == 10){
                if(init_flag != 10){
                    if(current_zaxis_level_ == 0){
                        if (has_pose_) {
                            geometry_msgs::msg::Pose new_initial_pose = current_pose_;
                            
                            RCLCPP_INFO(this->get_logger(), "Publishing new initial_pose for node switch: x=%.2f, y=%.2f, z=%.2f", 
                            new_initial_pose.position.x, new_initial_pose.position.y, new_initial_pose.position.z);
                            
                            pub_initial_pose_->publish(new_initial_pose);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Z-axis");
                        }
                    }
                    init_flag = 10;
                }

                return;
            }
            // 3. 値が変化した時だけリセット処理（initial_pose発行）を行う
            if (new_level != current_zaxis_level_) {
                RCLCPP_INFO(this->get_logger(), "Z-axis level changed: %d -> %d", current_zaxis_level_, new_level);
                init_flag  = new_level;
                if (has_pose_) {
                    geometry_msgs::msg::Pose new_initial_pose = current_pose_;
                    
                    if (new_level == 0) {
                        new_initial_pose.position.z = 0.0;
                    } else if (new_level == 1) {
                        new_initial_pose.position.z = 0.20;
                    } else if (new_level == 2) {
                        new_initial_pose.position.z = 0.40;
                    }

                    RCLCPP_INFO(this->get_logger(), "Publishing new initial_pose for node switch: x=%.2f, y=%.2f, z=%.2f", 
                                new_initial_pose.position.x, new_initial_pose.position.y, new_initial_pose.position.z);
                    
                    pub_initial_pose_->publish(new_initial_pose);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Z-axis level changed, but no pose received yet. Cannot initialize position.");
                }

                current_zaxis_level_ = new_level;
            }
        }

        void publishMclSelect(int level) {
            std_msgs::msg::Int32MultiArray select_msg;
            select_msg.data.push_back(level);
            pub_mcl_select_->publish(select_msg);
        }

        int current_zaxis_level_;
        int init_flag = 0;
        bool has_pose_;
        geometry_msgs::msg::Pose current_pose_;

        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_mcl_select_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_initial_pose_;
        
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_zaxis_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;

        // 4. タイマー用変数
        rclcpp::TimerBase::SharedPtr timer_;
    };

} 

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl_manage::MclManage>());
    rclcpp::shutdown();
    return 0;
}