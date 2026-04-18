#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

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

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MclManage::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "MCL Manage Node initialized.");
    }

private:
    // 最寄りの値を検索するヘルパー関数
    float findNearest(float value, const std::vector<float>& candidates) {
        if (candidates.empty()) return value;
        auto it = std::min_element(candidates.begin(), candidates.end(),
            [value](float a, float b) {
                return std::abs(a - value) < std::abs(b - value);
            });
        return *it;
    }

    // 補正してパブリッシュする共通処理
    void publishCorrectedPose(geometry_msgs::msg::Pose pose) {
        pose.position.x = findNearest(pose.position.x, initpose_x);
        pose.position.y = findNearest(pose.position.y, initpose_y);

        RCLCPP_INFO(this->get_logger(), "Publishing Corrected Pose: x=%.2f, y=%.2f, z=%.2f", 
                    pose.position.x, pose.position.y, pose.position.z);
        pub_initial_pose_->publish(pose);
    }

    void timerCallback() {
        publishMclSelect(current_zaxis_level_);
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
        has_pose_ = true;
    }

    void zaxisCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;
        int new_level = msg->data[0];

        // 特殊ステータス 10 の処理
        if(new_level == 10){
            if(init_flag != 10){
                if(current_zaxis_level_ == 0 && has_pose_){
                    // 現在のポーズ（Z=0想定）を補正して送信
                    publishCorrectedPose(current_pose_);
                }
                init_flag = 10;
            }
            return;
        }

        // 通常の階層切り替え (0, 1, 2...)
        if (new_level != current_zaxis_level_) {
            init_flag = new_level;
            if (has_pose_) {
                geometry_msgs::msg::Pose new_pose = current_pose_;
                
                // 切り替え先の階層に応じたZ座標の上書き
                if (new_level == 0)      new_pose.position.z = 0.0;
                else if (new_level == 1) new_pose.position.z = 0.20;
                else if (new_level == 2) new_pose.position.z = 0.40;

                publishCorrectedPose(new_pose);
            } else {
                RCLCPP_WARN(this->get_logger(), "Level changed but no pose available.");
            }
            current_zaxis_level_ = new_level;
        }
    }

    void publishMclSelect(int level) {
        std_msgs::msg::Int32MultiArray select_msg;
        select_msg.data.push_back(level);
        pub_mcl_select_->publish(select_msg);
    }

    // メンバ変数
    int current_zaxis_level_;
    int init_flag = 0;
    bool has_pose_;
    geometry_msgs::msg::Pose current_pose_;
    
    std::vector<float> initpose_x = {-1.8f, -3.0f, -4.2f};
    std::vector<float> initpose_y = {2.6f, 3.8f, 5.0f, 6.2f};

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_mcl_select_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_initial_pose_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_zaxis_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace mcl_manage

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl_manage::MclManage>());
    rclcpp::shutdown();
    return 0;
}