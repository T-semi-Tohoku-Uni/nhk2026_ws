#include "nhk2026_control/ide_arm_sim_bridge.hpp"

IdeArmSimBridge::IdeArmSimBridge()
: rclcpp::Node("ide_arm_sim_bridge")
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->joint_control_publisher_.reserve(3);
    this->control_subscriber_.reserve(3);

    for (int i = 0; i < 3; ++i) {
        const int topic_index = i + 1;

        this->joint_control_publisher_.push_back(
            this->create_publisher<std_msgs::msg::Float64>(
                "joint_sim" + std::to_string(topic_index),
                device
            )
        );

        this->control_subscriber_.push_back(
            this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "j" + std::to_string(topic_index),
                device,
                [this, i](const std_msgs::msg::Float32MultiArray::SharedPtr rxdata) {
                    this->joint_array_control_callback(rxdata, i);
                }
            )
        );
    }
}

void IdeArmSimBridge::joint_array_control_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr rxdata,
    int index
)
{
    if (index < 0 || static_cast<std::size_t>(index) >= this->joint_control_publisher_.size() || rxdata->data.empty()) {
        return;
    }

    std_msgs::msg::Float64 txdata;
    txdata.data = rxdata->data[0];
    this->joint_control_publisher_[index]->publish(txdata);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<IdeArmSimBridge> node = std::make_shared<IdeArmSimBridge>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}