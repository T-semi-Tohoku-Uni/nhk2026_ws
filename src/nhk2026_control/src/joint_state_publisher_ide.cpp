#include "joint_state_publisher_ide.hpp"

using std::placeholders::_1;

JointStatePublisherIde::JointStatePublisherIde()
: rclcpp::Node("joint_state_publisher_ide")
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_state",
        device
    );
    this->motor_feedback_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "motor_feedback",
        device,
        std::bind(&JointStatePublisherIde::motor_callback, this, _1)
    );
}

void JointStatePublisherIde::motor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr rxdata)
{
    sensor_msgs::msg::JointState txdata;
    txdata.name = {"joint1", "joint2", "joint3"};
    txdata.position = {
        rxdata->data[0],
        rxdata->data[1],
        rxdata->data[2]
    };
    txdata.header.stamp = this->get_clock()->now();

    this->joint_state_publisher_->publish(txdata);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<JointStatePublisherIde> node = std::make_shared<JointStatePublisherIde>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}