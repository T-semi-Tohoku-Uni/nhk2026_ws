#include "nhk2026_control/joint_state_publisher_dansa.hpp"

using std::placeholders::_1;

JointStatePublisherDansa::JointStatePublisherDansa()
: rclcpp::Node("joint_state_publisher_dansa")
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states_dansa",
        device
    );
    this->motor_feedback_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "motor_dansa_feedback",
        device,
        std::bind(&JointStatePublisherDansa::motor_callback, this, _1)
    );
}

void JointStatePublisherDansa::motor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr rxdata)
{
    sensor_msgs::msg::JointState txdata;
    txdata.name = {"right_arm", "left_arm", "back_arm"};
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
    std::shared_ptr<JointStatePublisherDansa> node = std::make_shared<JointStatePublisherDansa>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}