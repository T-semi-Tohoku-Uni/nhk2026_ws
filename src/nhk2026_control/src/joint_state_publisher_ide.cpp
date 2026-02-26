#include "joint_state_publisher_ide.hpp"

JointStatePublisherIde::JointStatePublisherIde()
: rclcpp::Node("joint_state_publisher_ide")
{

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<JointStatePublisherIde> node = std::make_shared<JointStatePublisherIde>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}