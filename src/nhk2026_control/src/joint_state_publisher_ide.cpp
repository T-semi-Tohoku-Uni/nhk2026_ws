#include "joint_state_publisher_ide.hpp"

JointStatePublisherIde::JointStatePublisherIde()
{

}

JointStatePublisherIde::~JointStatePublisherIde()
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