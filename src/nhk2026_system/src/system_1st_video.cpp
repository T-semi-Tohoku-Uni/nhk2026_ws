#include "system_1st_video.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // std::shared_ptr<CanBridgenhk2026> node = std::make_shared<CanBridgenhk2026>();
    // rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}