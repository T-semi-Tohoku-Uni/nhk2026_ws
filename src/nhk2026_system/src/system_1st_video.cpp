#include "system_1st_video.hpp"

using std::placeholders::_1;

System1stVideo::System1stVideo()
: rclcpp_lifecycle::LifecycleNode(std::string("syste,_1st_video"))
{
    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&System1stVideo::parameters_callback, this, _1)
    );
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // std::shared_ptr<CanBridgenhk2026> node = std::make_shared<CanBridgenhk2026>();
    // rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}