#include "nhk2026_control/vacuum_service.hpp"

VacuumServer::VacuumServer()
: rclcpp::Node(std::string("vacuum_server"))
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->vacuum_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "vacuum_device",
        device
    );

    this->server_ = this->create_service<std_srvs::srv::SetBool>(
        "vacuum",
        std::bind(&VacuumServer::server_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    using namespace std::chrono_literals;
    this->publish_timer_ = this->create_wall_timer(
        10ms,
        std::bind(&VacuumServer::publisher_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "create vacuum server");
}

void VacuumServer::server_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
    this->vacuum_on_ = request->data;

    response->message = "vacuum on";
    response->success = true;
}

void VacuumServer::publisher_timer_callback()
{
    std_msgs::msg::Float32MultiArray txdata;
    if (this->vacuum_on_)
    {
        txdata.data = {500};
    }
    else
    {
        txdata.data = {0};
    }

    this->vacuum_publisher_->publish(txdata);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<VacuumServer> node = std::make_shared<VacuumServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}