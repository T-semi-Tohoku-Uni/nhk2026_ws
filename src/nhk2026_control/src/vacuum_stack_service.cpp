#include "nhk2026_control/vacuum_stack_service.hpp"

VacuumStackServer::VacuumStackServer()
: rclcpp::Node(std::string("vacuum_stack_server"))
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->vacuum_stack_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "vacuum_stack_device",
        device
    );

    this->server_ = this->create_service<std_srvs::srv::SetBool>(
        "vacuum_stack",
        std::bind(&VacuumStackServer::server_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    using namespace std::chrono_literals;
    this->publish_timer_ = this->create_wall_timer(
        10ms,
        std::bind(&VacuumStackServer::publisher_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "create vacuum stack server");
}

void VacuumStackServer::server_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
    this->vacuum_stack_on_ = request->data;

    response->message = "vacuum stack on";
    response->success = true;
}

void VacuumStackServer::publisher_timer_callback()
{
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(this->vacuum_stack_on_ ? 1 : 0);
    this->vacuum_stack_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<VacuumStackServer> node = std::make_shared<VacuumStackServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}