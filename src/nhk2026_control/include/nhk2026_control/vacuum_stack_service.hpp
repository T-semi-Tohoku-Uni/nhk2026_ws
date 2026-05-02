#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class VacuumStackServer
: public rclcpp::Node
{
public:
    VacuumStackServer();

private:
    void server_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    );
    void publisher_timer_callback();

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vacuum_stack_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    bool vacuum_stack_on_{false};
};