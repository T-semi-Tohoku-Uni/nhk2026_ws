#include "tf_set_server.hpp"

TfSetServer::TfSetServer()
: rclcpp::Node("tf_set_server")
{
    this->tf_server_ = this->create_service<nhk2026_msgs::srv::TfSet>(
        "tf_set",
        std::bind(&TfSetServer::tf_set_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void TfSetServer::tf_set_callback(
    const std::shared_ptr<nhk2026_msgs::srv::TfSet::Request> request,
    std::shared_ptr<nhk2026_msgs::srv::TfSet::Response> response
)
{
    
}