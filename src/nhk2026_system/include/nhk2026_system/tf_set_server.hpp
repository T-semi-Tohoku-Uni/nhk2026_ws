#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nhk2026_msgs/srv/tf_set.hpp"

class TfSetServer
: public rclcpp::Node
{ 
public:
    TfSetServer();
private:
    void tf_set_callback(
        const std::shared_ptr<nhk2026_msgs::srv::TfSet::Request> request,
        std::shared_ptr<nhk2026_msgs::srv::TfSet::Response> response
    );

   rclcpp::Service<nhk2026_msgs::srv::TfSet>::SharedPtr tf_server_;
};
