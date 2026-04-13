#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nhk2026_msgs/srv/tf_set.hpp"

class TfSetServer
: public rclcpp::Node
{ 
public:
    TfSetServer();
    ~TfSetServer();
private:
   rclcpp::Service<nhk2026_msgs::srv::TfSet>::SharedPtr tf_server_;
};
