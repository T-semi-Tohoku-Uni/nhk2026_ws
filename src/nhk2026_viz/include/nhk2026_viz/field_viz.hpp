#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>

class MarkerPublisher
: public rclcpp::Node
{
public:
    MarkerPublisher();

private:
    void pub_timer_callback();
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};