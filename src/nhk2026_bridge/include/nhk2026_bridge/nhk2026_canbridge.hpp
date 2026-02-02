#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "ros2can_bridge.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include <atomic>
#include <thread>

class CanBridgenhk2026
: public rclcpp_lifecycle::LifecycleNode
{
public:
    CanBridgenhk2026();
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

private:
    const char* Ifname;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters
    );

    void rx_loop();

    std::vector<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> float_subscribers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr> int_subscribers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr> bytes_subscribers_;
    
    std::vector<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr> float_publisher_;
    std::vector<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32MultiArray>::SharedPtr> int_publisher_;
    std::vector<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::ByteMultiArray>::SharedPtr> bytes_publisher_;

    std::vector<std::string> pub_float_bridge_topic_list_;
    std::vector<std::string> pub_int_bridge_topic_list_;
    std::vector<std::string> pub_bytes_bridge_topic_list_;
    std::vector<int> pub_float_bridge_canid_list_;
    std::vector<int> pub_int_bridge_canid_list_;
    std::vector<int> pub_bytes_bridge_canid_list_;
    
    std::vector<std::string> sub_float_bridge_topic_list_;
    std::vector<std::string> sub_int_bridge_topic_list_;
    std::vector<std::string> sub_bytes_bridge_topic_list_;
    std::vector<int> sub_float_bridge_canid_list_;
    std::vector<int> sub_int_bridge_canid_list_;
    std::vector<int> sub_bytes_bridge_canid_list_;

    std::atomic<bool> running_{false};
    std::thread rx_thread_;

    std::unique_ptr<CanBridge> can_bridge;

    void float_sub_process(int canid, std_msgs::msg::Float32MultiArray::ConstSharedPtr rxdata);
    void int_sub_process(int canid, std_msgs::msg::Int32MultiArray::ConstSharedPtr rxdata);
    void bytes_sub_process(int canid, std_msgs::msg::ByteMultiArray::ConstSharedPtr rxdata);
};