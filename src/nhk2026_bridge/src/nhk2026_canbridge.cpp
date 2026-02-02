#include "nhk2026_canbridge.hpp"

#include <functional>
#include <utility>

using std::placeholders::_1;

CanBridgenhk2026::CanBridgenhk2026()
: rclcpp_lifecycle::LifecycleNode(std::string("nhk2026_canbridge"))
{
    this->declare_parameter("ifname", "can0");
    std::vector<std::string> default_topic = {};
    this->declare_parameter("pub_float_bridge_topic", default_topic);
    this->declare_parameter("pub_int_bridge_topic", default_topic);
    this->declare_parameter("pub_bytes_bridge_topic", default_topic);
    this->declare_parameter("sub_float_bridge_topic", default_topic);
    this->declare_parameter("sub_int_bridge_topic", default_topic);
    this->declare_parameter("sub_bytes_bridge_topic", default_topic);

    std::vector<int> default_canid = {};
    this->declare_parameter("pub_float_bridge_canid", default_canid);
    this->declare_parameter("pub_int_bridge_canid", default_canid);
    this->declare_parameter("pub_bytes_bridge_canid", default_canid);
    this->declare_parameter("sub_float_bridge_canid", default_canid);
    this->declare_parameter("sub_int_bridge_canid", default_canid);
    this->declare_parameter("sub_bytes_bridge_canid", default_canid);
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_configure(const rclcpp_lifecycle::State &state)
{
    this->pub_float_bridge_topic_list_ = this->get_parameter("pub_float_bridge_topic").as_string_array();
    this->pub_int_bridge_topic_list_ = this->get_parameter("pub_int_bridge_topic").as_string_array();
    this->pub_bytes_bridge_topic_list_ = this->get_parameter("pub_bytes_bridge_topic").as_string_array();
    this->sub_float_bridge_topic_list_ = this->get_parameter("sub_float_bridge_topic").as_string_array();
    this->sub_int_bridge_topic_list_ = this->get_parameter("sub_int_bridge_topic").as_string_array();
    this->sub_bytes_bridge_topic_list_ = this->get_parameter("sub_bytes_bridge_topic").as_string_array();

    auto assign_canid = [this](const char *name, std::vector<int> &dest)
    {
        auto src = this->get_parameter(name).as_integer_array();
        dest.assign(src.begin(), src.end());
    };
    assign_canid("pub_float_bridge_canid", this->pub_float_bridge_canid_list_);
    assign_canid("pub_int_bridge_canid", this->pub_int_bridge_canid_list_);
    assign_canid("pub_bytes_bridge_canid", this->pub_bytes_bridge_canid_list_);
    assign_canid("sub_float_bridge_canid", this->sub_float_bridge_canid_list_);
    assign_canid("sub_int_bridge_canid", this->sub_int_bridge_canid_list_);
    assign_canid("sub_bytes_bridge_canid", this->sub_bytes_bridge_canid_list_);

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CanBridgenhk2026::parameters_callback, this, _1)
    );
    RCLCPP_INFO(
        get_logger(),
        "on_configure() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_activate(const rclcpp_lifecycle::State &state)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->float_publisher_.reserve(this->pub_float_bridge_topic_list_.size());
    for (size_t i = 0; i < this->pub_float_bridge_canid_list_.size(); i++)
    {
        this->float_publisher_.push_back(
            this->create_publisher<std_msgs::msg::Float32MultiArray>(
                this->pub_float_bridge_topic_list_[i],
                device
            )
        );
    }
    this->float_subscribers_.reserve(this->sub_float_bridge_topic_list_.size());
    for (size_t i = 0; i < this->sub_float_bridge_canid_list_.size(); i++)
    {
        this->float_subscribers_.push_back(
            this->create_subscription<std_msgs::msg::Float32MultiArray>(
                this->sub_float_bridge_topic_list_[i],
                device,
                [this, i](std_msgs::msg::Float32MultiArray::ConstSharedPtr rxdata) {
                    this->float_sub_process(this->sub_float_bridge_canid_list_[i], rxdata);
                }
            )
        );
    }
    RCLCPP_INFO(
        get_logger(),
        "on_activate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_deactivate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
        get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
        get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
        get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
        get_logger(),
        "on_shutdown() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult CanBridgenhk2026::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        
    }

    return result;
}

void CanBridgenhk2026::rx_loop()
{
    while (rclcpp::ok() && this->running_.load() && this->can_bridge != nullptr)
    {
        CanBridge::RxData_struct rxdata;
        try
        {
            rxdata = this->can_bridge->receive_data();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            continue;
        }

        for (size_t i = 0; i < this->pub_float_bridge_canid_list_.size(); i++)
        {
            if (this->pub_float_bridge_canid_list_[i] == rxdata.canid)
            {
                std::vector<float> txdata_f = this->can_bridge->rxdata_to_float(rxdata);
                std_msgs::msg::Float32MultiArray txdata;
                txdata.data = txdata_f;
                this->float_publisher_[i]->publish(txdata);
                break;
            }
        }

        for (size_t i = 0; i < this->pub_int_bridge_canid_list_.size(); i++)
        {
            if (this->pub_int_bridge_canid_list_[i] == rxdata.canid)
            {
                std::vector<int> txdata_i = this->can_bridge->rxdata_to_int(rxdata);
                std_msgs::msg::Int32MultiArray txdata;
                txdata.data = txdata_i;
                this->int_publisher_[i]->publish(txdata);
                break;
            }
        }

        for (size_t i = 0; i < this->pub_bytes_bridge_canid_list_.size(); i++)
        {
            if (this->pub_bytes_bridge_canid_list_[i] == rxdata.canid)
            {
                std_msgs::msg::ByteMultiArray txdata;
                txdata.data = std::move(rxdata.data);
                this->bytes_publisher_[i]->publish(txdata);
                break;
            }
        }
    }
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<CanBridgenhk2026> node = std::make_shared<CanBridgenhk2026>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
