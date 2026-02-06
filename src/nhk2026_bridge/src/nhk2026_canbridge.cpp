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
    this->Ifname = this->get_parameter("ifname").as_string();

    const std::vector<int64_t> pub_float_canids = this->get_parameter("pub_float_bridge_canid").as_integer_array();
    const std::vector<int64_t> pub_int_canids = this->get_parameter("pub_int_bridge_canid").as_integer_array();
    const std::vector<int64_t> pub_bytes_canids = this->get_parameter("pub_bytes_bridge_canid").as_integer_array();
    const std::vector<int64_t> sub_float_canids = this->get_parameter("sub_float_bridge_canid").as_integer_array();
    const std::vector<int64_t> sub_int_canids = this->get_parameter("sub_int_bridge_canid").as_integer_array();
    const std::vector<int64_t> sub_bytes_canids = this->get_parameter("sub_bytes_bridge_canid").as_integer_array();

    this->pub_float_bridge_canid_list_.assign(pub_float_canids.begin(), pub_float_canids.end());
    this->pub_int_bridge_canid_list_.assign(pub_int_canids.begin(), pub_int_canids.end());
    this->pub_bytes_bridge_canid_list_.assign(pub_bytes_canids.begin(), pub_bytes_canids.end());
    this->sub_float_bridge_canid_list_.assign(sub_float_canids.begin(), sub_float_canids.end());
    this->sub_int_bridge_canid_list_.assign(sub_int_canids.begin(), sub_int_canids.end());
    this->sub_bytes_bridge_canid_list_.assign(sub_bytes_canids.begin(), sub_bytes_canids.end());

    const bool mismatch_pub_float = this->pub_float_bridge_topic_list_.size() != this->pub_float_bridge_canid_list_.size();
    const bool mismatch_pub_int = this->pub_int_bridge_topic_list_.size() != this->pub_int_bridge_canid_list_.size();
    const bool mismatch_pub_bytes = this->pub_bytes_bridge_topic_list_.size() != this->pub_bytes_bridge_canid_list_.size();
    const bool mismatch_sub_float = this->sub_float_bridge_topic_list_.size() != this->sub_float_bridge_canid_list_.size();
    const bool mismatch_sub_int = this->sub_int_bridge_topic_list_.size() != this->sub_int_bridge_canid_list_.size();
    const bool mismatch_sub_bytes = this->sub_bytes_bridge_topic_list_.size() != this->sub_bytes_bridge_canid_list_.size();

    if (mismatch_pub_float || mismatch_pub_int || mismatch_pub_bytes ||
        mismatch_sub_float || mismatch_sub_int || mismatch_sub_bytes)
    {
        return CallbackReturn::FAILURE;
    }

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
    const bool mismatch_pub_float = this->pub_float_bridge_topic_list_.size() != this->pub_float_bridge_canid_list_.size();
    const bool mismatch_pub_int = this->pub_int_bridge_topic_list_.size() != this->pub_int_bridge_canid_list_.size();
    const bool mismatch_pub_bytes = this->pub_bytes_bridge_topic_list_.size() != this->pub_bytes_bridge_canid_list_.size();
    const bool mismatch_sub_float = this->sub_float_bridge_topic_list_.size() != this->sub_float_bridge_canid_list_.size();
    const bool mismatch_sub_int = this->sub_int_bridge_topic_list_.size() != this->sub_int_bridge_canid_list_.size();
    const bool mismatch_sub_bytes = this->sub_bytes_bridge_topic_list_.size() != this->sub_bytes_bridge_canid_list_.size();

    if (mismatch_pub_float || mismatch_pub_int || mismatch_pub_bytes ||
        mismatch_sub_float || mismatch_sub_int || mismatch_sub_bytes)
    {
        return CallbackReturn::FAILURE;
    }
    
	try
	{
		this->can_bridge = std::make_unique<CanBridge>(this->Ifname);
	}
	catch(const std::exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "%s", e.what());
		return CallbackReturn::FAILURE;
	}

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

    this->int_publisher_.reserve(this->pub_int_bridge_topic_list_.size());
    for (size_t i = 0; i < this->pub_int_bridge_canid_list_.size(); i++)
    {
        this->int_publisher_.push_back(
            this->create_publisher<std_msgs::msg::Int32MultiArray>(
                this->pub_int_bridge_topic_list_[i],
                device
            )
        );
    }
    this->int_subscribers_.reserve(this->sub_int_bridge_topic_list_.size());
    for (size_t i = 0; i < this->sub_int_bridge_canid_list_.size(); i++)
    {
        this->int_subscribers_.push_back(
            this->create_subscription<std_msgs::msg::Int32MultiArray>(
                this->sub_int_bridge_topic_list_[i],
                device,
                [this, i](std_msgs::msg::Int32MultiArray::ConstSharedPtr rxdata) {
                    this->int_sub_process(this->sub_int_bridge_canid_list_[i], rxdata);
                }
            )
        );
    }

    this->bytes_publisher_.reserve(this->pub_bytes_bridge_topic_list_.size());
    for (size_t i = 0; i < this->pub_bytes_bridge_canid_list_.size(); i++)
    {
        this->bytes_publisher_.push_back(
            this->create_publisher<std_msgs::msg::ByteMultiArray>(
                this->pub_bytes_bridge_topic_list_[i],
                device
            )
        );
    }
    this->bytes_subscribers_.reserve(this->sub_bytes_bridge_topic_list_.size());
    for (size_t i = 0; i < this->sub_bytes_bridge_canid_list_.size(); i++)
    {
        this->bytes_subscribers_.push_back(
            this->create_subscription<std_msgs::msg::ByteMultiArray>(
                this->sub_bytes_bridge_topic_list_[i],
                device,
                [this, i](std_msgs::msg::ByteMultiArray::ConstSharedPtr rxdata) {
                    this->bytes_sub_process(this->sub_bytes_bridge_canid_list_[i], rxdata);
                }
            )
        );
    }

    this->running_.store(true);
    this->rx_thread_ = std::thread([this] {this->rx_loop();});

    RCLCPP_INFO(
        get_logger(),
        "on_activate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->float_subscribers_.clear();
    this->int_subscribers_.clear();
    this->bytes_subscribers_.clear();
    
    this->running_.store(false);
    if (this->can_bridge) this->can_bridge->shutdown();
    if (this->rx_thread_.joinable()) this->rx_thread_.join();
    this->can_bridge.reset();
    
    this->float_publisher_.clear();
    this->int_publisher_.clear();
    this->bytes_publisher_.clear();

    RCLCPP_INFO(
        get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->float_subscribers_.clear();
    this->int_subscribers_.clear();
    this->bytes_subscribers_.clear();
    
    this->running_.store(false);
    if (this->can_bridge) this->can_bridge->shutdown();
    if (this->rx_thread_.joinable()) this->rx_thread_.join();
    this->can_bridge.reset();
    
    this->float_publisher_.clear();
    this->int_publisher_.clear();
    this->bytes_publisher_.clear();

    RCLCPP_INFO(
        get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_error(const rclcpp_lifecycle::State &state)
{
    this->float_subscribers_.clear();
    this->int_subscribers_.clear();
    this->bytes_subscribers_.clear();
    
    this->running_.store(false);
    if (this->can_bridge) this->can_bridge->shutdown();
    if (this->rx_thread_.joinable()) this->rx_thread_.join();
    this->can_bridge.reset();
    
    this->float_publisher_.clear();
    this->int_publisher_.clear();
    this->bytes_publisher_.clear();

    RCLCPP_INFO(
        get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CanBridgenhk2026::CallbackReturn CanBridgenhk2026::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->float_subscribers_.clear();
    this->int_subscribers_.clear();
    this->bytes_subscribers_.clear();
    
    this->running_.store(false);
    if (this->can_bridge) this->can_bridge->shutdown();
    if (this->rx_thread_.joinable()) this->rx_thread_.join();
    this->can_bridge.reset();
    
    this->float_publisher_.clear();
    this->int_publisher_.clear();
    this->bytes_publisher_.clear();

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

    const uint8_t st = this->get_current_state().id();
    if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        result.successful = false;
        result.reason = "can_bridge_node is active";
        return result;
    }

    result.successful = true;
    result.reason = "success";

    for (const rclcpp::Parameter &param : parameters)
    {
        const std::string &name = param.get_name();
        const rclcpp::ParameterType type = param.get_type();

        if (name == "ifname" && type == rclcpp::ParameterType::PARAMETER_STRING)
        {
            this->Ifname = param.as_string();
            continue;
        }

        if (name == "pub_float_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->pub_float_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_int_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->pub_int_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_bytes_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->pub_bytes_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_float_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->sub_float_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_int_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->sub_int_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_bytes_bridge_topic" && type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            const std::vector<std::string> arr = param.as_string_array();
            this->sub_bytes_bridge_topic_list_.assign(arr.begin(), arr.end());
            continue;
        }

        if (name == "pub_float_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->pub_float_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_int_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->pub_int_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "pub_bytes_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->pub_bytes_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_float_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->sub_float_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_int_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->sub_int_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
        if (name == "sub_bytes_bridge_canid" && type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> arr = param.as_integer_array();
            this->sub_bytes_bridge_canid_list_.assign(arr.begin(), arr.end());
            continue;
        }
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
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
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

void CanBridgenhk2026::float_sub_process(int canid, std_msgs::msg::Float32MultiArray::ConstSharedPtr rxdata)
{
    try
    {
        this->can_bridge->send_float(canid, rxdata->data);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}

void CanBridgenhk2026::int_sub_process(int canid, std_msgs::msg::Int32MultiArray::ConstSharedPtr rxdata)
{
    try
    {
    this->can_bridge->send_int(canid, rxdata->data);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}

void CanBridgenhk2026::bytes_sub_process(int canid, std_msgs::msg::ByteMultiArray::ConstSharedPtr rxdata)
{
    try
    {
    this->can_bridge->send_bytes(canid, rxdata->data);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
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
