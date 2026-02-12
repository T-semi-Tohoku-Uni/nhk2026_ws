#include "system_1st_video.hpp"

using std::placeholders::_1;

System1stVideo::System1stVideo()
: rclcpp_lifecycle::LifecycleNode(std::string("syste_1st_video")),
cmd_vel_mode(system_request::STANDBY)
{

}

System1stVideo::CallbackReturn System1stVideo::on_configure(const rclcpp_lifecycle::State &state)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&System1stVideo::parameters_callback, this, _1)
    );

    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        std::string("cmd_vel"),
        device
    );

    RCLCPP_INFO(
        get_logger(),
        "on_configure() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}

System1stVideo::CallbackReturn System1stVideo::on_activate(const rclcpp_lifecycle::State &state)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    if (this->cmd_vel_publisher_)
    {
        this->cmd_vel_publisher_->on_activate();
    }

    this->cmd_vel_ui_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        std::string("cmd_vel_ui"),
        device,
        std::bind(&System1stVideo::cmd_vel_ui_callback, this, _1)
    );

    this->back_arm_robstride_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        std::string("back_arm_robstride"),
        device,
        std::bind(&System1stVideo::back_arm_robstride_callback, this, _1)
    );

    this->middle_arm_robstride_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        std::string("middle_arm_robstride"),
        device,
        std::bind(&System1stVideo::middle_arm_robstride_callback, this, _1)
    );

    this->back_robomastar_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        std::string("back_robomastar"),
        device,
        std::bind(&System1stVideo::back_robomastar_callback, this, _1)
    );

    this->flag_server_ = this->create_service<nhk2026_msgs::srv::SystemR2>(
        std::string("state_service"),
        std::bind(&System1stVideo::flag_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(
        get_logger(),
        "on_activate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}

System1stVideo::CallbackReturn System1stVideo::on_deactivate(const rclcpp_lifecycle::State &state)
{
    this->cmd_vel_ui_subscription_.reset();
    this->back_arm_robstride_subscription_.reset();
    this->middle_arm_robstride_subscription_.reset();
    this->back_robomastar_subscription_.reset();
    this->flag_server_.reset();

    this->cmd_vel_publisher_->on_deactivate();

    RCLCPP_INFO(
        get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}

System1stVideo::CallbackReturn System1stVideo::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->cmd_vel_ui_subscription_.reset();
    this->back_arm_robstride_subscription_.reset();
    this->middle_arm_robstride_subscription_.reset();
    this->back_robomastar_subscription_.reset();
    this->flag_server_.reset();
    this->cmd_vel_publisher_.reset();

    RCLCPP_INFO(
        get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}

System1stVideo::CallbackReturn System1stVideo::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(
        get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}

System1stVideo::CallbackReturn System1stVideo::on_shutdown(const rclcpp_lifecycle::State &state)
{
    this->cmd_vel_ui_subscription_.reset();
    this->back_arm_robstride_subscription_.reset();
    this->middle_arm_robstride_subscription_.reset();
    this->back_robomastar_subscription_.reset();
    this->flag_server_.reset();
    this->cmd_vel_publisher_.reset();

    RCLCPP_INFO(
        get_logger(),
        "on_shutdown() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());

    return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult System1stVideo::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    static_cast<void>(parameters);
    return result;
}

void System1stVideo::cmd_vel_ui_callback(geometry_msgs::msg::Twist::SharedPtr rxdata)
{
    static_cast<void>(rxdata);
}

void System1stVideo::cmd_vel_stick_callback(geometry_msgs::msg::Twist::SharedPtr rxdata)
{
    static_cast<void>(rxdata);
}

void System1stVideo::back_arm_robstride_callback(std_msgs::msg::Float32MultiArray::SharedPtr rxdata)
{
    static_cast<void>(rxdata);
}

void System1stVideo::middle_arm_robstride_callback(std_msgs::msg::Float32MultiArray::SharedPtr rxdata)
{
    static_cast<void>(rxdata);
}

void System1stVideo::back_robomastar_callback(std_msgs::msg::Float32MultiArray::SharedPtr rxdata)
{
    static_cast<void>(rxdata);
}

void System1stVideo::flag_callback(
    const std::shared_ptr<nhk2026_msgs::srv::SystemR2::Request> request_state,
    std::shared_ptr<nhk2026_msgs::srv::SystemR2::Response> success
)
{
    if (this->cmd_vel_mode == system_request::STANDBY)
    {
        this->cmd_vel_mode = request_state->mode;
        success->success = true;
    }
    else
    {
        success->success = false;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<System1stVideo> node = std::make_shared<System1stVideo>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
