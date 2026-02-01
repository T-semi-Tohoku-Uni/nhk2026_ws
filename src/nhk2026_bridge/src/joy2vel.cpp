#include "joy2vel.hpp"

Joy2Vel::Joy2Vel()
: rclcpp_lifecycle::LifecycleNode(std::string("joy_vel_converter"))
{
    this->declare_parameter<double>("max_vx", 2.0);
    this->declare_parameter<double>("max_vy", 2.0);
    this->declare_parameter<double>("max_omega", M_PI);
    this->declare_parameter<double>("max_button_vx", 0.2);
    this->declare_parameter<double>("max_button_vy", 0.2);

    this->max_vx = this->get_parameter("max_vx").as_double();
    this->max_vy = this->get_parameter("max_vy").as_double();
    this->max_omega = this->get_parameter("max_omega").as_double();
    
    this->max_button_vx = this->get_parameter("max_button_vx").as_double();
    this->max_button_vy = this->get_parameter("max_button_vy").as_double();

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Joy2Vel::parameters_callback, this, _1)
    );
}

Joy2Vel::CallbackReturn Joy2Vel::on_configure(const rclcpp_lifecycle::State &state)
{
    this->vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        std::string("cmd_vel"),
        rclcpp::SystemDefaultsQoS()
    );

    this->joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        std::string("joy"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&Joy2Vel::joy_callback, this, _1)
    );

    RCLCPP_INFO(
      get_logger(),
      "on_configure() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());

    return CallbackReturn::SUCCESS;
}

Joy2Vel::CallbackReturn Joy2Vel::on_activate(const rclcpp_lifecycle::State &state)
{
    this->vel_publisher->on_activate();

    RCLCPP_INFO(
      get_logger(),
      "on_activate() called. state: id=%u, label=%s",
      state.id(),
      state.label().c_str());

    return CallbackReturn::SUCCESS;
}

Joy2Vel::CallbackReturn Joy2Vel::on_deactivate(const rclcpp_lifecycle::State &state)
{
    if (this->vel_publisher->is_activated()) {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = 0;
        txdata.linear.y = 0;
        txdata.linear.z = 0;

        txdata.angular.x = 0;
        txdata.angular.y = 0;
        txdata.angular.z = 0;
        this->vel_publisher->publish(txdata);
    }
    this->vel_publisher->on_deactivate();

    RCLCPP_INFO(
        get_logger(),
        "on_deactivate() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

Joy2Vel::CallbackReturn Joy2Vel::on_cleanup(const rclcpp_lifecycle::State &state)
{
    this->joy_subscriber.reset();
    this->vel_publisher.reset();
    RCLCPP_INFO(
        get_logger(),
        "on_cleanup() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

Joy2Vel::CallbackReturn Joy2Vel::on_error(const rclcpp_lifecycle::State &state)
{
    if (this->vel_publisher && this->vel_publisher->is_activated()) {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = 0;
        txdata.linear.y = 0;
        txdata.linear.z = 0;

        txdata.angular.x = 0;
        txdata.angular.y = 0;
        txdata.angular.z = 0;
        this->vel_publisher->publish(txdata);
    }
    RCLCPP_INFO(
        get_logger(),
        "on_error() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

Joy2Vel::CallbackReturn Joy2Vel::on_shutdown(const rclcpp_lifecycle::State &state)
{
    if (this->vel_publisher && this->vel_publisher->is_activated()) {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = 0;
        txdata.linear.y = 0;
        txdata.linear.z = 0;

        txdata.angular.x = 0;
        txdata.angular.y = 0;
        txdata.angular.z = 0;
        this->vel_publisher->publish(txdata);
    }
    this->joy_subscriber.reset();
    this->vel_publisher.reset();
    RCLCPP_INFO(
        get_logger(),
        "on_shutdown() called. state: id=%u, label=%s",
        state.id(),
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void Joy2Vel::joy_callback(const sensor_msgs::msg::Joy::SharedPtr rxdata)
{
    if (this->vel_publisher->is_activated()) {
        geometry_msgs::msg::Twist txdata;
        txdata.linear.x = -rxdata->axes[0] * max_vx - rxdata->axes[6]*max_button_vx;
        txdata.linear.y =  rxdata->axes[1] * max_vy + rxdata->axes[7]*max_button_vy;
        txdata.linear.z = 0;

        txdata.angular.x = 0;
        txdata.angular.y = 0;
        txdata.angular.z = (rxdata->axes[5] - rxdata->axes[2])*max_omega;
        this->vel_publisher->publish(txdata);
    }
}

rcl_interfaces::msg::SetParametersResult Joy2Vel::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "max_vx" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_vx = param.as_double();
        }
        else if (param.get_name() == "max_vy" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_vy = param.as_double();
        }
        else if (param.get_name() == "max_omega" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_omega = param.as_double();
        }
        else if (param.get_name() == "max_button_vx" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_button_vx = param.as_double();
        }
        else if (param.get_name() == "max_button_vy" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->max_button_vy = param.as_double();
        }
    }

    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Joy2Vel> node = std::make_shared<Joy2Vel>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}