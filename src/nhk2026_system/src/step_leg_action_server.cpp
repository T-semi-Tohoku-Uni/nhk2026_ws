#include "nhk2026_system/step_leg_action_server.hpp"

using namespace std::chrono_literals;

StepLegActionServer::StepLegActionServer(): rclcpp::Node("step_leg_action_server"){
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->action_server_ = rclcpp_action::create_server<LegMove>(
        this,
        "step_leg",
        std::bind(&StepLegActionServer::handle_goal, this, _1, _2),
        std::bind(&StepLegActionServer::handle_cancel,this, _1),
        std::bind(&StepLegActionServer::handle_accepted, this, _1)
    );

    this->joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        std::string("/joint_states_step"),
        device,
        std::bind(&StepLegActionServer::joint_state_callback, this, _1)
    );

    this->declare_parameter<double>("kPosTolerance", 0.01);
    this->kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&StepLegActionServer::parameters_callback, this, _1)
    );
}

rcl_interfaces::msg::SetParametersResult StepLegActionServer::parameters_callback(const std::vector<rclcpp::Parameter> &parameters){
    rcl_interfaces::msg::SetParametersResult result;

    if (this->goal_active_)
    {
        result.successful = false;
        result.reason = "action is processing";
        return result;
    }

    for (const auto &param : parameters) {
        if (param.get_name() == "kPosTolerance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            this->kPosTolerance_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "kPosTolerance changed!");
        }
    }

    result.successful = true;
    result.reason = "success";
    return result;
}

rclcpp_action::GoalResponse StepLegActionServer::handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const LegMove::Goal> goal){
    // todo アームの到達範囲をチェック

    if (!this->joint_subscribe_flag_)
    {
        RCLCPP_ERROR(this->get_logger(), "please start joint state publisher");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->joint_states.position.size() < 3)
    {
        RCLCPP_ERROR(this->get_logger(), "joint states' size is short");
        return rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;
    if (!this->goal_active_.compare_exchange_strong(expected, true))
    {
        RCLCPP_ERROR(this->get_logger(), "action server is active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "leg start to move!");
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse StepLegActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleLegMove> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void StepLegActionServer::handle_accepted(const std::shared_ptr<GoalHandleLegMove> goal_handle)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->right_leg_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("right_leg"),
        device
    );
    this->left_leg_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("left_leg"),
        device
    );
    this->back_leg_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("back_leg"),
        device
    );
    this->feedback_timer_ = this->create_wall_timer(
        0.05s,
        std::bind(&StepLegActionServer::feedback_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    std::thread{std::bind(&StepLegActionServer::execute, this, _1), goal_handle}.detach();
}

void StepLegActionServer::execute(const std::shared_ptr<GoalHandleLegMove> goal_handle)
{
    const std::shared_ptr<const nhk2026_msgs::action::LegMove_Goal> goal = goal_handle->get_goal();
    LegMove::Result::SharedPtr result = std::make_shared<LegMove::Result>();

    rclcpp::Rate loop_rate(100.0);
    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->msg = "goal canceled";
            goal_handle->canceled(result);
            this->goal_active_ = false;
            return;
        }

        bool right_reached = std::fabs(this->now_joint_.position[0] - goal->joint_states.position[0]) <= this->kPosTolerance_;
        bool left_reached  = std::fabs(this->now_joint_.position[1] - goal->joint_states.position[1]) <= this->kPosTolerance_;
        bool back_reached  = std::fabs(this->now_joint_.position[2] - goal->joint_states.position[2]) <= this->kPosTolerance_;

        if (!right_reached)
        {
            std_msgs::msg::Float32MultiArray cmd;
            cmd.data = {static_cast<float>(goal->joint_states.position[0]), goal->max_speed, goal->max_acc};
            right_leg_motor_publisher_->publish(cmd);
        }
        if (!left_reached)
        {
            std_msgs::msg::Float32MultiArray cmd;
            cmd.data = {static_cast<float>(goal->joint_states.position[1]), goal->max_speed, goal->max_acc};
            left_leg_motor_publisher_->publish(cmd);
        }
        if (!back_reached)
        {
            std_msgs::msg::Float32MultiArray cmd;
            cmd.data = {static_cast<float>(goal->joint_states.position[2]), goal->max_speed, goal->max_acc};
            back_leg_motor_publisher_->publish(cmd);
        }
    
        if (right_reached && left_reached && back_reached)
        {
            result->success = true;
            result->msg = "goal reached";
            goal_handle->succeed(result);
            this->goal_active_ = false;
            return;
        }

        loop_rate.sleep();
    }

    result->success = false;
    result->msg = "rclcpp shutdown";
    goal_handle->abort(result);
    this->goal_active_ = false;
}
void StepLegActionServer::feedback_timer_callback()
{
    LegMove::Feedback::SharedPtr feedback = std::make_shared<LegMove::Feedback>();
    // todo フィードバックを送る
}

void StepLegActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata)
{
    if (rxdata->position.size() != 3)
    {
        RCLCPP_WARN(this->get_logger(), "joint size is invalid");
        return;
    }
    this->joint_subscribe_flag_ = true;
    this->now_joint_ = *rxdata;
    // todo jointからエンドエフェクタの場所を計算（ここじゃなくてもいいかも）
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<StepLegActionServer> node = std::make_shared<StepLegActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}