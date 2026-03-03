#include "nhk2026_system/dansa_arm_action_server.hpp"

using namespace std::chrono_literals;

DansaArmActionServer::DansaArmActionServer(): rclcpp::Node("dansa_arm_action_server"){
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->action_server_ = rclcpp_action::create_server<ArmMove>(
        this,
        "dansa_arm",
        std::bind(&DansaArmActionServer::handle_goal, this, _1, _2),
        std::bind(&DansaArmActionServer::handle_cancel,this, _1),
        std::bind(&DansaArmActionServer::handle_accepted, this, _1)
    );

    this->joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        std::string("/joint_states_dansa"),
        device,
        std::bind(&DansaArmActionServer::joint_state_callback, this, _1)
    );

    this->declare_parameter<double>("kPosTolerance", 0.01);
    this->kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DansaArmActionServer::parameters_callback, this, _1)
    );
}

rcl_interfaces::msg::SetParametersResult DansaArmActionServer::parameters_callback(const std::vector<rclcpp::Parameter> &parameters){
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

rclcpp_action::GoalResponse DansaArmActionServer::handle_goal(const rclcpp_action::GoalUUID &,std::shared_ptr<const ArmMove::Goal> goal){
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

    RCLCPP_INFO(this->get_logger(), "arm start to move!");
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DansaArmActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleArmMove> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DansaArmActionServer::handle_accepted(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->right_arm_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("right_arm"),
        device
    );
    this->left_arm_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("left_arm"),
        device
    );
    this->back_arm_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("back_arm"),
        device
    );
    this->feedback_timer_ = this->create_wall_timer(
        0.05s,
        std::bind(&DansaArmActionServer::feedback_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    std::thread{std::bind(&DansaArmActionServer::execute, this, _1), goal_handle}.detach();
}

void DansaArmActionServer::execute(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{
    const std::shared_ptr<const nhk2026_msgs::action::ArmMove_Goal> goal = goal_handle->get_goal();
    ArmMove::Result::SharedPtr result = std::make_shared<ArmMove::Result>();

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

        // 1. 各アームが目標に到達しているかを計算
        bool right_reached = std::fabs(this->now_joint_.position[0] - goal->joint_states.position[0]) <= this->kPosTolerance_;
        bool left_reached  = std::fabs(this->now_joint_.position[1] - goal->joint_states.position[1]) <= this->kPosTolerance_;
        bool back_reached  = std::fabs(this->now_joint_.position[2] - goal->joint_states.position[2]) <= this->kPosTolerance_;

        // 2. まだ到達していないアームにだけ司令を送る（ここは独立した if なので同時に動く）
        if (!right_reached)
        {
            std_msgs::msg::Float32MultiArray cmd;
            cmd.data = {static_cast<float>(goal->joint_states.position[0]), goal->max_speed, goal->max_acc};
            right_arm_motor_publisher_->publish(cmd);
        }
        if (!left_reached)
        {
            std_msgs::msg::Float32MultiArray cmd;
            cmd.data = {static_cast<float>(goal->joint_states.position[1]), goal->max_speed, goal->max_acc};
            left_arm_motor_publisher_->publish(cmd);
        }
        if (!back_reached)
        {
            std_msgs::msg::Float32MultiArray cmd;
            cmd.data = {static_cast<float>(goal->joint_states.position[2]), goal->max_speed, goal->max_acc};
            back_arm_motor_publisher_->publish(cmd);
        }
        
        // 3. 3つすべてのアームが目標に到達したかチェックする
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
void DansaArmActionServer::feedback_timer_callback()
{
    ArmMove::Feedback::SharedPtr feedback = std::make_shared<ArmMove::Feedback>();
    // todo フィードバックを送る
}

void DansaArmActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata)
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
    std::shared_ptr<DansaArmActionServer> node = std::make_shared<DansaArmActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}