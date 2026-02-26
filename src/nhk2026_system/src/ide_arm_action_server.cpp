#include "ide_arm_action_server.hpp"
#include <cmath>

using namespace std::chrono_literals;

IdeArmActionServer::IdeArmActionServer()
: rclcpp::Node("ide_arm_action_server")
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->action_server_ = rclcpp_action::create_server<ArmMove>(
        this,
        "ide_arm",
        std::bind(&IdeArmActionServer::handle_goal, this, _1, _2),
        std::bind(&IdeArmActionServer::handle_cancel,this, _1),
        std::bind(&IdeArmActionServer::handle_accepted, this, _1)
    );

    this->joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        std::string("/joint_states"),
        device,
        std::bind(&IdeArmActionServer::joint_state_callback, this, _1)
    );

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&IdeArmActionServer::parameters_callback, this, _1)
    );
}

rcl_interfaces::msg::SetParametersResult IdeArmActionServer::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;

    if (disable_set_parameter)
    {
        result.successful = false;
        result.reason = "action is processing";
        return result;
    }

    for (const auto &param : parameters) {
        
    }

    result.successful = true;
    result.reason = "success";
    return result;
}

rclcpp_action::GoalResponse IdeArmActionServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMove::Goal> goal
)
{
    // todo アームの到達範囲をチェック

    if (goal->joint_states.position.size() < 3)
    {
        RCLCPP_ERROR(this->get_logger(), "joint states' size is short");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "arm start to move!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IdeArmActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleArmMove> goal_handle
)
{
    // todo 位置を元に戻す
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void IdeArmActionServer::handle_accepted(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{
    rclcpp::QoS device = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    this->j1_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("j1"),
        device
    );
    this->j2_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("j2"),
        device
    );
    this->j3_motor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        std::string("j3"),
        device
    );
    this->feedback_timer_ = this->create_wall_timer(
        0.05s,
        std::bind(&IdeArmActionServer::feedback_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    std::thread{std::bind(&IdeArmActionServer::execute, this, _1), goal_handle}.detach();
}

void IdeArmActionServer::execute(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{
    const std::shared_ptr<const nhk2026_msgs::action::ArmMove_Goal> goal = goal_handle->get_goal();
    ArmMove::Result::SharedPtr result = std::make_shared<ArmMove::Result>();
    disable_set_parameter = true;

    rclcpp::Rate loop_rate(100.0);
    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->msg = "goal canceled";
            goal_handle->canceled(result);
            disable_set_parameter = false;
            return;
        }

        // 
        if (std::fabs(this->now_joint_.position[0] - goal->joint_states.position[0]) > this->kPosTolerance_)
        {
            std_msgs::msg::Float32MultiArray cmd;
            std::vector<float> cmd_data = {static_cast<float>(goal->joint_states.position[0]), goal->max_speed, goal->max_acc};
            cmd.data = cmd_data;
            j1_motor_publisher_->publish(cmd);
        }
        else if (!is_reached(1))
        {
            cmd.data = joints[1];
            j2_motor_publisher_->publish(cmd);
        }
        else if (!is_reached(2))
        {
            cmd.data = joints[2];
            j3_motor_publisher_->publish(cmd);
        }
        else
        {
            result->success = true;
            result->msg = "goal reached";
            goal_handle->succeed(result);
            disable_set_parameter = false;
            return;
        }

        loop_rate.sleep();
    }

    result->success = false;
    result->msg = "rclcpp shutdown";
    goal_handle->abort(result);
    disable_set_parameter = false;

}

void IdeArmActionServer::feedback_timer_callback()
{
    ArmMove::Feedback::SharedPtr feedback = std::make_shared<ArmMove::Feedback>();
    // todo フィードバックを送る
}

void IdeArmActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata)
{
    now_joint_ = *rxdata;
    // todo jointからエンドエフェクタの場所を計算（ここじゃなくてもいいかも）
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<IdeArmActionServer> node = std::make_shared<IdeArmActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
