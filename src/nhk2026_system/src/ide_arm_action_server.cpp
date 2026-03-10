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

    this->declare_parameter<double>("kPosTolerance", 0.01);
    this->kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();

    this->parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&IdeArmActionServer::parameters_callback, this, _1)
    );
}

rcl_interfaces::msg::SetParametersResult IdeArmActionServer::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
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

rclcpp_action::GoalResponse IdeArmActionServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmMove::Goal> goal
)
{
    // todo アームの到達範囲をチェック

    if (!this->joint_subscribe_flag_)
    {
        RCLCPP_ERROR(this->get_logger(), "please start joint state publisher");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else if (!this->robot_description_flag_)
    {
        RCLCPP_ERROR(this->get_logger(), "please robot state publisher");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else if (goal->joint_states.position.size() < 3)
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
    this->active_goal_handle_ = goal_handle;

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


    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    const std::shared_ptr<const nhk2026_msgs::action::ArmMove_Goal> goal = goal_handle->get_goal();
    KDL::JntArray q_current(chain.getNrOfJoints());
    q_current(0) = goal->joint_states.position[0];
    q_current(1) = goal->joint_states.position[1];
    q_current(2) = goal->joint_states.position[2];

    KDL::Frame current_pose;
    int fk_status = fk_solver.JntToCart(q_current, current_pose);
    if (fk_status < 0) {
        RCLCPP_ERROR(this->get_logger(), "FK failed.");
    }
    
    double roll, pitch, yaw;
    current_pose.M.GetRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    std::thread{std::bind(&IdeArmActionServer::execute, this, _1), goal_handle}.detach();
}

void IdeArmActionServer::execute(const std::shared_ptr<GoalHandleArmMove> goal_handle)
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
            this->active_goal_handle_.reset();
            return;
        }

        if (std::fabs(this->now_joint_.position[0] - goal->joint_states.position[0]) > this->kPosTolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "j1 moving!");
            std_msgs::msg::Float32MultiArray cmd;
            std::vector<float> cmd_data = {static_cast<float>(goal->joint_states.position[0]), goal->max_speed, goal->max_acc};
            cmd.data = cmd_data;
            j1_motor_publisher_->publish(cmd);
        }
        else if (std::fabs(this->now_joint_.position[1] - goal->joint_states.position[1]) > this->kPosTolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "j2 moving!");
            std_msgs::msg::Float32MultiArray cmd;
            std::vector<float> cmd_data = {static_cast<float>(goal->joint_states.position[1]), goal->max_speed, goal->max_acc};
            cmd.data = cmd_data;
            j2_motor_publisher_->publish(cmd);
        }
        else if (std::fabs(this->now_joint_.position[2] - goal->joint_states.position[2]) > this->kPosTolerance_)
        {
            RCLCPP_INFO(this->get_logger(), "j3 moving!");
            std_msgs::msg::Float32MultiArray cmd;
            std::vector<float> cmd_data = {static_cast<float>(goal->joint_states.position[2]), goal->max_speed, goal->max_acc};
            cmd.data = cmd_data;
            j3_motor_publisher_->publish(cmd);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "complete!");
            result->success = true;
            result->msg = "goal reached";
            goal_handle->succeed(result);
            this->goal_active_ = false;
            this->active_goal_handle_.reset();
            return;
        }

        loop_rate.sleep();
    }

    result->success = false;
    result->msg = "rclcpp shutdown";
    goal_handle->abort(result);
    this->goal_active_ = false;
    this->active_goal_handle_.reset();

}

void IdeArmActionServer::feedback_timer_callback()
{
    ArmMove::Feedback::SharedPtr feedback = std::make_shared<ArmMove::Feedback>();
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::JntArray q_current(chain.getNrOfJoints());
    q_current(0) = this->now_joint_.position[0];
    q_current(1) = this->now_joint_.position[1];
    q_current(2) = this->now_joint_.position[2];

    KDL::Frame current_pose;
    int fk_status = fk_solver.JntToCart(q_current, current_pose);
    if (fk_status < 0) {
        RCLCPP_ERROR(this->get_logger(), "FK failed.");
    }
    
    feedback->current.pose.position.x = current_pose.p.x();
    feedback->current.pose.position.y = current_pose.p.y();
    feedback->current.pose.position.z = current_pose.p.z();

    double ox, oy, oz, ow;
    current_pose.M.GetQuaternion(ox, oy, oz, ow);
    feedback->current.pose.orientation.w = ow;
    feedback->current.pose.orientation.x = ox;
    feedback->current.pose.orientation.y = oy;
    feedback->current.pose.orientation.z = oz;
    feedback->current.header.frame_id = "arm_base";
    feedback->current.header.stamp = this->get_clock()->now();

    if (this->active_goal_handle_ && this->goal_active_) {
        this->active_goal_handle_->publish_feedback(feedback);
    }
}

void IdeArmActionServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata)
{
    if (rxdata->position.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(), "joint size is invalid");
        return;
    }
    this->joint_subscribe_flag_ = true;
    this->now_joint_ = *rxdata;
    // todo jointからエンドエフェクタの場所を計算（ここじゃなくてもいいかも）
}

void IdeArmActionServer::robot_description_callback(const std_msgs::msg::String::SharedPtr rxdata)
{
    this->robot_description_flag_ = true;

    KDL::Tree tree;

    if (!kdl_parser::treeFromString(rxdata->data, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree.");
    }

    const std::string base_link = "arm_base";
    const std::string end_link = "tcp_link";

    if (!tree.getChain(base_link, end_link, chain)) {
        RCLCPP_ERROR(this->get_logger(), ("Failed to extract KDL chain from " + base_link + " to " + end_link).c_str());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<IdeArmActionServer> node = std::make_shared<IdeArmActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
