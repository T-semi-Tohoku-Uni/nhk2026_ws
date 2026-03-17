#include "ide_arm_action_server.hpp"
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>

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

    rclcpp::QoS robot = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    this->robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        std::string("robot_description"),
        robot,
        std::bind(&IdeArmActionServer::robot_description_callback, this, _1)
    );

    this->path_client_ = this->create_client<nhk2026_msgs::srv::ArmPathPlan>(std::string("arm_path"));

    this->declare_parameter<double>("kPosTolerance", 0.01);
    this->kPosTolerance_ = this->get_parameter("kPosTolerance").as_double();
    this->declare_parameter<int>("control_frequency", 20);
    this->control_frequency_ = this->get_parameter("control_frequency").as_int();

    this->declare_parameter<std::string>("joint1_name", "joint1");
    this->declare_parameter<std::string>("joint2_name", "joint2");
    this->declare_parameter<std::string>("joint3_name", "joint3");

    this->joint1_name_ = this->get_parameter("joint1_name").as_string();
    this->joint2_name_ = this->get_parameter("joint2_name").as_string();
    this->joint3_name_ = this->get_parameter("joint3_name").as_string();

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
            if (param.as_double() <= 0.0)
            {
                result.successful = false;
                result.reason = "kPosTolerance must be positive";
                return result;
            }
            this->kPosTolerance_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "kPosTolerance changed!");
        }
        else if (param.get_name() == "control_frequency" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            if (param.as_int() <= 0)
            {
                result.successful = false;
                result.reason = "control_frequency must be positive";
                return result;
            }
            this->control_frequency_ = static_cast<int>(param.as_int());
            RCLCPP_INFO(this->get_logger(), "control_frequency changed!");
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
    else if (!this->path_client_->service_is_ready())
    {
        RCLCPP_ERROR(this->get_logger(), "arm_path service is not available");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (this->joint_positions_.size() < chain.getNrOfJoints())
    {
        RCLCPP_ERROR(this->get_logger(), "joint positions are not ready");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->max_speed <= 0.0f || goal->max_acc <= 0.0f)
    {
        RCLCPP_ERROR(this->get_logger(), "max_speed and max_acc must be positive");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (this->control_frequency_ <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "control_frequency must be positive");
        return rclcpp_action::GoalResponse::REJECT;
    }

    const auto &target_pose = goal->goal_pos.pose;
    const double q_norm =
        std::abs(target_pose.orientation.x) +
        std::abs(target_pose.orientation.y) +
        std::abs(target_pose.orientation.z) +
        std::abs(target_pose.orientation.w);
    const KDL::Rotation target_rotation = (q_norm > 1e-6)
        ? KDL::Rotation::Quaternion(
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        )
        : KDL::Rotation::Identity();
    const KDL::Frame target_frame(
        target_rotation,
        KDL::Vector(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        )
    );

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::JntArray q_min(chain.getNrOfJoints());
    KDL::JntArray q_max(chain.getNrOfJoints());
    KDL::JntArray q_init(chain.getNrOfJoints());
    KDL::JntArray q_out(chain.getNrOfJoints());
    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
    {
        q_min(i) = -std::numeric_limits<double>::max();
        q_max(i) = std::numeric_limits<double>::max();
    }

    if (chain.getNrOfJoints() > 0)
    {
        q_min(0) = this->j1_limit_down_;
        q_max(0) = this->j1_limit_up_;
    }
    if (chain.getNrOfJoints() > 1)
    {
        q_min(1) = this->j2_limit_down_;
        q_max(1) = this->j2_limit_up_;
    }
    if (chain.getNrOfJoints() > 2)
    {
        q_min(2) = this->j3_limit_down_;
        q_max(2) = this->j3_limit_up_;
    }

    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
    {
        q_init(i) = this->joint_positions_[i];
    }

    KDL::ChainIkSolverVel_pinv ik_vel_solver(chain);
    KDL::ChainIkSolverPos_NR_JL ik_solver(
        chain,
        q_min,
        q_max,
        fk_solver,
        ik_vel_solver,
        100,
        1e-6
    );

    if (ik_solver.CartToJnt(q_init, target_frame, q_out) < 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "IK seed: [%.6f, %.6f, %.6f]",
            q_init(0), q_init(1), q_init(2)
        );
        RCLCPP_ERROR(this->get_logger(), "goal pose is out of reach");
        return rclcpp_action::GoalResponse::REJECT;
    }

    const double lower_limits[] = {this->j1_limit_down_, this->j2_limit_down_, this->j3_limit_down_};
    const double upper_limits[] = {this->j1_limit_up_, this->j2_limit_up_, this->j3_limit_up_};
    const unsigned int limit_joint_count = std::min<unsigned int>(3, chain.getNrOfJoints());
    for (unsigned int i = 0; i < limit_joint_count; ++i)
    {
        if (q_out(i) < lower_limits[i] || q_out(i) > upper_limits[i])
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "IK seed: [%.6f, %.6f, %.6f]",
                q_init(0), q_init(1), q_init(2)
            );
            RCLCPP_ERROR(this->get_logger(), "q_out[%u]: %f", i, q_out(i));
            RCLCPP_ERROR(this->get_logger(), "goal pose exceeds joint limits");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    KDL::Frame reached_frame;
    if (fk_solver.JntToCart(q_out, reached_frame) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "FK failed for goal pose");
        return rclcpp_action::GoalResponse::REJECT;
    }

    const double dx = reached_frame.p.x() - target_pose.position.x;
    const double dy = reached_frame.p.y() - target_pose.position.y;
    const double dz = reached_frame.p.z() - target_pose.position.z;
    if ((dx * dx + dy * dy + dz * dz) > (this->kPosTolerance_ * this->kPosTolerance_))
    {
        RCLCPP_ERROR(this->get_logger(), "goal pose is not reachable within tolerance");
        return rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;
    if (!this->goal_active_.compare_exchange_strong(expected, true)) {
        RCLCPP_ERROR(this->get_logger(), "action server is active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    this->goal_pos_ = goal->goal_pos;

    RCLCPP_INFO(this->get_logger(), "arm start to move!");
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse IdeArmActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleArmMove> goal_handle
)
{
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

    RCLCPP_INFO(this->get_logger(), "Goal accepted. Start execution.");
    goal_handle->execute();
    std::thread{std::bind(&IdeArmActionServer::execute, this, _1), goal_handle}.detach();
}

void IdeArmActionServer::execute(const std::shared_ptr<GoalHandleArmMove> goal_handle)
{
    const std::shared_ptr<const nhk2026_msgs::action::ArmMove_Goal> goal = goal_handle->get_goal();
    ArmMove::Result::SharedPtr result = std::make_shared<ArmMove::Result>();
    nhk2026_msgs::srv::ArmPathPlan::Request::SharedPtr request = std::make_shared<nhk2026_msgs::srv::ArmPathPlan::Request>();
    request->goal_pos = goal->goal_pos;
    request->now_pos = this->now_pos_;
    request->now_joint = this->now_joint_;
    request->waypoints = goal->waypoints;
    request->max_speed = goal->max_speed;
    request->max_acc = goal->max_acc;
    request->control_frequency = this->control_frequency_;
    request->urdf.data = this->urdf_;

    auto future = this->path_client_->async_send_request(request);
    while (rclcpp::ok() && future.wait_for(10ms) != std::future_status::ready)
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
    }

    if (!rclcpp::ok())
    {
        result->success = false;
        result->msg = "rclcpp shutdown";
        goal_handle->abort(result);
        this->goal_active_ = false;
        this->active_goal_handle_.reset();
        return;
    }

    std_msgs::msg::Float32MultiArray j1_msg;
    std_msgs::msg::Float32MultiArray j2_msg;
    std_msgs::msg::Float32MultiArray j3_msg;
    j1_msg.data.resize(1);
    j2_msg.data.resize(1);
    j3_msg.data.resize(1);

    rclcpp::Rate loop_rate(static_cast<double>(this->control_frequency_));
    size_t i = 0;
    const auto response = future.get();
    if (!response || response->route.empty())
    {
        result->success = false;
        result->msg = "path plan failed";
        goal_handle->abort(result);
        this->goal_active_ = false;
        this->active_goal_handle_.reset();
        return;
    }

    while (rclcpp::ok())
    {
        if (i >= response->route.size())
        {
            i = response->route.size() - 1;
        }

        const auto &target_joint_state = response->route[i];
        if (
            target_joint_state.position.size() < 3 ||
            target_joint_state.name.size() != target_joint_state.position.size() ||
            this->joint_positions_.size() < 3
        )
        {
            result->success = false;
            result->msg = "joint state is invalid";
            goal_handle->abort(result);
            this->goal_active_ = false;
            this->active_goal_handle_.reset();
            return;
        }

        bool found_joint1 = false;
        bool found_joint2 = false;
        bool found_joint3 = false;
        std::vector<double> target_positions(3, 0.0);
        for (size_t joint_index = 0; joint_index < target_joint_state.name.size(); ++joint_index)
        {
            const auto &joint_name = target_joint_state.name[joint_index];
            const auto joint_position = static_cast<float>(target_joint_state.position[joint_index]);
            if (joint_name == this->joint1_name_)
            {
                j1_msg.data[0] = joint_position;
                target_positions[0] = target_joint_state.position[joint_index];
                found_joint1 = true;
            }
            else if (joint_name == this->joint2_name_)
            {
                j2_msg.data[0] = joint_position;
                target_positions[1] = target_joint_state.position[joint_index];
                found_joint2 = true;
            }
            else if (joint_name == this->joint3_name_)
            {
                j3_msg.data[0] = joint_position;
                target_positions[2] = target_joint_state.position[joint_index];
                found_joint3 = true;
            }
        }

        if (!found_joint1 || !found_joint2 || !found_joint3)
        {
            result->success = false;
            result->msg = "planned joint names are invalid";
            goal_handle->abort(result);
            this->goal_active_ = false;
            this->active_goal_handle_.reset();
            return;
        }

        this->j1_motor_publisher_->publish(j1_msg);
        this->j2_motor_publisher_->publish(j2_msg);
        this->j3_motor_publisher_->publish(j3_msg);

        const bool is_last_sample = (i + 1 >= response->route.size());
        if (!is_last_sample)
        {
            ++i;
        }
        else
        {
            bool reached = true;
            for (size_t joint_index = 0; joint_index < 3; ++joint_index)
            {
                if (std::abs(this->joint_positions_[joint_index] - target_positions[joint_index]) > this->kPosTolerance_)
                {
                    reached = false;
                    break;
                }
            }

            if (reached)
            {
                result->success = true;
                result->msg = "success";
                goal_handle->succeed(result);
                this->goal_active_ = false;
                this->active_goal_handle_.reset();
                return;
            }
        }

        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->msg = "goal canceled";
            goal_handle->canceled(result);
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
    q_current(0) = this->joint_positions_[0];
    q_current(1) = this->joint_positions_[1];
    q_current(2) = this->joint_positions_[2];

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

    double distance = std::sqrt(
        std::pow(current_pose.p.x() - this->goal_pos_.pose.position.x, 2) +
        std::pow(current_pose.p.y() - this->goal_pos_.pose.position.y, 2) +
        std::pow(current_pose.p.z() - this->goal_pos_.pose.position.z, 2)
    );

    feedback->pos_error = distance;

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

    if (!this->robot_description_flag_ || chain.getNrOfJoints() < 3)
    {
        return;
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::JntArray q_current(chain.getNrOfJoints());

    if (this->joint_positions_.size() < 3)
    {
        this->joint_positions_.resize(3);
    }

    std::vector<double> next_joint_positions(3);
    int flag = 0;

    const size_t joint_count = std::min(rxdata->name.size(), rxdata->position.size());
    for (size_t i = 0; i < joint_count; ++i)
    {
        const auto &joint_name = rxdata->name[i];
        if (joint_name == this->joint1_name_)
        {
            next_joint_positions[0] = rxdata->position[i];
            flag += 1;
        }
        else if (joint_name == this->joint2_name_)
        {
            next_joint_positions[1] = rxdata->position[i];
            flag += 1;
        }
        else if (joint_name == this->joint3_name_)
        {
            next_joint_positions[2] = rxdata->position[i];
            flag += 1;
        }
    }

    if (flag < 3) return;

    this->joint_positions_ = next_joint_positions;
    
    this->joint_subscribe_flag_ = true;
    this->now_joint_ = *rxdata;

    q_current(0) = this->joint_positions_[0];
    q_current(1) = this->joint_positions_[1];
    q_current(2) = this->joint_positions_[2];

    KDL::Frame current_pose;
    if (fk_solver.JntToCart(q_current, current_pose) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "FK failed in joint_state_callback.");
        return;
    }

    this->now_pos_.pose.position.x = current_pose.p.x();
    this->now_pos_.pose.position.y = current_pose.p.y();
    this->now_pos_.pose.position.z = current_pose.p.z();

    double ox, oy, oz, ow;
    current_pose.M.GetQuaternion(ox, oy, oz, ow);
    this->now_pos_.pose.orientation.w = ow;
    this->now_pos_.pose.orientation.x = ox;
    this->now_pos_.pose.orientation.y = oy;
    this->now_pos_.pose.orientation.z = oz;
    this->now_pos_.header.frame_id = "arm_base";
    this->now_pos_.header.stamp = this->get_clock()->now();
}

void IdeArmActionServer::robot_description_callback(const std_msgs::msg::String::SharedPtr rxdata)
{
    this->robot_description_flag_ = true;

    KDL::Tree tree;

    if (!kdl_parser::treeFromString(rxdata->data, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree.");
        return;
    }

    const std::string base_link = "arm_base";
    const std::string end_link = "tcp_link";

    if (!tree.getChain(base_link, end_link, chain)) {
        RCLCPP_ERROR(this->get_logger(), ("Failed to extract KDL chain from " + base_link + " to " + end_link).c_str());
        return;
    }

    this->urdf_ = rxdata->data;
    urdf::Model model;
    model.initString(rxdata->data);

    auto joint1 = model.getJoint(this->joint1_name_);
    auto joint2 = model.getJoint(this->joint2_name_);
    auto joint3 = model.getJoint(this->joint3_name_);

    this->j1_limit_up_ = joint1->limits->upper;
    this->j2_limit_up_ = joint2->limits->upper;
    this->j3_limit_up_ = joint3->limits->upper;

    this->j1_limit_down_ = joint1->limits->lower;
    this->j2_limit_down_ = joint2->limits->lower;
    this->j3_limit_down_ = joint3->limits->lower;

    RCLCPP_INFO(
        this->get_logger(),
        "joint limits: j1[%.6f, %.6f], j2[%.6f, %.6f], j3[%.6f, %.6f]",
        this->j1_limit_down_, this->j1_limit_up_,
        this->j2_limit_down_, this->j2_limit_up_,
        this->j3_limit_down_, this->j3_limit_up_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<IdeArmActionServer> node = std::make_shared<IdeArmActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
