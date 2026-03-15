#include "arm_path_plan_server.hpp"

ArmPathPlan::ArmPathPlan()
: rclcpp::Node("arm_path_plan")
{
    rclcpp::QoS route_qos(rclcpp::KeepLast(1));
    route_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    route_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    this->route_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        std::string("arm_route"),
        route_qos
    );
    
    this->arm_path_service_ = this->create_service<nhk2026_msgs::srv::ArmPathPlan>(
        std::string("arm_path"),
        std::bind(&ArmPathPlan::path_gen_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Create arm_path service server.");
}

void ArmPathPlan::path_gen_callback(
    const std::shared_ptr<nhk2026_msgs::srv::ArmPathPlan::Request> request,
    std::shared_ptr<nhk2026_msgs::srv::ArmPathPlan::Response> response
)
{
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(request->urdf.data, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree.");
        return;
    }

    const std::string base_link = "arm_base";
    const std::string end_link = "tcp_link";

    KDL::Chain chain;
    if (!tree.getChain(base_link, end_link, chain)) {
        RCLCPP_ERROR(this->get_logger(), ("Failed to extract KDL chain from " + base_link + " to " + end_link).c_str());
        return;
    }

    geometry_msgs::msg::PoseStamped now_pos = request->now_pos;
    geometry_msgs::msg::PoseStamped goal_pos = request->goal_pos;
    const auto pose_to_frame = [](const geometry_msgs::msg::PoseStamped & pose) {
        return KDL::Frame(
            KDL::Rotation::Quaternion(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ),
            KDL::Vector(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            )
        );
    };

    KDL::ChainIkSolverPos_LMA ik_solver(chain);
    const auto joint_count = chain.getNrOfJoints();
    KDL::JntArray seed(joint_count);
    KDL::JntArray now_joints(joint_count);
    KDL::JntArray goal_joints(joint_count);

    if (ik_solver.CartToJnt(seed, pose_to_frame(now_pos), now_joints) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to solve IK for now_pos.");
        return;
    }

    if (ik_solver.CartToJnt(now_joints, pose_to_frame(goal_pos), goal_joints) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to solve IK for goal_pos.");
        return;
    }

    std::vector<std::string> joint_names;
    joint_names.reserve(joint_count);
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const auto & joint = chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
            joint_names.push_back(joint.getName());
        }
    }

    int step = 100;

    sensor_msgs::msg::JointState msg;
    msg.header.frame_id = "arm_base";
    msg.header.stamp = this->now();
    msg.name = joint_names;

    response->route.clear();
    response->route.reserve(step + 1);
    
    nav_msgs::msg::Path path;
    path.header.frame_id = "arm_base";
    path.header.stamp = msg.header.stamp;
    path.poses.reserve(step + 1);

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::JntArray interpolated_joints(joint_count);
    msg.position.resize(joint_count);

    for (int i = 0; i <= step; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(step);

        for (unsigned int joint_index = 0; joint_index < joint_count; ++joint_index) {
            interpolated_joints(joint_index) =
                now_joints(joint_index) +
                (goal_joints(joint_index) - now_joints(joint_index)) * ratio;
            msg.position[joint_index] = interpolated_joints(joint_index);
        }

        KDL::Frame tcp_frame;
        if (fk_solver.JntToCart(interpolated_joints, tcp_frame) >= 0) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = tcp_frame.p.x();
            pose.pose.position.y = tcp_frame.p.y();
            pose.pose.position.z = tcp_frame.p.z();
            tcp_frame.M.GetQuaternion(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );
            path.poses.push_back(std::move(pose));
        }

        response->route.push_back(msg);
    }

    this->route_publisher_->publish(path);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ArmPathPlan> node = std::make_shared<ArmPathPlan>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
