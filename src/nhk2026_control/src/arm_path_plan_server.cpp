#include "arm_path_plan_server.hpp"

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <urdf/model.h>

namespace
{

constexpr double kEpsilon = 1e-6;

double position_distance(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to)
{
    const double dx = to.pose.position.x - from.pose.position.x;
    const double dy = to.pose.position.y - from.pose.position.y;
    const double dz = to.pose.position.z - from.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

tf2::Quaternion pose_quaternion(const geometry_msgs::msg::PoseStamped & pose)
{
    const auto & orientation = pose.pose.orientation;
    const double norm =
        std::abs(orientation.x) +
        std::abs(orientation.y) +
        std::abs(orientation.z) +
        std::abs(orientation.w);

    if (norm <= kEpsilon)
    {
        return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }

    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
    q.normalize();
    return q;
}

geometry_msgs::msg::PoseStamped interpolate_pose(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    const double ratio,
    const std_msgs::msg::Header & header)
{
    geometry_msgs::msg::PoseStamped interpolated;
    interpolated.header = header;
    interpolated.pose.position.x =
        from.pose.position.x + (to.pose.position.x - from.pose.position.x) * ratio;
    interpolated.pose.position.y =
        from.pose.position.y + (to.pose.position.y - from.pose.position.y) * ratio;
    interpolated.pose.position.z =
        from.pose.position.z + (to.pose.position.z - from.pose.position.z) * ratio;

    const tf2::Quaternion q_from = pose_quaternion(from);
    const tf2::Quaternion q_to = pose_quaternion(to);
    const tf2::Quaternion q = q_from.slerp(q_to, ratio);
    interpolated.pose.orientation.x = q.x();
    interpolated.pose.orientation.y = q.y();
    interpolated.pose.orientation.z = q.z();
    interpolated.pose.orientation.w = q.w();
    return interpolated;
}

std::vector<double> sample_path_distance(
    const double total_distance,
    const double max_speed,
    const double max_acc,
    const double dt)
{
    std::vector<double> samples;
    samples.push_back(0.0);

    if (total_distance <= kEpsilon)
    {
        return samples;
    }

    const double safe_speed = std::max(max_speed, kEpsilon);
    const double safe_acc = std::max(max_acc, kEpsilon);

    double t_acc = safe_speed / safe_acc;
    double d_acc = 0.5 * safe_acc * t_acc * t_acc;
    double t_cruise = 0.0;
    double peak_speed = safe_speed;
    bool triangular = false;

    if ((2.0 * d_acc) >= total_distance)
    {
        triangular = true;
        t_acc = std::sqrt(total_distance / safe_acc);
        d_acc = 0.5 * safe_acc * t_acc * t_acc;
        peak_speed = safe_acc * t_acc;
    }
    else
    {
        t_cruise = (total_distance - (2.0 * d_acc)) / safe_speed;
    }

    const double total_time = triangular ? (2.0 * t_acc) : (2.0 * t_acc + t_cruise);
    const size_t sample_count = std::max<size_t>(1, static_cast<size_t>(std::ceil(total_time / dt)));
    samples.reserve(sample_count + 1);

    for (size_t i = 1; i <= sample_count; ++i)
    {
        const double t = std::min(static_cast<double>(i) * dt, total_time);
        double distance = 0.0;

        if (t <= t_acc)
        {
            distance = 0.5 * safe_acc * t * t;
        }
        else if (!triangular && t <= (t_acc + t_cruise))
        {
            distance = d_acc + safe_speed * (t - t_acc);
        }
        else
        {
            const double t_decel = triangular ? (t - t_acc) : (t - t_acc - t_cruise);
            const double decel_start_distance = triangular ? d_acc : (d_acc + safe_speed * t_cruise);
            distance = decel_start_distance + peak_speed * t_decel - 0.5 * safe_acc * t_decel * t_decel;
        }

        samples.push_back(std::min(distance, total_distance));
    }

    if ((total_distance - samples.back()) > kEpsilon)
    {
        samples.push_back(total_distance);
    }
    else
    {
        samples.back() = total_distance;
    }

    return samples;
}

geometry_msgs::msg::PoseStamped sample_pose_on_path(
    const std::vector<geometry_msgs::msg::PoseStamped> & control_points,
    const std::vector<double> & cumulative_lengths,
    const double distance,
    const std_msgs::msg::Header & header)
{
    if (control_points.empty())
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        return pose;
    }

    if (distance <= kEpsilon)
    {
        geometry_msgs::msg::PoseStamped pose = control_points.front();
        pose.header = header;
        return pose;
    }

    const double total_length = cumulative_lengths.back();
    if ((total_length - distance) <= kEpsilon)
    {
        geometry_msgs::msg::PoseStamped pose = control_points.back();
        pose.header = header;
        return pose;
    }

    const auto upper = std::upper_bound(
        cumulative_lengths.begin(),
        cumulative_lengths.end(),
        distance);
    size_t segment_index = static_cast<size_t>(std::distance(cumulative_lengths.begin(), upper));
    segment_index = std::max<size_t>(1, segment_index) - 1;

    const double segment_start = cumulative_lengths[segment_index];
    const double segment_end = cumulative_lengths[segment_index + 1];
    const double segment_length = segment_end - segment_start;
    if (segment_length <= kEpsilon)
    {
        geometry_msgs::msg::PoseStamped pose = control_points[segment_index + 1];
        pose.header = header;
        return pose;
    }

    const double ratio = (distance - segment_start) / segment_length;
    return interpolate_pose(
        control_points[segment_index],
        control_points[segment_index + 1],
        ratio,
        header);
}

}  // namespace

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

    const auto joint_count = chain.getNrOfJoints();
    if (joint_count == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No joints found in chain.");
        return;
    }

    geometry_msgs::msg::PoseStamped now_pos = request->now_pos;
    geometry_msgs::msg::PoseStamped goal_pos = request->goal_pos;
    const auto pose_to_frame = [](const geometry_msgs::msg::PoseStamped & pose) {
        const auto q = pose_quaternion(pose);
        return KDL::Frame(
            KDL::Rotation::Quaternion(
                q.x(),
                q.y(),
                q.z(),
                q.w()
            ),
            KDL::Vector(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            )
        );
    };

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_vel_solver(chain);
    KDL::JntArray q_min(joint_count);
    KDL::JntArray q_max(joint_count);
    for (unsigned int i = 0; i < joint_count; ++i)
    {
        q_min(i) = -std::numeric_limits<double>::max();
        q_max(i) = std::numeric_limits<double>::max();
    }

    KDL::JntArray now_joints(joint_count);
    KDL::JntArray sampled_joints(joint_count);

    std::vector<std::string> joint_names;
    joint_names.reserve(joint_count);
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const auto & joint = chain.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
            joint_names.push_back(joint.getName());
        }
    }

    urdf::Model model;
    if (model.initString(request->urdf.data))
    {
        for (unsigned int i = 0; i < joint_names.size(); ++i)
        {
            const auto joint = model.getJoint(joint_names[i]);
            if (joint && joint->limits)
            {
                q_min(i) = joint->limits->lower;
                q_max(i) = joint->limits->upper;
            }
        }
    }

    KDL::ChainIkSolverPos_NR_JL ik_solver(
        chain,
        q_min,
        q_max,
        fk_solver,
        ik_vel_solver,
        100,
        1e-6);

    bool have_current_joint_seed = false;
    if (request->now_joint.name.size() == request->now_joint.position.size())
    {
        have_current_joint_seed = true;
        for (unsigned int i = 0; i < joint_names.size(); ++i)
        {
            bool found = false;
            for (size_t joint_index = 0; joint_index < request->now_joint.name.size(); ++joint_index)
            {
                if (request->now_joint.name[joint_index] == joint_names[i])
                {
                    now_joints(i) = request->now_joint.position[joint_index];
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                have_current_joint_seed = false;
                break;
            }
        }
    }

    if (!have_current_joint_seed)
    {
        KDL::JntArray seed(joint_count);
        if (ik_solver.CartToJnt(seed, pose_to_frame(now_pos), now_joints) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to solve IK for now_pos.");
            return;
        }
    }

    const int control_frequency = std::max(1, request->control_frequency);
    const double dt = 1.0 / static_cast<double>(control_frequency);
    const double max_speed = std::max(static_cast<double>(request->max_speed), 1e-3);
    const double max_acc = std::max(static_cast<double>(request->max_acc), 1e-3);

    std::vector<geometry_msgs::msg::PoseStamped> control_points;
    control_points.reserve(request->waypoints.size() + 2);
    control_points.push_back(now_pos);
    control_points.insert(control_points.end(), request->waypoints.begin(), request->waypoints.end());
    control_points.push_back(goal_pos);

    std::vector<double> cumulative_lengths;
    cumulative_lengths.reserve(control_points.size());
    cumulative_lengths.push_back(0.0);
    for (size_t i = 1; i < control_points.size(); ++i)
    {
        cumulative_lengths.push_back(
            cumulative_lengths.back() + position_distance(control_points[i - 1], control_points[i]));
    }

    const double total_distance = cumulative_lengths.back();
    const std::vector<double> sampled_distances = sample_path_distance(
        total_distance,
        max_speed,
        max_acc,
        dt);

    sensor_msgs::msg::JointState msg;
    msg.header.frame_id = "arm_base";
    msg.header.stamp = this->now();
    msg.name = joint_names;

    response->route.clear();
    response->route.reserve(std::max<size_t>(sampled_distances.size(), control_points.size()));
    
    nav_msgs::msg::Path path;
    path.header.frame_id = "arm_base";
    path.header.stamp = msg.header.stamp;
    path.poses.reserve(response->route.capacity());

    msg.position.resize(joint_count);

    auto append_joint_sample =
        [&](const KDL::JntArray & joints)
        {
            for (unsigned int joint_index = 0; joint_index < joint_count; ++joint_index)
            {
                msg.position[joint_index] = joints(joint_index);
            }

            KDL::Frame tcp_frame;
            if (fk_solver.JntToCart(joints, tcp_frame) >= 0)
            {
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
        };

    append_joint_sample(now_joints);

    if (total_distance <= kEpsilon)
    {
        for (size_t control_point_index = 1; control_point_index < control_points.size(); ++control_point_index)
        {
            if (ik_solver.CartToJnt(now_joints, pose_to_frame(control_points[control_point_index]), sampled_joints) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to solve IK for low-distance Cartesian sample.");
                response->route.clear();
                return;
            }

            append_joint_sample(sampled_joints);
            now_joints = sampled_joints;
        }
    }
    else
    {
        for (size_t sample_index = 1; sample_index < sampled_distances.size(); ++sample_index)
        {
            const auto sampled_pose = sample_pose_on_path(
                control_points,
                cumulative_lengths,
                sampled_distances[sample_index],
                path.header);

            if (ik_solver.CartToJnt(now_joints, pose_to_frame(sampled_pose), sampled_joints) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to solve IK for sampled Cartesian point.");
                response->route.clear();
                return;
            }

            append_joint_sample(sampled_joints);
            now_joints = sampled_joints;
        }
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
