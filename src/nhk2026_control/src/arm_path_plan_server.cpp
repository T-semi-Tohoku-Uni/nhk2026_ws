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

    rclcpp::QoS robot = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    this->robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        std::string("robot_description"),
        robot,
        std::bind(&ArmPathPlan::robot_description_callback, this, std::placeholders::_1)
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
    std::pair<double, double> start = {request->now_pos.pose.position.y, request->now_pos.pose.position.z};
    std::pair<double, double> goal  = {request->goal_pos.pose.position.y, request->goal_pos.pose.position.z};

    std::vector<std::pair<double, double>> path;

    std::vector<std::pair<double, double>> all_points;
    all_points.push_back(start);
    for (size_t i = 0; i < request->waypoints.size(); i ++)
    {
        std::pair<double, double> waypoints_path_one;
        waypoints_path_one.first = request->waypoints[i].pose.position.y;
        waypoints_path_one.second = request->waypoints[i].pose.position.z;
        all_points.push_back(waypoints_path_one);
    }
    all_points.push_back(goal);

    if (!all_points.empty()) {
        for (size_t i = 0; i + 1 < all_points.size(); i++) {
            std::vector<std::pair<double, double>> segment = this->generator(all_points[i], all_points[i + 1]);
            if (i == 0) {
                path.insert(path.end(), segment.begin(), segment.end());
            } else {
                path.insert(path.end(), segment.begin() + 1, segment.end());
            }
        }
    }

    sensor_msgs::msg::JointState msg;
    msg.header.frame_id = "arm_base";
    msg.header.stamp = this->now();
}

std::vector<std::pair<double, double>> ArmPathPlan::generator(
    std::pair<double, double> start_point,
    std::pair<double, double> goal_point
)
{
    std::vector<std::pair<double, double>> path;

    double sx = start_point.first;
    double sy = start_point.second;
    double gx = goal_point.first;
    double gy = goal_point.second;

    double dist = std::hypot(gx - sx, gy - sy);
    double step = 0.005;
    int n = std::max(2, static_cast<int>(dist / step) + 1);

    for (int i = 0; i < n; i++) {
        double t = static_cast<double>(i) / (n - 1);
        path.push_back({
            sx + t * (gx - sx),
            sy + t * (gy - sy)
        });
    }

    return path;
}

void ArmPathPlan::robot_description_callback(const std_msgs::msg::String::SharedPtr rxdata)
{
    this->robot_description_flag_ = true;

    KDL::Tree tree;

    if (!kdl_parser::treeFromString(rxdata->data, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree.");
    }

    const std::string base_link = "arm_base";
    const std::string end_link = "tcp_link";

    if (!tree.getChain(base_link, end_link, this->chain_)) {
        RCLCPP_ERROR(this->get_logger(), ("Failed to extract KDL chain from " + base_link + " to " + end_link).c_str());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ArmPathPlan> node = std::make_shared<ArmPathPlan>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
