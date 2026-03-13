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

    nav_msgs::msg::Path msg;
    msg.header.frame_id = "arm_base";
    msg.header.stamp = this->now();
    constexpr double roll = -1.57;
    constexpr double half_roll = roll * 0.5;
    constexpr double qx = std::sin(half_roll);
    constexpr double qw = std::cos(half_roll);

    for (std::pair<double, double> &p : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = -0.182751;
        pose.pose.position.y = p.first;
        pose.pose.position.z = p.second;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = qw;
        msg.poses.push_back(pose);
    }

    response->route = msg;
    this->route_publisher_->publish(msg);
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ArmPathPlan> node = std::make_shared<ArmPathPlan>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
