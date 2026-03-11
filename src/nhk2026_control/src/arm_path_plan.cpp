#include "arm_path_plan.hpp"

ArmPathPlan::ArmPathPlan()
: rclcpp::Node("arm_path_plan")
{
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

    std::vector<std::pair<double, double>> path = this->generator(start, goal);

    nav_msgs::msg::Path msg;
    msg.header.frame_id = "arm_base";
    msg.header.stamp = this->now();

    for (std::pair<double, double> &p : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = -0.182751;
        pose.pose.position.y = p.first;
        pose.pose.position.z = p.second;
        msg.poses.push_back(pose);
    }

    response->route = msg;
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