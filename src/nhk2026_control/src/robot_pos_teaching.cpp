#include "nhk2026_control/robot_pos_teaching.hpp"

RobotPosTeaching::RobotPosTeaching()
: rclcpp::Node("robot_pos_teaching")
{
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

std::vector<double> RobotPosTeaching::listen_transform()
{
    geometry_msgs::msg::TransformStamped t;
    std::vector<double> empty;

    if (!tf_buffer_->canTransform(
            "base_link","base_footprint",  tf2::TimePointZero, tf2::durationFromSec(3.0))) {
        RCLCPP_WARN(this->get_logger(),
                    "Timed out waiting for transform base_link -> base_footprint");
        return empty;
    }

    try
    {
        t = tf_buffer_->lookupTransform(
            "base_link","base_footprint",
            tf2::TimePointZero
        );
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform base_link to base_footprint: %s",
            ex.what());
        return empty;
    }
    
    
    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
        this->get_logger(), "base_link->base_footprint: %f %f %f",
        t.transform.translation.x, t.transform.translation.y, yaw);
    std::vector<double> zahyo = {t.transform.translation.x, t.transform.translation.y, yaw};
    return zahyo;
}

int main(int argc, char * argv[])
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << "robot_pos" << "_"
        << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
        << ".csv";
    std::string filename = oss.str();
    const std::filesystem::path output_dir =
    std::filesystem::path(NHK2026_CONTROL_SOURCE_DIR) / "config";
    std::filesystem::create_directories(output_dir);

    const std::filesystem::path output_path = output_dir / filename;
    std::ofstream file(output_path);
    if (!file) {
        std::cerr << "ファイルを作成できませんでした\n";
        return 1;
    }
    file << "name, x, y, yaw\n";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPosTeaching>();

    std::string pos_name;
    while (rclcpp::ok())
    {
        std::cout << "name?";
        if (!(std::cin >> pos_name)) {
            break;
        }

        if (!rclcpp::ok()) {
            break;
        }

        std::vector<double> zahyo = node->listen_transform();
        if (zahyo.size() != 3) continue;
        file << pos_name << ","
             << std::to_string(zahyo[0]) << ","
             << std::to_string(zahyo[1]) << ","
             << std::to_string(zahyo[2]) << "\n";
    }

    file.close();
    rclcpp::shutdown();
    return 0;
}