#include "nhk2026_control/ide_arm_teaching.hpp"

IdeArmTeaching::IdeArmTeaching()
: rclcpp::Node("ide_arm_teaching")
{
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

std::vector<double> IdeArmTeaching::listen_transform()
{
    geometry_msgs::msg::TransformStamped t;
    std::vector<double> empty;

    if (!tf_buffer_->canTransform(
            "tcp_link", "arm_base", tf2::TimePointZero, tf2::durationFromSec(3.0))) {
        RCLCPP_WARN(this->get_logger(),
                    "Timed out waiting for transform arm_base -> tcp_link");
        return empty;
    }

    try
    {
        t = tf_buffer_->lookupTransform(
            "tcp_link", "arm_base",
            tf2::TimePointZero
        );
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform base_link to dynamic_frame: %s",
            ex.what());
        return empty;
    }
    
    
    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
        this->get_logger(), "base_link->dynamic_frame: %f %f %f",
        t.transform.translation.y, t.transform.translation.z, roll);
    std::vector<double> zahyo = {t.transform.translation.y, t.transform.translation.z, roll};
    return zahyo;
}

int main(int argc, char * argv[])
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << "arm_pos" << "_"
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
    file << "name, y, z, roll\n";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<IdeArmTeaching>();

    std::string pos_name;
    while (rclcpp::ok())
    {
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
    

    rclcpp::shutdown();
    return 0;
}