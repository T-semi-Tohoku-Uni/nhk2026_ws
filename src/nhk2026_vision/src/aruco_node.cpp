#include "aruco_node.hpp"

ArucoNode::ArucoNode()
: rclcpp_lifecycle::LifecycleNode(std::string("aruco_node"))
{   
    this->declare_parameter<double>("marker_size", 1.0);
    this->aruco_detector = std::make_unique<ArucoDetect>();
}

ArucoNode::CallbackReturn ArucoNode::on_configure(const rclcpp_lifecycle::State & state)
{
    this->markersize = this->get_parameter("marker_size").as_double();

    this->aruco_pose_publisher = this->create_publisher<nhk2026_msgs::msg::ArucoPose>(
        std::string("aruco_pose"),
        rclcpp::SystemDefaultsQoS()
    );
    this->parameter_callback_hanle_ = this->add_on_set_parameters_callback(
        std::bind(&ArucoNode::parameters_callback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

ArucoNode::CallbackReturn ArucoNode::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
    this->img_subscriber = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
        std::string("/camera/camera/rgbd"),
        rclcpp::SensorDataQoS(),
        std::bind(&ArucoNode::img_callback, this, _1)
    );
    this->aruco_pose_publisher->on_activate();
    return CallbackReturn::SUCCESS;
}

ArucoNode::CallbackReturn ArucoNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
    this->aruco_pose_publisher->on_deactivate();
    this->img_subscriber.reset();
    return CallbackReturn::SUCCESS;
}

ArucoNode::CallbackReturn ArucoNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
    this->img_subscriber.reset();
    this->aruco_pose_publisher.reset();
    this->parameter_callback_hanle_.reset();
    return CallbackReturn::SUCCESS;
}

ArucoNode::CallbackReturn ArucoNode::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(this->get_logger(), "on_error() called. Previous state: %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

ArucoNode::CallbackReturn ArucoNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(
        this->get_logger(), "on_shutdown() called. Previous state: %s",
        state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void ArucoNode::img_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr rxdata)
{
    cv::Mat img = cv_bridge::toCvCopy(rxdata->rgb, "bgr8")->image;
    cv::Mat cameramatrix = cv::Mat(3, 3, CV_64F, (void*)rxdata->rgb_camera_info.k.data()).clone();
    cv::Mat distcoeffs = cv::Mat(rxdata->rgb_camera_info.d).clone();
    std::vector<std::vector<cv::Mat>> aruco_data = this->aruco_detector->detect(img, this->markersize, cameramatrix, distcoeffs);
    
    nhk2026_msgs::msg::ArucoPose txdata;

    txdata.pose.header = rxdata->header;
    for (size_t i = 0; i < aruco_data.size(); i++)
    {
        if (this->aruco_pose_publisher->is_activated())
        {
            txdata.id = aruco_data[i][0].at<int>(0, 0);
            txdata.pose.pose.position.x = aruco_data[i][2].at<double>(0, 0);
            txdata.pose.pose.position.y = aruco_data[i][2].at<double>(1, 0);
            txdata.pose.pose.position.z = aruco_data[i][2].at<double>(2, 0);

            cv::Mat rotation_matrix;
            cv::Rodrigues(aruco_data[i][1], rotation_matrix);
            Eigen::Matrix3d eigen_rotation_matrix;
            eigen_rotation_matrix << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                                    rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                                    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);
            Eigen::Quaterniond quaternion(eigen_rotation_matrix);
            txdata.pose.pose.orientation.x = quaternion.x();
            txdata.pose.pose.orientation.y = quaternion.y();
            txdata.pose.pose.orientation.z = quaternion.z();
            txdata.pose.pose.orientation.w = quaternion.w();
            aruco_pose_publisher->publish(txdata);
        }
    }
}

rcl_interfaces::msg::SetParametersResult ArucoNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters
)
{
    rcl_interfaces::msg::SetParametersResult result;

    auto st = this->get_current_state().id();
    if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        result.successful = false;
        result.reason = "aruco_node is active";
        return result;
    }

    for (const auto &param : parameters) {
        if (param.get_name() == "marker_size")
        {
            this->markersize = param.as_double();
            RCLCPP_INFO(this->get_logger(), "marker_size updated to: %f", this->markersize);
        }
    }
    result.successful = true;
    result.reason = "success";
    return result;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ArucoNode> node = std::make_shared<ArucoNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}