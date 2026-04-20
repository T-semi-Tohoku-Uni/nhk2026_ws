#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "nhk2026_msgs/msg/aruco_pose.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{

struct MarkerPose
{
    int id;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
};

std::vector<cv::Point3f> make_object_points(const double marker_size)
{
    const float half_size = static_cast<float>(marker_size / 2.0);
    return {
        cv::Point3f(-half_size,  half_size, 0.0F),
        cv::Point3f( half_size,  half_size, 0.0F),
        cv::Point3f( half_size, -half_size, 0.0F),
        cv::Point3f(-half_size, -half_size, 0.0F),
    };
}

double calculate_reprojection_error(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & image_points,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec)
{
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, reprojected_points);

    double total_error = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        total_error += cv::norm(image_points[i] - reprojected_points[i]);
    }

    return total_error / static_cast<double>(image_points.size());
}

cv::Mat make_camera_matrix(const realsense2_camera_msgs::msg::RGBD::SharedPtr & rgbd)
{
    const auto & k = rgbd->rgb_camera_info.k;
    return (cv::Mat_<double>(3, 3) <<
        k[0], k[1], k[2],
        k[3], k[4], k[5],
        k[6], k[7], k[8]);
}

cv::Mat make_dist_coeffs(const realsense2_camera_msgs::msg::RGBD::SharedPtr & rgbd)
{
    const auto & d = rgbd->rgb_camera_info.d;
    cv::Mat dist_coeffs(static_cast<int>(d.size()), 1, CV_64F);

    for (size_t i = 0; i < d.size(); ++i) {
        dist_coeffs.at<double>(static_cast<int>(i), 0) = d[i];
    }

    return dist_coeffs;
}

Eigen::Quaterniond rotation_matrix_to_quaternion(const cv::Vec3d & rvec)
{
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    Eigen::Matrix3d eigen_rotation_matrix;
    eigen_rotation_matrix <<
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);

    return Eigen::Quaterniond(eigen_rotation_matrix);
}

}  // namespace

class ArucoNodeOpenCv410 : public rclcpp_lifecycle::LifecycleNode
{
public:
    ArucoNodeOpenCv410()
    : rclcpp_lifecycle::LifecycleNode("aruco_node"),
      dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100))
    {
        this->declare_parameter<double>("marker_size", 1.0);

        detector_parameters_.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_ = std::make_unique<cv::aruco::ArucoDetector>(dictionary_, detector_parameters_);
    }

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state)
    {
        markersize_ = this->get_parameter("marker_size").as_double();

        aruco_pose_publisher_ = this->create_publisher<nhk2026_msgs::msg::ArucoPose>(
            "aruco_pose",
            rclcpp::SystemDefaultsQoS()
        );
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ArucoNodeOpenCv410::parameters_callback, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
        img_subscriber_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
            "/camera/camera/rgbd",
            rclcpp::SensorDataQoS(),
            std::bind(&ArucoNodeOpenCv410::img_callback, this, _1)
        );
        aruco_pose_publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
        aruco_pose_publisher_->on_deactivate();
        img_subscriber_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(this->get_logger(), "Previous state: %s", state.label().c_str());
        img_subscriber_.reset();
        aruco_pose_publisher_.reset();
        parameter_callback_handle_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(this->get_logger(), "on_error() called. Previous state: %s", state.label().c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(
            this->get_logger(), "on_shutdown() called. Previous state: %s",
            state.label().c_str());
        return CallbackReturn::SUCCESS;
    }

    void img_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr rgbd)
    {
        if (!aruco_pose_publisher_ || !aruco_pose_publisher_->is_activated()) {
            return;
        }

        const cv::Mat image = cv_bridge::toCvCopy(rgbd->rgb, "bgr8")->image;
        const cv::Mat camera_matrix = make_camera_matrix(rgbd);
        const cv::Mat dist_coeffs = make_dist_coeffs(rgbd);
        const auto detections = detect(image, markersize_, camera_matrix, dist_coeffs);

        nhk2026_msgs::msg::ArucoPose txdata;
        txdata.pose.header = rgbd->header;

        for (const auto & detection : detections)
        {
            txdata.id = detection.id;
            txdata.pose.pose.position.x = detection.tvec[0];
            txdata.pose.pose.position.y = detection.tvec[1];
            txdata.pose.pose.position.z = detection.tvec[2];

            const auto quaternion = rotation_matrix_to_quaternion(detection.rvec);
            txdata.pose.pose.orientation.x = quaternion.x();
            txdata.pose.pose.orientation.y = quaternion.y();
            txdata.pose.pose.orientation.z = quaternion.z();
            txdata.pose.pose.orientation.w = quaternion.w();

            aruco_pose_publisher_->publish(txdata);
        }
    }

    std::vector<MarkerPose> detect(
        const cv::Mat & image,
        const double marker_size,
        const cv::Mat & camera_matrix,
        const cv::Mat & dist_coeffs) const
    {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<std::vector<cv::Point2f>> rejected_candidates;
        std::vector<int> ids;
        detector_->detectMarkers(image, corners, ids, rejected_candidates);

        std::vector<MarkerPose> results;
        if (ids.empty()) {
            return results;
        }

        const auto object_points = make_object_points(marker_size);
        results.reserve(ids.size());

        for (size_t i = 0; i < ids.size(); ++i)
        {
            cv::Vec3d rvec;
            cv::Vec3d tvec;
            const bool solved = cv::solvePnP(
                object_points,
                corners.at(i),
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
                false,
                cv::SOLVEPNP_IPPE_SQUARE
            );

            if (!solved) {
                continue;
            }

            const double error = calculate_reprojection_error(
                object_points,
                corners.at(i),
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec
            );

            if (error > 2.0) {
                continue;
            }

            results.push_back(MarkerPose{ids.at(i), rvec, tvec});
        }

        return results;
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        const auto state_id = this->get_current_state().id();
        if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            result.successful = false;
            result.reason = "aruco_node is active";
            return result;
        }

        for (const auto & parameter : parameters) {
            if (parameter.get_name() == "marker_size") {
                markersize_ = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "marker_size updated to: %f", markersize_);
            }
        }

        result.successful = true;
        result.reason = "success";
        return result;
    }

    cv::aruco::Dictionary dictionary_;
    cv::aruco::DetectorParameters detector_parameters_;
    std::unique_ptr<cv::aruco::ArucoDetector> detector_;

    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr img_subscriber_;
    rclcpp_lifecycle::LifecyclePublisher<nhk2026_msgs::msg::ArucoPose>::SharedPtr aruco_pose_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    double markersize_ = 1.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoNodeOpenCv410>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
