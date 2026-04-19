#include "field_viz.hpp"

MarkerPublisher::MarkerPublisher()
: rclcpp::Node("marker_publisher")
{
    rclcpp::QoS marker_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    this->marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "field_marker",
        marker_qos
    );

    using namespace std::chrono_literals;
    this->pub_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&MarkerPublisher::pub_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "start to publish field model");
}

void MarkerPublisher::pub_timer_callback()
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();

    marker.ns = "field";
    marker.id = 0;


    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://nhk2026_viz/meshes/field_color.glb";
    marker.mesh_use_embedded_materials = true;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(M_PI / 2.0, M_PI, M_PI);  // roll, pitch, yaw

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(1);  // 0 = 永続

    marker_pub_->publish(marker);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}