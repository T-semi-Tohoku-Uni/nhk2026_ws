#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// PCL関連
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

namespace detection {

class ObjectDetection : public rclcpp::Node {
public:
    ObjectDetection() : Node("object_detection"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // QoS設定
        auto lidar_qos = rclcpp::SensorDataQoS();
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar", lidar_qos, std::bind(&ObjectDetection::pointCloudCallback, this, std::placeholders::_1)
        );

        sub_pose_ = create_subscription<geometry_msgs::msg::Pose>(
            "pose", rclcpp::QoS(10), std::bind(&ObjectDetection::odomCallback, this, std::placeholders::_1)
        );

        // 立方体表示用のパブリッシャー
        pub_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("detected_objects", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. 座標変換 (Sensor -> base_footprint)
        sensor_msgs::msg::PointCloud2 cloud_out;
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "base_footprint", msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*msg, cloud_out, transform);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", ex.what());
            return;
        }

        // 2. PCL型への変換とフィルタリング
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_out, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_out, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) continue;

            double dist_sq = (*iter_x)*(*iter_x) + (*iter_y)*(*iter_y);
            if (dist_sq < 0.45*0.45 || dist_sq > 5.0*5.0) continue; // 距離フィルタ

            double z_map = *iter_z + pose_.position.z;
            // 地面・棚の高さ除去（ユーザーのロジックを継承）
            if (std::abs(z_map - 0.00) <= 0.05 || std::abs(z_map - 0.20) <= 0.05 || std::abs(z_map - 0.40) <= 0.05) continue;
            if (*iter_z > 1.0) continue;

            pcl_cloud->points.emplace_back(*iter_x, *iter_y, *iter_z);
        }

        if (pcl_cloud->empty()) return;

        // 3. クラスタリング (Euclidean Cluster Extraction)
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pcl_cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.15); // 15cm以内の点を結合
        ec.setMinClusterSize(30);     // 最小点数
        ec.setMaxClusterSize(2000);   // 最大点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_cloud);
        ec.extract(cluster_indices);

        // 4. 立方体（Bounding Box）の生成と可視化
        publishMarkers(pcl_cloud, cluster_indices, msg->header.stamp);
    }

    void publishMarkers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                        const std::vector<pcl::PointIndices>& clusters,
                        const rclcpp::Time& stamp) 
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto& indices : clusters) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices) cluster_cloud->points.push_back(cloud->points[idx]);

            // AABB (Axis Aligned Bounding Box) の計算
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_footprint";
            marker.header.stamp = stamp;
            marker.ns = "objects";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // 中心座標
            marker.pose.position.x = (max_pt.x + min_pt.x) / 2.0;
            marker.pose.position.y = (max_pt.y + min_pt.y) / 2.0;
            marker.pose.position.z = (max_pt.z + min_pt.z) / 2.0;
            marker.pose.orientation.w = 1.0;

            // サイズ
            marker.scale.x = std::max(0.1, (double)(max_pt.x - min_pt.x));
            marker.scale.y = std::max(0.1, (double)(max_pt.y - min_pt.y));
            marker.scale.z = std::max(0.1, (double)(max_pt.z - min_pt.z));

            // 色 (半透明の緑)
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.5f;
            
            marker.lifetime = rclcpp::Duration::from_seconds(0.2); // 次のフレームまで維持
            marker_array.markers.push_back(marker);
        }
        pub_marker_->publish(marker_array);
    }

    void odomCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        pose_ = *msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

    geometry_msgs::msg::Pose pose_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
} // namespace detection

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<detection::ObjectDetection>());
    rclcpp::shutdown();
    return 0;
}