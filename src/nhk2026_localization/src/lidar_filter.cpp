#include <rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lidar_filter{
    class Lidar_filter: public rclcpp::Node{
        public:
            Lidar_filter():Node("lidar_filter"){
                auto lidarScanqos = rclcpp::SensorDataQoS();
                subScanFront_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan_front",lidarScanqos,[this](const sensor_msgs::msg::LaserScan::SharedPtr msg){this -> laserScanCallback(msg,1);});
                scan_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);
            }


        private:
            void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg, int lidar_id) {
                if (lidar_id == 0) {
                    // LD-Lidar: そのまま保存
                    //scanFront_ = msg;
                } 
                else if (lidar_id == 1) {
                    // Front Lidar
                    filterScan(msg, scanFront_, -M_PI / 2.6, M_PI / 2.3);
                } 
                else if (lidar_id == 2) {
                    // Back Lidar
                    //filterScan(msg, scanBack_, -M_PI * 3/ 4, -M_PI / 6);
                }
            }

            void filterScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, sensor_msgs::msg::LaserScan::SharedPtr &return_scan, double min_angle, double max_angle) {
                auto filtered_scan = *msg; 
                double threshold = this->get_parameter("filter_threshold").as_double();
                size_t num_points = msg->ranges.size();
                
                std::vector<Point2D> points(num_points);
                std::vector<bool> is_valid(num_points, false);

                for (size_t i = 0; i < num_points; ++i) {
                    double r = msg->ranges[i];
                    if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
                        double angle = msg->angle_min + i * msg->angle_increment;
                        if (angle < min_angle || angle > max_angle) {
                            filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
                            is_valid[i] = false;
                            continue;
                        }
                        points[i].x = r * std::cos(angle);
                        points[i].y = r * std::sin(angle);
                        is_valid[i] = true;
                    } else {
                        is_valid[i] = false;
                    }
                }

                for (size_t i = 0; i < num_points - 1; ++i) {
                    if (!is_valid[i] || !is_valid[i+1]) continue;

                    Point2D pA = points[i];
                    Point2D pB = points[i+1];
                    Point2D vec_AB = {pB.x - pA.x, pB.y - pA.y};
                    Point2D vec_M = {(pA.x + pB.x) / 2.0, (pA.y + pB.y) / 2.0};

                    double norm_AB = std::hypot(vec_AB.x, vec_AB.y);
                    double norm_M = std::hypot(vec_M.x, vec_M.y);

                    if (norm_AB < 1e-6 || norm_M < 1e-6) continue;

                    double dot_product = (vec_AB.x * vec_M.x + vec_AB.y * vec_M.y) / (norm_AB * norm_M);
                    double abs_cos = std::abs(dot_product);

                    if (abs_cos > threshold) {
                        filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
                        filtered_scan.ranges[i+1] = std::numeric_limits<float>::infinity();
                        if (!filtered_scan.intensities.empty()) {
                            filtered_scan.intensities[i] = 0;
                            filtered_scan.intensities[i+1] = 0;
                        }
                    }
                }
                return_scan = std::make_shared<sensor_msgs::msg::LaserScan>(filtered_scan);
            }

            void publishScanClouds(const sensor_msgs::msg::LaserScan::SharedPtr scan1, const sensor_msgs::msg::LaserScan::SharedPtr scan2) {
                if (scan_cloud_pub_->get_subscription_count() == 0) return;

                std::vector<sensor_msgs::msg::PointCloud2> clouds_to_merge;
                size_t total_points = 0;

                auto process_scan = [&](const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
                    if (!scan) return;
                    try {
                        sensor_msgs::msg::PointCloud2 local_cloud, global_cloud;
                        projector_.projectLaser(*scan, local_cloud);
                        
                        geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
                            "base_footprint", 
                            scan->header.frame_id, 
                            tf2::TimePointZero
                        );
                        tf2::doTransform(local_cloud, global_cloud, tf);
                        
                        clouds_to_merge.push_back(global_cloud);
                        total_points += global_cloud.width; 
                    } catch (const std::exception& e) {
                        //TFエラー
                    }
                };

                process_scan(scan1);
                process_scan(scan2);

                if (total_points == 0) return;

                sensor_msgs::msg::PointCloud2 combined_cloud;
                combined_cloud.header.frame_id = "base_footprint";
                combined_cloud.header.stamp = this->now();

                sensor_msgs::PointCloud2Modifier modifier(combined_cloud);
                modifier.setPointCloud2FieldsByString(1, "xyz");
                modifier.resize(total_points);

                // イテレータでデータをコピー
                sensor_msgs::PointCloud2Iterator<float> iter_x(combined_cloud, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(combined_cloud, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(combined_cloud, "z");

                for (const auto& cloud : clouds_to_merge) {
                    sensor_msgs::PointCloud2ConstIterator<float> src_x(cloud, "x");
                    sensor_msgs::PointCloud2ConstIterator<float> src_y(cloud, "y");
                    // projectLaserの結果には通常zが含まれる
                    
                    for (size_t i = 0; i < cloud.width; ++i) {
                        *iter_x = *src_x;
                        *iter_y = *src_y;
                        *iter_z = 0.0f; 
                        
                        ++iter_x; ++iter_y; ++iter_z;
                        ++src_x; ++src_y;
                    }
                }

                
                scan_cloud_pub_->publish(combined_cloud);
            }
            
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScanFront_;
            sensor_msgs::msg::LaserScan::SharedPtr scanFront_;
            sensor_msgs::msg::LaserScan::SharedPtr scanBack_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_cloud_pub_; 
            laser_geometry::LaserProjection projector_;
            tf2_ros::Buffer tf_buffer_;
            
            struct Point2D {
                double x;
                double y;
            };



    };


}