#include <rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

namespace lidar_filter{
    class Lidar_filter: public rclcpp::Node{
        public:
            Lidar_filter():Node("lidar_filter"){
                auto lidarScanqos = rclcpp::SensorDataQoS();
                subScanFront_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan_front",lidarScanqos,[this](const sensor_msgs::msg::LaserScan::SharedPtr msg){this -> laserScanCallback(msg,1);});
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
            
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScanFront_;
            sensor_msgs::msg::LaserScan::SharedPtr scanFront_;
            sensor_msgs::msg::LaserScan::SharedPtr scanBack_;
            
            struct Point2D {
                double x;
                double y;
            };



    };


}