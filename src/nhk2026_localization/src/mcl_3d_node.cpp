#include <rclcpp/rclcpp.hpp>
#include <H5Cpp.h>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <random>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <fstream> 
#include <geometry_msgs/msg/pose2_d.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp" 
#include "nhk2026_msgs/msg/multi_laser_scan.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace H5;
using namespace std::chrono_literals;

namespace mcl {
    struct Point3D {
        double x, y, z;
    };

    class Particle {
        public:
            Particle() : w_(0.0) {
                pose_.position.x = 0.0;
                pose_.position.y = 0.0;
                pose_.position.z = 0.0;
                pose_.orientation.x = 0.0;
                pose_.orientation.y = 0.0;
                pose_.orientation.z = 0.0;
                pose_.orientation.w = 1.0;
            }

            const double getX() const { return pose_.position.x; }
            const double getY() const { return pose_.position.y; }
            const double getZ() const { return pose_.position.z; }
            
            double getTheta() const {
                tf2::Quaternion q(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
                tf2::Matrix3x3 m(q);
                double r, p, yaw;
                m.getRPY(r, p, yaw);
                return yaw;
            }

            const geometry_msgs::msg::Pose& getPose() const { return pose_; }
            const double getW() const { return w_; }

            void setPose(double x, double y, double z, double yaw) {
                pose_.position.x = x;
                pose_.position.y = y;
                pose_.position.z = z;
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw); 
                pose_.orientation.x = q.x();
                pose_.orientation.y = q.y();
                pose_.orientation.z = q.z();
                pose_.orientation.w = q.w();
            }

            void setW(double w) { w_ = w; }

        private:
            geometry_msgs::msg::Pose pose_;
            double w_;
    };

    class MCL_3D : public rclcpp::Node {
        public:
            explicit MCL_3D(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("mcl_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
                // --- パラメータ宣言 ---
                this->declare_parameter<std::string>("mapFile", "src/nhk2026_localization/map/nhk2026_field_tamokuteki.h5"); 
                this->declare_parameter<double>("mapResolution", 0.01); 
                this->declare_parameter<double>("lfmSigma", 0.03);
                this->declare_parameter<int>("particleNum", 100);
                this->declare_parameter<double>("initial_x", -1.0);
                this->declare_parameter<double>("initial_y", 1.0);
                this->declare_parameter<double>("initial_z", 0.0);
                this->declare_parameter<double>("initial_theta", M_PI/2);
                this->declare_parameter<double>("zHit", 0.9);
                this->declare_parameter<double>("zRand", 0.1);
                this->declare_parameter<double>("odomNoise1", 1.0);
                this->declare_parameter<double>("odomNoise2", 0.1);
                this->declare_parameter<double>("odomNoise3", 0.5);
                this->declare_parameter<double>("odomNoise4", 0.5);
                this->declare_parameter<double>("resampleThreshold", 0.9);
                this->declare_parameter<int>("scanStep", 10); // 2D LiDARの間引き用

                // --- パラメータ取得 ---
                mapFile_ = this->get_parameter("mapFile").as_string();
                mapResolution_ = this->get_parameter("mapResolution").as_double();
                lfmSigma_ = this->get_parameter("lfmSigma").as_double();
                particleNum_ = this->get_parameter("particleNum").as_int();
                scanStep_ = this->get_parameter("scanStep").as_int();
                resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();
                zHit_ = this->get_parameter("zHit").as_double();
                zRand_ = this->get_parameter("zRand").as_double();
                odomNoise1_ = this->get_parameter("odomNoise1").as_double();
                odomNoise2_ = this->get_parameter("odomNoise2").as_double();
                odomNoise3_ = this->get_parameter("odomNoise3").as_double();
                odomNoise4_ = this->get_parameter("odomNoise4").as_double();

                particles_.resize(particleNum_);

                // 初期姿勢設定
                mclPose_.position.x = this->get_parameter("initial_x").as_double();
                mclPose_.position.y = this->get_parameter("initial_y").as_double();
                mclPose_.position.z = this->get_parameter("initial_z").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();
                tf2::Quaternion init_q; init_q.setRPY(0.0, 0.0, initial_theta);
                mclPose_.orientation.x = init_q.x(); mclPose_.orientation.y = init_q.y();
                mclPose_.orientation.z = init_q.z(); mclPose_.orientation.w = init_q.w();

                resetParticlesDistribution(0.3, 0.3, M_PI/18.0, 1.0 / particleNum_);

                // --- Pub/Sub設定 ---
                auto qos_transient = rclcpp::QoS(1).transient_local().reliable();
                sdf_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_cloud", qos_transient);
                
                auto qos_default = rclcpp::QoS(10);
                pubPose_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", qos_default);
                pubPath_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", qos_default);
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos_default);
                vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity_marker", qos_default);
                filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan3d_cloud", qos_default);

                subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_feedback", 10, std::bind(&MCL_3D::cmdVelCallback, this, std::placeholders::_1));
                subCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("livox/lidar", rclcpp::SensorDataQoS(), std::bind(&MCL_3D::pointCloudCallback, this, std::placeholders::_1));
                subInitialPose_ = create_subscription<geometry_msgs::msg::Pose>("initial_pose", 10, std::bind(&MCL_3D::initialPoseCallback, this, std::placeholders::_1));
                subExtQuat_ = create_subscription<std_msgs::msg::Float32MultiArray>("quaternion_feedback", 10, std::bind(&MCL_3D::externalQuatCallback, this, std::placeholders::_1));
                zaxissub_= create_subscription<std_msgs::msg::Int32MultiArray>("mcl_select", 10, std::bind(&MCL_3D::lidarSelectCallback, this, std::placeholders::_1));
                subMultiScan_ = create_subscription<nhk2026_msgs::msg::MultiLaserScan>("multi_scan", rclcpp::SensorDataQoS(), [this](const nhk2026_msgs::msg::MultiLaserScan::SharedPtr msg) { last_multi_scan_ = msg; });

                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
                path_.header.frame_id = "map";

                const char *sim = std::getenv("WITH_SIM");
                is_sim_ = (sim && std::string(sim) == "1");

                readMap();
                timer_ = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&MCL_3D::loop, this));
            }

        private:
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVel_ = msg; }

            void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                last_pc_points_.clear(); // 届くたびにバッファを更新
                sensor_msgs::msg::PointCloud2 cloud_out;
                try {
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform("base_footprint", msg->header.frame_id, tf2::TimePointZero);
                    tf2::doTransform(*msg, cloud_out, transform);
                } catch (const tf2::TransformException & ex) { return; }

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x"), iter_y(cloud_out, "y"), iter_z(cloud_out, "z");
                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) continue;
                    double dist_sq = (*iter_x)*(*iter_x) + (*iter_y)*(*iter_y) + (*iter_z)*(*iter_z);
                    if (dist_sq < 0.45*0.45 || dist_sq > 3.0*3.0 || *iter_z > 1.5) continue;

                    double z_map = *iter_z + mclPose_.position.z;
                    if (std::abs(z_map - 0.00) <= 0.03 || std::abs(z_map - 0.20) <= 0.03 || std::abs(z_map - 0.40) <= 0.03) continue;

                    last_pc_points_.push_back({(double)*iter_x, (double)*iter_y, (double)*iter_z});
                }
            }

            void process2DScans(const nhk2026_msgs::msg::MultiLaserScan::SharedPtr& multi_scan, std::vector<Point3D>& points_out) {
                if (!multi_scan) return;
                for (const auto& scan : multi_scan->scans) {
                    int lidar_id = 1; // 1:front, 2:back, 0:ld
                    if (scan.header.frame_id.find("lidar_back") != std::string::npos) lidar_id = 2;
                    else if (scan.header.frame_id.find("ldlidar") != std::string::npos) lidar_id = 0;

                    double mounting_z = (lidar_id == 0) ? 0.20 : 0.15; // 取り付け高さ[m]

                    for (std::size_t i = 0; i < scan.ranges.size(); i += scanStep_) {
                        double r = scan.ranges[i];
                        if (std::isnan(r) || r < scan.range_min || r > scan.range_max) continue;
                        double theta = scan.angle_min + (double(i) * scan.angle_increment);
                        if (lidar_id == 0 && !is_sim_) theta -= 1.5 * M_PI;

                        double lx, ly;
                        if (lidar_id == 0) { lx = r * cos(theta) + 0.084; ly = r * sin(theta); }
                        else if (lidar_id == 1) { lx = r * cos(-1 * theta + M_PI/2) - 0.157; ly = r * sin(-1 * theta + M_PI/2) + 0.2885; }
                        else { lx = r * cos(theta) + 0.27; ly = r * sin(-1 * theta) - 0.3615; }

                        points_out.push_back({lx, ly, mounting_z});
                    }
                }
            }

            void loop() {
                if (zaxics_ && !zaxics_->data.empty() && zaxics_->data[0] == 0) return;
                if (!cmdVel_) return;

                // --- データ合流 ---
                local_points_.clear();
                if (!last_pc_points_.empty()) local_points_.insert(local_points_.end(), last_pc_points_.begin(), last_pc_points_.end());
                if (last_multi_scan_) process2DScans(last_multi_scan_, local_points_);

                if (local_points_.empty()) return;

                // --- MCL 更新サイクル ---
                double dt = 0.100;
                geometry_msgs::msg::Twist delta;
                delta.linear.x = cmdVel_->linear.x * dt;
                delta.linear.y = cmdVel_->linear.y * dt;
                delta.angular.z = cmdVel_->angular.z * dt;

                updateParticles(delta);
                caculateMeasurementModel();
                estimatePose();
                resampleParticles();

                // 可視化
                printParticlesMakerOnRviz2();
                printTrajectoryOnRviz2();
                publishVelocityMarker();
            }

            // --- MCL コア関数群 ---
            void updateParticles(geometry_msgs::msg::Twist delta) {
                double dd2 = delta.linear.x * delta.linear.x + delta.linear.y * delta.linear.y;
                double dy2 = delta.angular.z * delta.angular.z;
                double sigma_xy = std::sqrt(odomNoise1_ * dd2 + odomNoise2_ * dy2);
                double ext_yaw = 0.0;
                if (has_external_quat_) {
                    tf2::Quaternion q(external_quat_.x, external_quat_.y, external_quat_.z, external_quat_.w);
                    tf2::Matrix3x3 m(q); double r, p; m.getRPY(r, p, ext_yaw);
                }

                for (auto& p : particles_) {
                    double dx = delta.linear.x + randNormal(sigma_xy);
                    double dy = delta.linear.y + randNormal(sigma_xy);
                    double th_old = p.getTheta();
                    double x_new = p.getX() + std::cos(th_old)*dx - std::sin(th_old)*dy;
                    double y_new = p.getY() + std::sin(th_old)*dx + std::cos(th_old)*dy;
                    double th_new = has_external_quat_ ? ext_yaw + randNormal(0.005) : th_old + delta.angular.z + randNormal(std::sqrt(odomNoise3_*dd2 + odomNoise4_*dy2));
                    p.setPose(x_new, y_new, p.getZ(), th_new);
                }
            }

            void caculateMeasurementModel() {
                std::vector<double> log_weights(particleNum_);
                double max_log_weight = -std::numeric_limits<double>::infinity();
                for (int i = 0; i < particleNum_; ++i) {
                    log_weights[i] = caculateLogLikelihood(particles_[i].getPose(), local_points_);
                    if (log_weights[i] > max_log_weight) max_log_weight = log_weights[i];
                }
                double w_sum = 0.0;
                for (int i = 0; i < particleNum_; ++i) {
                    double w = std::exp(log_weights[i] - max_log_weight);
                    particles_[i].setW(w); w_sum += w;
                }
                double w_sq_sum = 0.0;
                for (auto& p : particles_) {
                    p.setW(p.getW() / w_sum);
                    w_sq_sum += p.getW() * p.getW();
                }
                effectiveSampleSize_ = 1.0 / w_sq_sum;
            }

            double caculateLogLikelihood(const geometry_msgs::msg::Pose& pose, const std::vector<Point3D>& points) {
                double total_log_p = 0.0;
                double var = lfmSigma_ * lfmSigma_;
                double normConst = 1.0 / (std::sqrt(2.0 * M_PI * var));
                double pRand_const = (1.0 / 30.0) * mapResolution_;
                tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                tf2::Matrix3x3 m(q); double r, p, yaw; m.getRPY(r, p, yaw);
                double ct = std::cos(yaw), st = std::sin(yaw);

                for (const auto& pt : points) {
                    double mx = pt.x * ct - pt.y * st + pose.position.x;
                    double my = pt.x * st + pt.y * ct + pose.position.y;
                    double mz = pt.z + pose.position.z;
                    int u, v, w; xyz2uvw(mx, my, mz, &u, &v, &w);
                    double prob = zRand_ * pRand_const;
                    if (u >= 0 && u < (int)dim_x_ && v >= 0 && v < (int)dim_y_ && w >= 0 && w < (int)dim_z_) {
                        double d = std::abs(distField3D_[getIdx3D(u, v, w)]);
                        if (d < 0.5) {
                            double pHit = normConst * std::exp(-(d * d) / (2.0 * var)) * mapResolution_;
                            prob = std::min(1.0, zHit_ * pHit + zRand_ * pRand_const);
                        }
                    }
                    total_log_p += std::log(std::max(prob, 1e-10));
                }
                return total_log_p;
            }

            // --- その他ユーティリティ (既存コードより) ---
            void estimatePose() {
                double tx = 0, ty = 0, tz = 0, tth = 0, old_th = mclPose_.position.x; // ダミー初期化
                tf2::Quaternion q_old(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                tf2::Matrix3x3(q_old).getRPY(tz, tz, old_th);
                tz = 0; // tz再利用
                for(const auto& p : particles_) {
                    tx += p.getX() * p.getW(); ty += p.getY() * p.getW(); tz += p.getZ() * p.getW();
                    double dth = old_th - p.getTheta();
                    while(dth < -M_PI) dth += 2*M_PI; while(dth > M_PI) dth -= 2*M_PI;
                    tth += dth * p.getW();
                }
                mclPose_.position.x = tx; mclPose_.position.y = ty; mclPose_.position.z = tz;
                tf2::Quaternion q_new; q_new.setRPY(0, 0, old_th - tth); mclPose_.orientation = tf2::toMsg(q_new);
                pubPose_->publish(mclPose_);
            }

            void resampleParticles() {
                if (effectiveSampleSize_ > (double)particleNum_ * resampleThreshold_) return;
                std::vector<double> w_buf(particleNum_); w_buf[0] = particles_[0].getW();
                for(int i=1; i<particleNum_; ++i) w_buf[i] = w_buf[i-1] + particles_[i].getW();
                std::vector<Particle> old = particles_;
                for(int i=0; i<particleNum_; ++i) {
                    double dart = (double)rand() / (RAND_MAX + 1.0);
                    for(int j=0; j<particleNum_; ++j) {
                        if(dart < w_buf[j]) {
                            particles_[i].setPose(old[j].getX(), old[j].getY(), old[j].getZ(), old[j].getTheta());
                            particles_[i].setW(1.0/particleNum_); break;
                        }
                    }
                }
            }

            void readMap() { /* 既存のSDF読み込み/計算ロジックをここに挿入 */ }
            void compute3DEDT(std::vector<float>& g) { /* 既存ロジック */ }
            void dt1d(const std::vector<float>& f, std::vector<float>& d, int n) { /* 既存ロジック */ }
            void resetParticlesDistribution(double nx, double ny, double nt, double w) {
                std::double_t current_yaw = 0, r, p;
                tf2::Quaternion q(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                tf2::Matrix3x3(q).getRPY(r, p, current_yaw);
                for(auto& par : particles_) par.setPose(mclPose_.position.x + randNormal(nx), mclPose_.position.y + randNormal(ny), mclPose_.position.z, current_yaw + randNormal(nt)), par.setW(w);
            }
            double randNormal(double s) { return (s <= 0) ? 0 : std::normal_distribution<double>(0, s)(gen_); }
            inline size_t getIdx3D(int x, int y, int z) const { return (size_t)x*(dim_y_*dim_z_) + (size_t)y*dim_z_ + z; }
            inline void xyz2uvw(double x, double y, double z, int *u, int *v, int *w) const { *u = (int)((x-mapOrigin_[0])/mapResolution_); *v = (int)((y-mapOrigin_[1])/mapResolution_); *w = (int)((z-mapOrigin_[2])/mapResolution_); }
            void initialPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) { mclPose_ = *msg; resetParticlesDistribution(0.1, 0.1, M_PI/36.0, 1.0/particleNum_); }
            void externalQuatCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) { if(msg->data.size()>=4) { external_quat_.w=msg->data[0]; external_quat_.x=msg->data[1]; external_quat_.y=msg->data[2]; external_quat_.z=msg->data[3]; has_external_quat_=true; } }
            void lidarSelectCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) { zaxics_ = msg; }
            void publishSDFCloud() { /* 既存ロジック */ }
            void printParticlesMakerOnRviz2() { /* 既存ロジック */ }
            void printTrajectoryOnRviz2() { /* 既存ロジック */ }
            void publishVelocityMarker() { /* 既存ロジック */ }

            // メンバ変数
            int particleNum_, scanStep_;
            std::vector<Particle> particles_;
            double effectiveSampleSize_, resampleThreshold_, mapResolution_, zHit_, zRand_, odomNoise1_, odomNoise2_, odomNoise3_, odomNoise4_, lfmSigma_;
            std::string mapFile_;
            hsize_t dim_x_, dim_y_, dim_z_;
            std::vector<float> distField3D_;
            std::vector<double> mapOrigin_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_cloud_pub_, particleMarker_, filtered_cloud_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubPose_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud_;
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subInitialPose_;
            rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subExtQuat_;
            rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr zaxissub_;
            rclcpp::Subscription<nhk2026_msgs::msg::MultiLaserScan>::SharedPtr subMultiScan_;
            geometry_msgs::msg::Pose mclPose_;
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            nav_msgs::msg::Path path_;
            std::vector<Point3D> local_points_, last_pc_points_;
            nhk2026_msgs::msg::MultiLaserScan::SharedPtr last_multi_scan_;
            std::mt19937 gen_;
            std_msgs::msg::Int32MultiArray::SharedPtr zaxics_;
            geometry_msgs::msg::Quaternion external_quat_;
            bool has_external_quat_ = false, is_sim_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL_3D>());
    rclcpp::shutdown();
    return 0;
}