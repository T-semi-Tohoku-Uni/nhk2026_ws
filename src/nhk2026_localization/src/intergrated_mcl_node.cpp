#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <H5Cpp.h>
#include <opencv2/opencv.hpp>
#include "nhk2026_msgs/msg/multi_laser_scan.hpp"

#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <random>
#include <fstream>
#include <chrono>

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
                tf2::Quaternion q(
                    pose_.orientation.x, pose_.orientation.y,
                    pose_.orientation.z, pose_.orientation.w
                );
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
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

    // -----------------------------------------------------------------
    // 統合 MCL ノード クラス
    // -----------------------------------------------------------------
    class IntegratedMCLNode: public rclcpp::Node {
        public:
            explicit IntegratedMCLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("integrated_mcl_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
              current_zaxis_level_(0) // 初期状態は2Dモード
            {
                // パラメータ宣言
                this->declare_parameter<std::string>("mapFile2D", "src/nhk2026_localization/map/nhk2026_field.h5");
                this->declare_parameter<std::string>("mapFile3D", "src/nhk2026_localization/map/nhk2026_field_tamokuteki.h5");
                this->declare_parameter<int>("mapZIndex2D", 4);
                this->declare_parameter<double>("mapResolution2D", 0.01);
                this->declare_parameter<double>("mapResolution3D", 0.01);
                
                this->declare_parameter<int>("particleNum", 100);
                this->declare_parameter<float>("initial_x", -1.0);
                this->declare_parameter<float>("initial_y", 1.0);
                this->declare_parameter<float>("initial_z", 0.0);
                this->declare_parameter<float>("initial_theta", M_PI/2);

                this->declare_parameter<double>("resampleThreshold", 0.5);
                this->declare_parameter<double>("odomNoise1_2D", 1.0);
                this->declare_parameter<double>("odomNoise2_2D", 1.0);
                this->declare_parameter<double>("odomNoise3_2D", 1.0);
                this->declare_parameter<double>("odomNoise4_2D", 1.0);

                this->declare_parameter<double>("odomNoise1_3D", 1.0);
                this->declare_parameter<double>("odomNoise2_3D", 1.0);
                this->declare_parameter<double>("odomNoise3_3D", 1.0);
                this->declare_parameter<double>("odomNoise4_3D", 1.0);

                this->declare_parameter<double>("lfmSigma", 0.03);
                this->declare_parameter<double>("zHit", 0.9);
                this->declare_parameter<double>("zRand", 0.1);
                this->declare_parameter<int>("scanStep", 50);

                // パラメータの取得
                mapFile2D_ = this->get_parameter("mapFile2D").as_string();
                mapFile3D_ = this->get_parameter("mapFile3D").as_string();
                mapZIndex2D_ = this->get_parameter("mapZIndex2D").as_int();
                mapResolution2D_ = this->get_parameter("mapResolution2D").as_double();
                mapResolution3D_ = this->get_parameter("mapResolution3D").as_double();

                particleNum_ = this->get_parameter("particleNum").as_int();
                double init_x = this->get_parameter("initial_x").as_double();
                double init_y = this->get_parameter("initial_y").as_double();
                double init_z = this->get_parameter("initial_z").as_double();
                double init_theta = this->get_parameter("initial_theta").as_double();

                resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();
                odomNoise1_2D_ = this->get_parameter("odomNoise1_2D").as_double();
                odomNoise2_2D_ = this->get_parameter("odomNoise2_2D").as_double();
                odomNoise3_2D_ = this->get_parameter("odomNoise3_2D").as_double();
                odomNoise4_2D_ = this->get_parameter("odomNoise4_2D").as_double();

                odomNoise1_3D_ = this->get_parameter("odomNoise1_3D").as_double();
                odomNoise2_3D_ = this->get_parameter("odomNoise2_3D").as_double();
                odomNoise3_3D_ = this->get_parameter("odomNoise3_3D").as_double();
                odomNoise4_3D_ = this->get_parameter("odomNoise4_3D").as_double();

                lfmSigma_ = this->get_parameter("lfmSigma").as_double();
                zHit_ = this->get_parameter("zHit").as_double();
                zRand_ = this->get_parameter("zRand").as_double();
                scanStep_ = this->get_parameter("scanStep").as_int();

                // パーティクルの初期化
                particles_.resize(particleNum_);
                mclPose_.position.x = init_x;
                mclPose_.position.y = init_y;
                mclPose_.position.z = init_z;
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, init_theta);
                mclPose_.orientation.x = q.x();
                mclPose_.orientation.y = q.y();
                mclPose_.orientation.z = q.z();
                mclPose_.orientation.w = q.w();

                resetParticlesDistribution(0.3, 0.3, M_PI/18.0, 1.0 / particleNum_);

                // パブリッシャの初期化
                auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
                auto qos_default = rclcpp::QoS(10);
                
                pubPose_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", qos_default);
                pubPath_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", qos_default);
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos_default);
                vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity_marker", qos_default);
                
                pubMapCloud2D_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_map_cloud", qos_transient);
                sdf_cloud_pub3D_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_cloud", qos_transient);
                filtered_cloud_pub3D_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan3d_cloud", qos_default);

                // サブスクライバの初期化
                auto sensor_qos = rclcpp::SensorDataQoS();

                subCmdVel_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "cmd_vel_feedback", qos_default, std::bind(&IntegratedMCLNode::cmdVelCallback, this, std::placeholders::_1));
                
                subZaxis_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
                    "", qos_default, std::bind(&IntegratedMCLNode::zaxisCallback, this, std::placeholders::_1));
                
                subInitialPose_ = this->create_subscription<geometry_msgs::msg::Pose>(
                    "initial_pose", qos_default, std::bind(&IntegratedMCLNode::initialPoseCallback, this, std::placeholders::_1));

                subExtQuat_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                    "quaternion_feedback", qos_default, std::bind(&IntegratedMCLNode::externalQuatCallback, this, std::placeholders::_1));

                // センサデータ (2D & 3D)
                subMultiScan_ = this->create_subscription<nhk2026_msgs::msg::MultiLaserScan>(
                    "multi_scan", sensor_qos, std::bind(&IntegratedMCLNode::multiScanCallback, this, std::placeholders::_1));
                
                subCloud3D_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "livox/lidar", sensor_qos, std::bind(&IntegratedMCLNode::pointCloudCallback, this, std::placeholders::_1));

                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
                path_.header.frame_id = "map";

                // シミュレーション判定
                const char *sim = std::getenv("WITH_SIM");
                is_sim_ = (sim && std::string(sim) == "1");
                RCLCPP_INFO(this->get_logger(), "Integrated MCL Node Started. is_sim_: %s", is_sim_ ? "true" : "false");

                // 2Dと3Dの両方のマップを読み込む
                readMap2D();
                readMap3D();

                // メインループタイマー (25ms = 40Hz)
                timer_ = rclcpp::create_timer(this, this->get_clock(), 25ms, std::bind(&IntegratedMCLNode::loop, this));
            }

        private:

          
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVel_ = msg;
            }

            void multiScanCallback(const nhk2026_msgs::msg::MultiLaserScan::SharedPtr msg) {
                scan2D_ = msg;
            }

            void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                // 3Dモードでない場合は処理をスキップして負荷を減らす
                if (current_zaxis_level_ == 0) return;

                local_points3D_.clear();
                sensor_msgs::msg::PointCloud2 cloud_out;

                try {
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                        "base_footprint", msg->header.frame_id, tf2::TimePointZero);
                    tf2::doTransform(*msg, cloud_out, transform);
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "TF Error: %s", ex.what());
                    return;
                }

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_out, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_out, "z");

                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    float x = *iter_x, y = *iter_y, z = *iter_z;
                    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
                    
                    double dist_sq = x*x + y*y + z*z;
                    if (dist_sq < 0.45*0.45 || dist_sq > 3.0*3.0) continue; 
                    if (z > 1.5) continue;

                    double z_map = z + mclPose_.position.z;
                    // 床面フィルタリング
                    if (std::abs(z_map - 0.00) <= 0.03 || std::abs(z_map - 0.20) <= 0.03 || std::abs(z_map - 0.40) <= 0.03) {
                        continue;
                    }

                    local_points3D_.push_back({x, y, z});
                }

                // デバッグ用パブリッシュ
                if (!local_points3D_.empty()) {
                    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
                    filtered_cloud_msg.header.stamp = msg->header.stamp; 
                    filtered_cloud_msg.header.frame_id = "base_footprint"; 
                    filtered_cloud_msg.height = 1;
                    filtered_cloud_msg.width = local_points3D_.size();
                    filtered_cloud_msg.is_dense = true;
                    filtered_cloud_msg.is_bigendian = false;
                    sensor_msgs::PointCloud2Modifier modifier(filtered_cloud_msg);
                    modifier.setPointCloud2FieldsByString(1, "xyz");
                    modifier.resize(local_points3D_.size());
                    sensor_msgs::PointCloud2Iterator<float> out_x(filtered_cloud_msg, "x");
                    sensor_msgs::PointCloud2Iterator<float> out_y(filtered_cloud_msg, "y");
                    sensor_msgs::PointCloud2Iterator<float> out_z(filtered_cloud_msg, "z");
                    for (const auto& pt : local_points3D_) {
                        *out_x = pt.x; *out_y = pt.y; *out_z = pt.z;
                        ++out_x; ++out_y; ++out_z;
                    }
                    filtered_cloud_pub3D_->publish(filtered_cloud_msg);
                }
            }

         
            void zaxisCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
                if (msg->data.empty()) return;
                int new_level = msg->data[0];
                
                if (new_level != current_zaxis_level_) {
                    RCLCPP_INFO(this->get_logger(), "Mode Switch: Level %d -> %d", current_zaxis_level_, new_level);
                    
                    if (new_level == 0) mclPose_.position.z = 0.0;
                    else if (new_level == 1) mclPose_.position.z = 0.20;
                    else if (new_level == 2) mclPose_.position.z = 0.40;

                    // 高さが変わったのでパーティクルをリセット (XYとYawは現在値をベースに少しノイズを乗せる)
                    double noise_x = 0.05, noise_y = 0.05, noise_yaw = 2.0 * M_PI / 180.0;
                    resetParticlesDistribution(noise_x, noise_y, noise_yaw, 1.0 / particleNum_);
                    
                    current_zaxis_level_ = new_level;
                }
            }

            void initialPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
                mclPose_.position = msg->position;
                mclPose_.orientation = msg->orientation;

                // 外部から与えられたZ軸に従ってレベルを強制上書き
                if (std::abs(mclPose_.position.z - 0.0) < 0.1) current_zaxis_level_ = 0;
                else if (std::abs(mclPose_.position.z - 0.20) < 0.1) current_zaxis_level_ = 1;
                else if (std::abs(mclPose_.position.z - 0.40) < 0.1) current_zaxis_level_ = 2;

                tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                resetParticlesDistribution(0.1, 0.1, 5.0 * M_PI / 180.0, 1.0 / particleNum_);
                RCLCPP_INFO(this->get_logger(), "Pose reset: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", 
                            mclPose_.position.x, mclPose_.position.y, mclPose_.position.z, yaw);
            }

            void externalQuatCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 4) {
                    external_quat_.w = msg->data[0];
                    external_quat_.x = msg->data[1];
                    external_quat_.y = msg->data[2];
                    external_quat_.z = msg->data[3];
                    has_external_quat_ = true;
                }
            }

          
            void loop() {
                if (!cmdVel_) return;

                double dt = 0.025; // タイマー周期に合わせる
                geometry_msgs::msg::Twist delta;
                delta.linear.x = cmdVel_->linear.x * dt;
                delta.linear.y = cmdVel_->linear.y * dt;
                delta.angular.z = cmdVel_->angular.z * dt;

                updateParticles(delta);
                
                // モードに応じて観測更新を切り替え
                if (current_zaxis_level_ == 0) {
                    // 2Dモード
                    if (scan2D_ && !scan2D_->scans.empty()) {
                        caculateMeasurementModel2D(scan2D_);
                    }
                } else {
                    // 3Dモード (Level 1, 2)
                    if (!local_points3D_.empty()) {
                        caculateMeasurementModel3D(local_points3D_);
                    }
                }

                estimatePose();
                resampleParticles();
                
                printTrajectoryOnRviz2();
                printParticlesMakerOnRviz2();
                publishVelocityMarker();
            }

          
            double randNormal(double sigma) {
                if (sigma <= 0.0) return 0.0;
                std::normal_distribution<double> dist(0.0, sigma);
                return dist(gen_);
            }

            void resetParticlesDistribution(double noise_x, double noise_y, double noise_theta, double initial_w) {
                for (std::size_t i = 0; i < particles_.size(); i++ ) {
                    double x = mclPose_.position.x + randNormal(noise_x);
                    double y = mclPose_.position.y + randNormal(noise_y);
                    
                    tf2::Quaternion q(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                    tf2::Matrix3x3 m(q);
                    double r, p, yaw;
                    m.getRPY(r, p, yaw);

                    double theta = yaw + randNormal(noise_theta);
                    particles_[i].setPose(x, y, mclPose_.position.z, theta);
                    particles_[i].setW(initial_w);
                }
            }

            void updateParticles(geometry_msgs::msg::Twist delta) {
                std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y * delta.linear.y;
                std::double_t dy2 = delta.angular.z * delta.angular.z;
                
                double n1 = (current_zaxis_level_ == 0) ? odomNoise1_2D_ : odomNoise1_3D_;
                double n2 = (current_zaxis_level_ == 0) ? odomNoise2_2D_ : odomNoise2_3D_;
                double n3 = (current_zaxis_level_ == 0) ? odomNoise3_2D_ : odomNoise3_3D_;
                double n4 = (current_zaxis_level_ == 0) ? odomNoise4_2D_ : odomNoise4_3D_;

                std::double_t sigma_xy = std::sqrt(n1 * dd2 + n2 * dy2);
                
                double ext_yaw = 0.0;
                if (has_external_quat_ && current_zaxis_level_ != 0) { // 3Dモード時のみ外部姿勢を考慮
                    tf2::Quaternion q(external_quat_.x, external_quat_.y, external_quat_.z, external_quat_.w);
                    tf2::Matrix3x3 m(q);
                    double r, p;
                    m.getRPY(r, p, ext_yaw);
                }

                for (size_t i = 0; i < particles_.size(); i++) {
                    std::double_t dx = delta.linear.x + randNormal(sigma_xy);
                    std::double_t dy = delta.linear.y + randNormal(sigma_xy);
                    std::double_t theta_old = particles_[i].getTheta();
                    
                    std::double_t x_ = particles_[i].getX() + std::cos(theta_old) * dx - std::sin(theta_old) * dy;
                    std::double_t y_ = particles_[i].getY() + std::sin(theta_old) * dx + std::cos(theta_old) * dy;
                    std::double_t z_ = particles_[i].getZ(); // Zは変化しない

                    std::double_t theta_new;
                    if (has_external_quat_ && current_zaxis_level_ != 0) {
                        theta_new = ext_yaw + randNormal(0.005);
                    } else {
                        std::double_t sigma_theta = std::sqrt(n3 * dd2 + n4 * dy2);
                        theta_new = theta_old + delta.angular.z + randNormal(sigma_theta);
                    }

                    particles_[i].setPose(x_, y_, z_, theta_new);
                }
            }

            void resampleParticles() {
                double threshold = ((double)particles_.size()) * resampleThreshold_;
                if (effectiveSampleSize_ > threshold) return;

                std::vector<double> wBuffer((int)particles_.size());
                wBuffer[0] = particles_[0].getW();
                for (size_t i=1; i<particles_.size(); i++ ) {
                    wBuffer[i] = particles_[i].getW() + wBuffer[i-1];
                }

                std::vector<Particle> tmpParticles = particles_;
                double wo = 1.0 / (double)particles_.size();
                for (size_t i = 0; i < particles_.size(); i++ ) {
                    double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                    for (size_t j=0; j<particles_.size(); j++ ) {
                        if (darts < wBuffer[j]) {
                            geometry_msgs::msg::Pose tmpPos = tmpParticles[j].getPose();
                            particles_[i].setPose(tmpPos.position.x, tmpPos.position.y, tmpPos.position.z, tmpParticles[j].getTheta());
                            particles_[i].setW(wo);
                            break;
                        }
                    }
                }
            }

            void estimatePose() {
                tf2::Quaternion q_old(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                tf2::Matrix3x3 m(q_old);
                double roll, pitch, tmpTheta;
                m.getRPY(roll, pitch, tmpTheta);

                std::double_t x = 0.0, y = 0.0, z = 0.0, theta = 0.0;
                for (size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t w = particles_[i].getW();
                    x += particles_[i].getX() * w;
                    y += particles_[i].getY() * w;
                    z += particles_[i].getZ() * w; 
                    
                    std::double_t dTheta = tmpTheta - particles_[i].getTheta();
                    while (dTheta < -M_PI) dTheta += 2.0*M_PI;
                    while (dTheta > M_PI) dTheta -= 2.0*M_PI;
                    theta += dTheta * w;
                }
                theta = tmpTheta - theta;

                mclPose_.position.x = x;
                mclPose_.position.y = y;
                mclPose_.position.z = z;
                
                tf2::Quaternion q_new;
                q_new.setRPY(0.0, 0.0, theta);
                mclPose_.orientation.x = q_new.x();
                mclPose_.orientation.y = q_new.y();
                mclPose_.orientation.z = q_new.z();
                mclPose_.orientation.w = q_new.w();

                pubPose_->publish(mclPose_);

                if (!is_sim_) {
                    geometry_msgs::msg::TransformStamped tf_msg;
                    tf_msg.header.stamp = this->get_clock()->now();
                    tf_msg.header.frame_id = "odom";
                    tf_msg.child_frame_id = "base_footprint";

                    tf_msg.transform.translation.x = x;
                    tf_msg.transform.translation.y = y;
                    tf_msg.transform.translation.z = z; 
                    tf_msg.transform.rotation = mclPose_.orientation;

                    tf_broadcaster_->sendTransform(tf_msg);
                }
            }

          
            void caculateMeasurementModel2D(const nhk2026_msgs::msg::MultiLaserScan::SharedPtr& multi_scan) {
                if(distField2D_.empty()) return;

                std::vector<double> log_weights(particleNum_);
                double max_log_weight = -std::numeric_limits<double>::infinity();

                double var = lfmSigma_ * lfmSigma_;
                double normConst = 1.0 / (sqrt(2.0 * M_PI * var));

                for (std::size_t p_idx = 0; p_idx < particles_.size(); p_idx++) {
                    double total_log_p = 0.0;
                    double pose_x = particles_[p_idx].getX();
                    double pose_y = particles_[p_idx].getY();
                    double pose_theta = particles_[p_idx].getTheta();

                    for (const auto& scan : multi_scan->scans) {
                        double pRand = 1.0 / scan.range_max * mapResolution2D_;
                        int lidar_pose = 1; // 1: Front, 2: Back, 0: LD
                        if (scan.header.frame_id.find("lidar_back") != std::string::npos) lidar_pose = 2;
                        else if (scan.header.frame_id.find("ldlidar") != std::string::npos) lidar_pose = 0;

                        for (std::size_t i = 0; i < scan.ranges.size(); i += scanStep_) {
                            double r = scan.ranges[i];
                            if (std::isnan(r) || r < scan.range_min || scan.range_max < r) {
                                total_log_p += std::log(zRand_ * pRand);
                                continue;
                            }

                            double theta_lidar = scan.angle_min + (double(i) * scan.angle_increment);
                            if (lidar_pose == 0 && !is_sim_) theta_lidar -= 1.5 * M_PI;

                            // ロボットから見たLiDARの相対位置
                            double x_lidar, y_lidar;
                            if (lidar_pose == 0) {
                                x_lidar = r*cos(theta_lidar) + 0.084; y_lidar = r*sin(theta_lidar);
                            } else if(lidar_pose == 1){
                                x_lidar = r * cos(-1 * theta_lidar + M_PI/2) - 0.157; y_lidar = r * sin(-1 * theta_lidar + M_PI/2) + 0.2885;
                            } else {
                                x_lidar = r * cos(theta_lidar) + 0.27; y_lidar = r * sin(-1 * theta_lidar) - 0.3615;
                            }
                            
                            // マップ座標系への変換
                            double x_odom = x_lidar*cos(pose_theta) - y_lidar*sin(pose_theta) + pose_x;
                            double y_odom = x_lidar*sin(pose_theta) + y_lidar*cos(pose_theta) + pose_y;

                            int u = (int)((x_odom - mapOrigin2D_[0]) / mapResolution2D_);
                            int v = mapHeight2D_ - 1 - (int)((y_odom - mapOrigin2D_[1]) / mapResolution2D_);

                            double p = zRand_ * pRand;
                            if (u >= 0 && u < mapWidth2D_ && v >= 0 && v < mapHeight2D_) {
                                double sdf_val = distField2D_.at<double>(v, u);
                                if (sdf_val > 0.5) { // 動的障害物除外
                                    total_log_p += std::log(p);
                                    continue;
                                }
                                double d = (sdf_val >= 0) ? sdf_val : (std::abs(sdf_val) + 1.0);
                                double pHit = normConst * std::exp(-(d * d) / (2.0 * var)) * mapResolution2D_;
                                p = std::min(1.0, zHit_ * pHit + zRand_ * pRand);
                            }
                            total_log_p += std::log(std::max(p, 1e-10));
                        }
                    }
                    log_weights[p_idx] = total_log_p;
                    if (total_log_p > max_log_weight) max_log_weight = total_log_p;
                }

                // 正規化
                double w_sum = 0.0;
                std::vector<double> linear_weights(particleNum_);
                for (std::size_t i = 0; i < particles_.size(); i++) {
                    linear_weights[i] = std::exp(log_weights[i] - max_log_weight);
                    w_sum += linear_weights[i];
                }
                double w_sq_sum = 0.0;
                for (std::size_t i = 0; i < particles_.size(); i++) {
                    double normalized_w = linear_weights[i] / w_sum;
                    particles_[i].setW(normalized_w);
                    w_sq_sum += normalized_w * normalized_w;
                }
                effectiveSampleSize_ = 1.0 / w_sq_sum;
            }

          
            void caculateMeasurementModel3D(const std::vector<Point3D>& local_points) {
                if (distField3D_.empty() || local_points.empty()) return;

                std::vector<double> log_weights(particleNum_);
                double max_log_weight = -std::numeric_limits<double>::infinity();

                double var = lfmSigma_ * lfmSigma_;
                double normConst = 1.0 / (std::sqrt(2.0 * M_PI * var));
                const double scan_range_max = 30.0; 
                const double pRand_const = (1.0 / scan_range_max) * mapResolution3D_;

                for (std::size_t p_idx = 0; p_idx < particles_.size(); p_idx++) {
                    double total_log_p = 0.0;
                    double pose_x = particles_[p_idx].getX();
                    double pose_y = particles_[p_idx].getY();
                    double pose_z = particles_[p_idx].getZ();
                    double pose_theta = particles_[p_idx].getTheta();

                    double cos_theta = std::cos(pose_theta);
                    double sin_theta = std::sin(pose_theta);

                    for (const auto& pt : local_points) {
                        double map_x = pt.x * cos_theta - pt.y * sin_theta + pose_x;
                        double map_y = pt.x * sin_theta + pt.y * cos_theta + pose_y;
                        double map_z = pt.z + pose_z; 

                        int u = static_cast<int>((map_x - mapOrigin3D_[0]) / mapResolution3D_);
                        int v = static_cast<int>((map_y - mapOrigin3D_[1]) / mapResolution3D_);
                        int w = static_cast<int>((map_z - mapOrigin3D_[2]) / mapResolution3D_);

                        double prob = zRand_ * pRand_const; 

                        if (u >= 0 && u < static_cast<int>(dim_x3D_) && 
                            v >= 0 && v < static_cast<int>(dim_y3D_) && 
                            w >= 0 && w < static_cast<int>(dim_z3D_)) {
                            
                            size_t idx = static_cast<size_t>(u) * (dim_y3D_ * dim_z3D_) + static_cast<size_t>(v) * dim_z3D_ + w;
                            double sdf_val = static_cast<double>(distField3D_[idx]);
                            
                            if (sdf_val > 0.5) {
                                total_log_p += std::log(prob);
                                continue;
                            }

                            double d = (sdf_val >= 0.0) ? sdf_val : (std::abs(sdf_val) + 1.0);
                            double pHit = normConst * std::exp(-(d * d) / (2.0 * var)) * mapResolution3D_;
                            prob = std::min(1.0, zHit_ * pHit + zRand_ * pRand_const);
                        }
                        total_log_p += std::log(std::max(prob, 1e-10));
                    }
                    log_weights[p_idx] = total_log_p;
                    if (total_log_p > max_log_weight) max_log_weight = total_log_p;
                }

                // 正規化
                double w_sum = 0.0;
                std::vector<double> linear_weights(particleNum_);
                for (std::size_t i = 0; i < particles_.size(); i++) {
                    linear_weights[i] = std::exp(log_weights[i] - max_log_weight);
                    w_sum += linear_weights[i];
                }
                double w_sq_sum = 0.0;
                for (std::size_t i = 0; i < particles_.size(); i++) {
                    double normalized_w = linear_weights[i] / w_sum;
                    particles_[i].setW(normalized_w);
                    w_sq_sum += normalized_w * normalized_w;
                }
                effectiveSampleSize_ = 1.0 / w_sq_sum;
            }

          
            void readMap2D() {
                try {
                    RCLCPP_INFO(this->get_logger(), "Loading 2D map from: %s", mapFile2D_.c_str());
                    H5File file(mapFile2D_, H5F_ACC_RDONLY);
                    DataSet dataset = file.openDataSet("map_data");
                    DataSpace dataspace = dataset.getSpace();
                    
                    std::vector<hsize_t> dims(dataspace.getSimpleExtentNdims());
                    dataspace.getSimpleExtentDims(dims.data(), NULL);

                    mapWidth2D_ = dims[0]; mapHeight2D_ = dims[1];
                    hsize_t dim_z = dims[2];

                    if (dataset.attrExists("origin")) {
                        Attribute attr = dataset.openAttribute("origin");
                        float origin_buf[3];
                        attr.read(PredType::NATIVE_FLOAT, origin_buf);
                        mapOrigin2D_ = {(double)origin_buf[0], (double)origin_buf[1], (double)origin_buf[2]};
                    } else {
                        mapOrigin2D_ = {0.0, 0.0, 0.0};
                    }

                    std::vector<uint8_t> map_data(mapWidth2D_ * mapHeight2D_ * dim_z);
                    dataset.read(map_data.data(), PredType::NATIVE_UINT8);

                    cv::Mat binary_img(mapHeight2D_, mapWidth2D_, CV_8UC1);
                    for (int y = 0; y < mapHeight2D_; ++y) {
                        for (int x = 0; x < mapWidth2D_; ++x) {
                            unsigned long idx = x * (mapHeight2D_ * dim_z) + y * dim_z + mapZIndex2D_;
                            int v_img = mapHeight2D_ - 1 - y; 
                            binary_img.at<uint8_t>(v_img, x) = (map_data[idx] == 1) ? 0 : 255;
                        }
                    }

                    cv::Mat distFieldOut, distFieldIn, inverted_binary_img;
                    cv::distanceTransform(binary_img, distFieldOut, cv::DIST_L2, 5);
                    cv::bitwise_not(binary_img, inverted_binary_img);
                    cv::distanceTransform(inverted_binary_img, distFieldIn, cv::DIST_L2, 5);

                    distField2D_ = cv::Mat(mapHeight2D_, mapWidth2D_, CV_64FC1);
                    for (int y = 0; y < mapHeight2D_; ++y) {
                        for (int x = 0; x < mapWidth2D_; ++x) {
                            distField2D_.at<double>(y, x) = (distFieldOut.at<float>(y, x) - distFieldIn.at<float>(y, x)) * mapResolution2D_;
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "2D Distance Field created.");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error in readMap2D: %s", e.what());
                }
            }

            void dt1d(const std::vector<float>& f, std::vector<float>& d, int n) {
                std::vector<int> v(n);
                std::vector<float> z(n + 1);
                int k = 0; v[0] = 0; z[0] = -1e9; z[1] = 1e9;
                for (int q = 1; q < n; q++) {
                    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
                    while (s <= z[k]) {
                        k--; if (k < 0) { k = 0; break; }
                        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
                    }
                    k++; v[k] = q; z[k] = s; z[k + 1] = 1e9;
                }
                k = 0;
                for (int q = 0; q < n; q++) {
                    while (z[k + 1] < q) k++;
                    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
                }
            }

            void compute3DEDT(std::vector<float>& grid) {
                auto getIdx = [this](int x, int y, int z) { return x * (dim_y3D_ * dim_z3D_) + y * dim_z3D_ + z; };
                for (size_t z = 0; z < dim_z3D_; z++) {
                    for (size_t y = 0; y < dim_y3D_; y++) {
                        std::vector<float> f(dim_x3D_), d(dim_x3D_);
                        for (size_t x = 0; x < dim_x3D_; x++) f[x] = grid[getIdx(x, y, z)];
                        dt1d(f, d, dim_x3D_);
                        for (size_t x = 0; x < dim_x3D_; x++) grid[getIdx(x, y, z)] = d[x];
                    }
                }
                for (size_t z = 0; z < dim_z3D_; z++) {
                    for (size_t x = 0; x < dim_x3D_; x++) {
                        std::vector<float> f(dim_y3D_), d(dim_y3D_);
                        for (size_t y = 0; y < dim_y3D_; y++) f[y] = grid[getIdx(x, y, z)];
                        dt1d(f, d, dim_y3D_);
                        for (size_t y = 0; y < dim_y3D_; y++) grid[getIdx(x, y, z)] = d[y];
                    }
                }
                for (size_t y = 0; y < dim_y3D_; y++) {
                    for (size_t x = 0; x < dim_x3D_; x++) {
                        std::vector<float> f(dim_z3D_), d(dim_z3D_);
                        for (size_t z = 0; z < dim_z3D_; z++) f[z] = grid[getIdx(x, y, z)];
                        dt1d(f, d, dim_z3D_);
                        for (size_t z = 0; z < dim_z3D_; z++) grid[getIdx(x, y, z)] = d[z];
                    }
                }
            }

            void readMap3D() {
                try {
                    RCLCPP_INFO(this->get_logger(), "Loading 3D map from: %s", mapFile3D_.c_str());
                    H5File file(mapFile3D_, H5F_ACC_RDONLY);
                    DataSet dataset = file.openDataSet("map_data");
                    DataSpace dataspace = dataset.getSpace();
                    
                    std::vector<hsize_t> dims(dataspace.getSimpleExtentNdims());
                    dataspace.getSimpleExtentDims(dims.data(), NULL);
                    dim_x3D_ = dims[0]; dim_y3D_ = dims[1]; dim_z3D_ = dims[2];

                    if (dataset.attrExists("origin")) {
                        Attribute attr = dataset.openAttribute("origin");
                        float origin_buf[3]; attr.read(PredType::NATIVE_FLOAT, origin_buf);
                        mapOrigin3D_ = {(double)origin_buf[0], (double)origin_buf[1], (double)origin_buf[2]};
                    } else {
                        mapOrigin3D_ = {0.0, 0.0, 0.0};
                    }

                    size_t total_size = dim_x3D_ * dim_y3D_ * dim_z3D_;
                    std::string cacheFile = mapFile3D_ + ".sdf.bin";
                    bool cache_loaded = false;

                    std::ifstream ifs(cacheFile, std::ios::binary | std::ios::ate);
                    if (ifs.is_open()) {
                        std::streamsize size = ifs.tellg(); 
                        ifs.seekg(0, std::ios::beg);
                        if (size == static_cast<std::streamsize>(total_size * sizeof(float))) {
                            distField3D_.resize(total_size);
                            if (ifs.read(reinterpret_cast<char*>(distField3D_.data()), size)) cache_loaded = true;
                        }
                    }

                    if (!cache_loaded) {
                        RCLCPP_INFO(this->get_logger(), "Computing 3D SDF from scratch...");
                        std::vector<uint8_t> map_data(total_size);
                        dataset.read(map_data.data(), PredType::NATIVE_UINT8);

                        std::vector<float> distFieldOut(total_size, 0.0f), distFieldIn(total_size, 0.0f);
                        for (size_t i = 0; i < total_size; ++i) {
                            if (map_data[i] == 1) { distFieldOut[i] = 0.0f; distFieldIn[i] = 1e9; } 
                            else { distFieldOut[i] = 1e9; distFieldIn[i] = 0.0f; }
                        }
                        compute3DEDT(distFieldOut); compute3DEDT(distFieldIn);

                        distField3D_.resize(total_size);
                        for (size_t i = 0; i < total_size; ++i) {
                            distField3D_[i] = (std::sqrt(distFieldOut[i]) - std::sqrt(distFieldIn[i])) * mapResolution3D_;
                        }

                        std::ofstream ofs(cacheFile, std::ios::binary);
                        if (ofs.is_open()) ofs.write(reinterpret_cast<const char*>(distField3D_.data()), total_size * sizeof(float));
                    }
                    RCLCPP_INFO(this->get_logger(), "3D Distance Field ready.");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error in readMap3D: %s", e.what());
                }
            }

            void printTrajectoryOnRviz2() {
                geometry_msgs::msg::PoseStamped stamped;
                stamped.header.stamp = this->now();
                stamped.header.frame_id = path_.header.frame_id;
                stamped.pose = mclPose_;
                path_.poses.push_back(stamped);
                path_.header.stamp = stamped.header.stamp;
                pubPath_->publish(path_);
            }

            void printParticlesMakerOnRviz2() {
                sensor_msgs::msg::PointCloud2 cloud_;
                cloud_.header.stamp = this->get_clock()->now();
                cloud_.header.frame_id = "map";
                cloud_.height = 1; cloud_.width = particleNum_;
                cloud_.is_dense = false; cloud_.is_bigendian = false;

                sensor_msgs::PointCloud2Modifier modifier(cloud_);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(particleNum_);

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_, "x"), iter_y(cloud_, "y"), iter_z(cloud_, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_, "r"), iter_g(cloud_, "g"), iter_b(cloud_, "b");

                double max_w = 0.0;
                for (const auto& p : particles_) if (p.getW() > max_w) max_w = p.getW();
                if (max_w < 1e-9) max_w = 1.0; 

                for (const Particle &p : particles_) {
                    *iter_x = p.getX(); *iter_y = p.getY(); *iter_z = p.getZ(); 
                    double intensity = p.getW() / max_w;
                    if (intensity < 0.5) {
                        *iter_r = 0; *iter_g = static_cast<uint8_t>(255 * (intensity * 2.0)); *iter_b = static_cast<uint8_t>(255 * (1.0 - intensity * 2.0));
                    } else {
                        *iter_r = static_cast<uint8_t>(255 * ((intensity - 0.5) * 2.0)); *iter_g = static_cast<uint8_t>(255 * (1.0 - (intensity - 0.5) * 2.0)); *iter_b = 0;
                    }
                    ++iter_x; ++iter_y; ++iter_z; ++iter_r; ++iter_g; ++iter_b;
                }
                particleMarker_->publish(cloud_);
            }

            void publishVelocityMarker() {
                if (!cmdVel_) return;
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map"; marker.header.stamp = this->now();
                marker.ns = "velocity"; marker.id = 1;
                marker.type = visualization_msgs::msg::Marker::ARROW; marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position = mclPose_.position;

                double vel_angle = std::atan2(cmdVel_->linear.y, cmdVel_->linear.x);
                double speed = std::sqrt(cmdVel_->linear.x * cmdVel_->linear.x + cmdVel_->linear.y * cmdVel_->linear.y);

                tf2::Quaternion q_mcl(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                double r, p, current_yaw; tf2::Matrix3x3(q_mcl).getRPY(r, p, current_yaw);

                tf2::Quaternion q_marker; q_marker.setRPY(0, 0, current_yaw + vel_angle);
                marker.pose.orientation.x = q_marker.x(); marker.pose.orientation.y = q_marker.y(); marker.pose.orientation.z = q_marker.z(); marker.pose.orientation.w = q_marker.w();

                marker.scale.x = speed * 0.5 + 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05; 
                marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
                vel_marker_pub_->publish(marker);
            }

            // --- Variables ---
            int current_zaxis_level_; 
            int particleNum_;
            std::vector<Particle> particles_;
            geometry_msgs::msg::Pose mclPose_;
            std::mt19937 gen_;
            double effectiveSampleSize_;

            // 2D Map Data
            std::string mapFile2D_;
            int mapZIndex2D_, mapWidth2D_, mapHeight2D_;
            double mapResolution2D_;
            std::vector<double> mapOrigin2D_;
            cv::Mat distField2D_;

            // 3D Map Data
            std::string mapFile3D_;
            hsize_t dim_x3D_, dim_y3D_, dim_z3D_;
            double mapResolution3D_;
            std::vector<double> mapOrigin3D_;
            std::vector<float> distField3D_;

            // Sensor Data
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            nhk2026_msgs::msg::MultiLaserScan::SharedPtr scan2D_;
            std::vector<Point3D> local_points3D_;
            geometry_msgs::msg::Quaternion external_quat_;
            bool has_external_quat_ = false;

            // ROS Objects
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            nav_msgs::msg::Path path_;
            rclcpp::TimerBase::SharedPtr timer_;

            // Publishers
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubPose_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particleMarker_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapCloud2D_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_cloud_pub3D_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub3D_;

            // Subscribers
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;
            rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subZaxis_;
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subInitialPose_;
            rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subExtQuat_;
            rclcpp::Subscription<nhk2026_msgs::msg::MultiLaserScan>::SharedPtr subMultiScan_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud3D_;

            // Parameters
            double resampleThreshold_;
            double odomNoise1_2D_, odomNoise2_2D_, odomNoise3_2D_, odomNoise4_2D_;
            double odomNoise1_3D_, odomNoise2_3D_, odomNoise3_3D_, odomNoise4_3D_;
            double lfmSigma_, zHit_, zRand_;
            int scanStep_;
            bool is_sim_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::IntegratedMCLNode>());
    rclcpp::shutdown();
    return 0;
}