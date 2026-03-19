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

using namespace H5;
using namespace std::chrono_literals;

namespace mcl {
    struct Point3D {
        double x, y, z;
    };

    struct SensorTransform {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        bool valid = false; 
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
                return yaw; // Yaw角のみを返す
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

    enum class MeasurementModel { LikelihoodFieldModel };

    class MCL_3D: public rclcpp::Node{
        public:
            explicit MCL_3D(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("mcl_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
                this->declare_parameter<std::string>("mapFile", "src/nhk2026_localization/map/nhk2026_field_tamokuteki.h5"); 
                this->declare_parameter<double>("mapResolution", 0.01); 
                this->declare_parameter<std::double_t>("lfmSigma", 0.03);
                
                this->declare_parameter<int>("particleNum", 100);
                this->declare_parameter<std::float_t>("initial_x", -1.0);
                this->declare_parameter<std::float_t>("initial_y", 1.0);
                this->declare_parameter<std::float_t>("initial_z", 0.0);
                this->declare_parameter<std::float_t>("initial_theta", M_PI/2);

                this->declare_parameter<std::double_t>("zHit", 0.9);
                this->declare_parameter<std::double_t>("zRand", 0.1);
                this->declare_parameter<double>("odomNoise1", 1.5);
                this->declare_parameter<double>("odomNoise2", 1.0);
                this->declare_parameter<double>("odomNoise3", 2.0);
                this->declare_parameter<double>("odomNoise4", 1.0);
                this->declare_parameter<double>("resampleThreshold", 0.9);

                this->mapFile_ = this->get_parameter("mapFile").as_string();
                this->mapResolution_ = this->get_parameter("mapResolution").as_double();
                this->lfmSigma_ = this->get_parameter("lfmSigma").as_double();
                this->particleNum_ = this->get_parameter("particleNum").as_int();
                
                double initial_x = this->get_parameter("initial_x").as_double();
                double initial_y = this->get_parameter("initial_y").as_double();
                double initial_z = this->get_parameter("initial_z").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();

                this->zHit_ = this->get_parameter("zHit").as_double();
                this->zRand_ = this->get_parameter("zRand").as_double();
                this->odomNoise1_ = this->get_parameter("odomNoise1").as_double();
                this->odomNoise2_ = this->get_parameter("odomNoise2").as_double();
                this->odomNoise3_ = this->get_parameter("odomNoise3").as_double();
                this->odomNoise4_ = this->get_parameter("odomNoise4").as_double();
                this->resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();

                particles_.resize(particleNum_);
                measurementModel_ = MeasurementModel::LikelihoodFieldModel;

                // mclPose_の初期化
                mclPose_.position.x = initial_x;
                mclPose_.position.y = initial_y;
                mclPose_.position.z = initial_z;;
                tf2::Quaternion init_q;
                init_q.setRPY(0.0, 0.0, initial_theta);
                mclPose_.orientation.x = init_q.x();
                mclPose_.orientation.y = init_q.y();
                mclPose_.orientation.z = init_q.z();
                mclPose_.orientation.w = init_q.w();

                // 2. パーティクル群の初期位置・重みのセットアップ
                double initial_w = 1.0 / particleNum_;
                geometry_msgs::msg::Pose initialNoise;
                initialNoise.position.x = 0.3; // xの分散
                initialNoise.position.y = 0.3; // yの分散
                // quaternionのx,y,z,wを分散として流用するのは不適切なので、専用に渡す
                resetParticlesDistribution(initialNoise.position.x, initialNoise.position.y, M_PI/18.0, initial_w);

                // 各種パブリッシャの初期化
                rclcpp::QoS qos_transient(rclcpp::KeepLast(1));
                qos_transient.transient_local().reliable(); 
                sdf_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_cloud", qos_transient);
                
                rclcpp::QoS qos_default(10);
                pubPose_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", qos_default);
                pubPath_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", qos_default);
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos_default);
                vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity_marker", qos_default);
                filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan3d_cloud", qos_default);

                // サブスクライバの処理
                rclcpp::QoS cmdVelQos(rclcpp::KeepLast(10));
                subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>(
                    "cmd_vel_feedback", cmdVelQos, std::bind(&MCL_3D::cmdVelCallback, this, std::placeholders::_1)
                );

                auto lidarQos = rclcpp::SensorDataQoS();
                subCloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                    "livox/lidar", lidarQos, std::bind(&MCL_3D::pointCloudCallback, this, std::placeholders::_1)
                );

                subInitialPose_ = create_subscription<geometry_msgs::msg::Pose>(
                    "initial_pose", 10, std::bind(&MCL_3D::initialPoseCallback, this, std::placeholders::_1)
                );

                subExtQuat_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                    "quaternion_feedback", 10, std::bind(&MCL_3D::externalQuatCallback, this, std::placeholders::_1)
                );

                zaxissub_= create_subscription<std_msgs::msg::Int32MultiArray>(
                    "mcl_select",10,std::bind(&MCL_3D::lidarSelectCallback, this,std::placeholders::_1)
                );

                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
                path_.header.frame_id = "map";

                const char *sim = std::getenv("WITH_SIM");
                if (sim) {
                    RCLCPP_INFO(this->get_logger(), "Environment variable WITH_SIM is set to: %s", sim);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Environment variable WITH_SIM is NOT set.");
                }
                if (!sim || std::string(sim) != "1") {
                    is_sim_ = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "freofkprekfore");
                    is_sim_ = true;
                }

                // 地図の読み込み
                MCL_3D::readMap();
                
                timer_ = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&MCL_3D::loop, this));

                RCLCPP_INFO(this->get_logger(), "Success initialize. Initial Pose x:%f, y:%f, theta:%f", initial_x, initial_y, initial_theta);
            }

        private:
            // コールバック関数群
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVel_ = msg;
            }

            // void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            //     // 古い点群データをクリア
            //     local_points_.clear();

            //     sensor_msgs::msg::PointCloud2 cloud_out;

            //     // 1. 座標変換 (LiDAR座標系 -> base_footprint)
            //     try {
            //         geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
            //             "base_footprint", 
            //             msg->header.frame_id, 
            //             tf2::TimePointZero
            //         );
            //         tf2::doTransform(*msg, cloud_out, transform);

            //     } catch (const tf2::TransformException & ex) {
            //         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            //             "Could not transform %s to base_footprint: %s", 
            //             msg->header.frame_id.c_str(), ex.what());
            //         return;
            //     }

            //     sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x");
            //     sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_out, "y");
            //     sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_out, "z");

            //     int step = 1; 
            //     int count = 0;

            //     // --- 動的床面フィルタのための準備 ---
            //     std::unordered_map<int, int> z_histogram;
            //     std::vector<Point3D> valid_range_points;
            //     const double BIN_SIZE = 0.05; // 5cm刻みで高さを評価

            //     // --- 第1パス: 範囲内の点群抽出とヒストグラムの作成 ---
            //     for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            //         count++;
            //         if (count % step != 0) continue;

            //         float x = *iter_x;
            //         float y = *iter_y;
            //         float z = *iter_z;

            //         if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
                    
            //         // 距離と高さの基本フィルタ (0.45m〜3.0mの範囲、高さ1.5m以下)
            //         double dist_sq = x*x + y*y + z*z;
            //         if (dist_sq < 0.45*0.45 || dist_sq > 3.0*3.0) continue; 
            //         if (z > 1.5) continue;

            //         // マップ上の絶対的なZ座標を計算
            //         double z_map = z + mclPose_.position.z;

            //         Point3D pt;
            //         pt.x = x;
            //         pt.y = y;
            //         pt.z = z;
            //         valid_range_points.push_back(pt);

            //         // ヒストグラムのカウントを増やす
            //         int z_bin = static_cast<int>(std::floor(z_map / BIN_SIZE));
            //         z_histogram[z_bin]++;
            //     }

            //     // --- 閾値の計算 ---
            //     // 有効な点群全体の5%が同じ高さに集中していれば「床」とみなす (最低でも50点は必要とする)
            //     int floor_threshold = std::max(50, static_cast<int>(valid_range_points.size() * 0.03));

            //     // --- 第2パス: 床面と判定された高さの点群を除外 ---
            //     for (const auto& pt : valid_range_points) {
            //         double z_map = pt.z + mclPose_.position.z;
            //         int z_bin = static_cast<int>(std::floor(z_map / BIN_SIZE));

            //         // その点の高さの集中度が閾値を超えている場合（床）はスキップ
            //         // ※少し傾斜がある環境で弾き残しが出る場合は、隣のビンも確認すると強力になります：
            //         // if (z_histogram[z_bin] > floor_threshold || z_histogram[z_bin - 1] > floor_threshold || z_histogram[z_bin + 1] > floor_threshold)
            //         if (z_histogram[z_bin] > floor_threshold) {
            //             continue;
            //         }

            //         local_points_.push_back(pt);
            //     }

            //     // デバッグ用: 抽出結果の確認
            //     // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            //     //    "Total valid points: %zu, Extracted non-floor points: %zu", 
            //     //    valid_range_points.size(), local_points_.size());

            //     // --- 抽出した点群のパブリッシュ ---
            //     if (!local_points_.empty()) {
            //         sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
            //         filtered_cloud_msg.header.stamp = msg->header.stamp; 
            //         filtered_cloud_msg.header.frame_id = "base_footprint"; 
            //         filtered_cloud_msg.height = 1;
            //         filtered_cloud_msg.width = local_points_.size();
            //         filtered_cloud_msg.is_dense = true;
            //         filtered_cloud_msg.is_bigendian = false;

            //         sensor_msgs::PointCloud2Modifier modifier(filtered_cloud_msg);
            //         modifier.setPointCloud2FieldsByString(1, "xyz");
            //         modifier.resize(local_points_.size());

            //         sensor_msgs::PointCloud2Iterator<float> out_x(filtered_cloud_msg, "x");
            //         sensor_msgs::PointCloud2Iterator<float> out_y(filtered_cloud_msg, "y");
            //         sensor_msgs::PointCloud2Iterator<float> out_z(filtered_cloud_msg, "z");

            //         for (const auto& pt : local_points_) {
            //             *out_x = pt.x;
            //             *out_y = pt.y;
            //             *out_z = pt.z;
            //             ++out_x; ++out_y; ++out_z;
            //         }

            //         filtered_cloud_pub_->publish(filtered_cloud_msg);
            //     }
            // }
            
            void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                // 古い点群データをクリア
                local_points_.clear();

                sensor_msgs::msg::PointCloud2 cloud_out;

                try {
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                        "base_footprint", 
                        msg->header.frame_id, 
                        tf2::TimePointZero
                    );

                    
                    tf2::doTransform(*msg, cloud_out, transform);

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Could not transform %s to base_footprint: %s", 
                        msg->header.frame_id.c_str(), ex.what());
                    return;
                }

                
                sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_out, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_out, "z");

            
                int step = 1; 
                int count = 0;

                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                    count++;
                    if (count % step != 0) continue;

                    float x = *iter_x;
                    float y = *iter_y;
                    float z = *iter_z;

                    
                    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
                    
                    double dist_sq = x*x + y*y + z*z;
                    if (dist_sq < 0.45*0.45 || dist_sq > 3.0*3.0) continue; // 0.2m以下、30m以上は無視

                    if ( z > 1.5) continue;

                    double z_map = z + mclPose_.position.z;

                    // z_mapが 0.0m, 0.20m, 0.40m の +-0.01m (1cm) の範囲内なら除外
                    if (std::abs(z_map - 0.00) <= 0.03 ||
                        std::abs(z_map - 0.20) <= 0.03 ||
                        std::abs(z_map - 0.40) <= 0.03) {
                        continue;
                    }

                    // if (std::abs(z_map - 0.00) <= 0.03){
                    //     continue;
                    // }

                    Point3D pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    local_points_.push_back(pt);
                }

                // デバッグ用: 何点抽出されたか表示
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                //    "Extracted %zu points for MCL.", local_points_.size());

                if (!local_points_.empty()) {
                    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
                    filtered_cloud_msg.header.stamp = msg->header.stamp; // 元のLiDARのタイムスタンプを使用
                    filtered_cloud_msg.header.frame_id = "base_footprint"; // 変換済みの座標系
                    filtered_cloud_msg.height = 1;
                    filtered_cloud_msg.width = local_points_.size();
                    filtered_cloud_msg.is_dense = true;
                    filtered_cloud_msg.is_bigendian = false;

                    sensor_msgs::PointCloud2Modifier modifier(filtered_cloud_msg);
                    modifier.setPointCloud2FieldsByString(1, "xyz");
                    modifier.resize(local_points_.size());

                    sensor_msgs::PointCloud2Iterator<float> out_x(filtered_cloud_msg, "x");
                    sensor_msgs::PointCloud2Iterator<float> out_y(filtered_cloud_msg, "y");
                    sensor_msgs::PointCloud2Iterator<float> out_z(filtered_cloud_msg, "z");

                    for (const auto& pt : local_points_) {
                        *out_x = pt.x;
                        *out_y = pt.y;
                        *out_z = pt.z;
                        ++out_x; ++out_y; ++out_z;
                    }

                    filtered_cloud_pub_->publish(filtered_cloud_msg);
                }
            }

            void initialPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
                // 1. 推定位置 (mclPose_) を受信したメッセージで更新
                // msgは Pose型なので、そのまま position と orientation をコピー
                mclPose_.position = msg->position;
                mclPose_.orientation = msg->orientation;

                // クォータニオンからYaw角を抽出（パーティクル散布時の計算用）
                tf2::Quaternion q(
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // 2. パーティクルを再散布
                // ここでの数値（0.1m, 5度など）は、初期位置の「確信度」に合わせて調整してください
                double noise_x = 0.1;           // xの標準偏差 [m]
                double noise_y = 0.1;           // yの標準偏差 [m]
                double noise_yaw = 5.0 * M_PI / 180.0; // yawの標準偏差 [rad] (5度)
                double initial_w = 1.0 / static_cast<double>(particles_.size());

                // 既存の散布関数を呼び出す（引数をシンプルに整理）
                resetParticlesDistribution(noise_x, noise_y, noise_yaw, initial_w);

                RCLCPP_INFO(this->get_logger(), 
                            "Pose reset: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", 
                            mclPose_.position.x, mclPose_.position.y, mclPose_.position.z, yaw);
            }

            void externalQuatCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 4) {
                    external_quat_.w = msg->data[0];
                    external_quat_.x = msg->data[1];
                    external_quat_.y = msg->data[2];
                    external_quat_.z = msg->data[3];
                    //has_external_quat_ = true;

                    
                    double siny_cosp = 2.0 * (external_quat_.w * external_quat_.z + external_quat_.x * external_quat_.y);
                    double cosy_cosp = 1.0 - 2.0 * (external_quat_.y * external_quat_.y + external_quat_.z * external_quat_.z);
                    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
                    double yaw_deg = yaw_rad * (180.0 / M_PI);

                    //RCLCPP_INFO(this->get_logger(), "Calculated Yaw: [rad: %.3f, deg: %.1f]", yaw_rad, yaw_deg);
                }
            }
            void lidarSelectCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
                zaxics_ = msg;
            }
            
            double randNormal(double sigma) {
                if (sigma <= 0.0) return 0.0;
                std::normal_distribution<double> dist(0.0, sigma);
                return dist(gen_);
            }

            // 初期位置周辺にパーティクルを散らす (3D用に引数を変更)
            void resetParticlesDistribution(double noise_x, double noise_y, double noise_theta, double initial_w) {
                for (std::size_t i = 0; i < particles_.size(); i++ ) {
                    double x = mclPose_.position.x + randNormal(noise_x);
                    double y = mclPose_.position.y + randNormal(noise_y);
                    
                    // 現在のmclPose_のYawを取得
                    tf2::Quaternion q(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                    tf2::Matrix3x3 m(q);
                    double r, p, yaw;
                    m.getRPY(r, p, yaw);

                    double theta = yaw + randNormal(noise_theta);
                    
                    particles_[i].setPose(x, y, mclPose_.position.z, theta);
                    particles_[i].setW(initial_w);
                }
            }

            void loop() {
                rclcpp::Time current_time = this->get_clock()->now();
                if (!zaxics_) {
                } 
                else if (!zaxics_->data.empty()) {
                    if (zaxics_->data[0] == 0) { 
                        RCLCPP_INFO(this->get_logger(), "MCL3d is disabled by mcl_select");
                        return;
                    }
                }
                
                if (!cmdVel_) return;

                double dt = 0.100; // 100ms
                            
                std::double_t vx_ = cmdVel_->linear.x;
                std::double_t vy_ = cmdVel_->linear.y;
                std::double_t omega_ = cmdVel_->angular.z;
                
                geometry_msgs::msg::Twist delta_;
                delta_.linear.x = vx_ * dt;
                delta_.linear.y = vy_ * dt;
                delta_.angular.z = omega_ * dt;

                updateParticles(delta_);
                printParticlesMakerOnRviz2();
               
                caculateMeasurementModel();
                
                estimatePose();
                resampleParticles();
                printTrajectoryOnRviz2();
                publishVelocityMarker();
                
            }

            // 1次元の2乗距離変換
            void dt1d(const std::vector<float>& f, std::vector<float>& d, int n) {
                std::vector<int> v(n);
                std::vector<float> z(n + 1);
                int k = 0;
                v[0] = 0;
                z[0] = -std::numeric_limits<float>::infinity();
                z[1] = std::numeric_limits<float>::infinity();
                
                for (int q = 1; q < n; q++) {
                    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
                    while (s <= z[k]) {
                        k--;
                        if (k < 0) { k = 0; break; }
                        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
                    }
                    k++;
                    v[k] = q;
                    z[k] = s;
                    z[k + 1] = std::numeric_limits<float>::infinity();
                }
                
                k = 0;
                for (int q = 0; q < n; q++) {
                    while (z[k + 1] < q) k++;
                    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
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
                //RCLCPP_INFO(this->get_logger(), "%.4f %.4f  %.4f %.4f", x, y,z, theta);
            }

            void updateParticles(geometry_msgs::msg::Twist delta) {
                std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y * delta.linear.y;
                std::double_t dy2 = delta.angular.z * delta.angular.z;

                // 位置のノイズ用標準偏差
                std::double_t sigma_xy = std::sqrt(odomNoise1_ * dd2 + odomNoise2_ * dy2);
                
                // 外部クォータニオンからYawを取得（一度だけ計算）
                double ext_yaw = 0.0;
                if (has_external_quat_) {
                    tf2::Quaternion q(external_quat_.x, external_quat_.y, external_quat_.z, external_quat_.w);
                    tf2::Matrix3x3 m(q);
                    double r, p;
                    m.getRPY(r, p, ext_yaw);
                }

                for (size_t i = 0; i < this->particles_.size(); i++) {
                    std::double_t dx = delta.linear.x + randNormal(sigma_xy);
                    std::double_t dy = delta.linear.y + randNormal(sigma_xy);
                    
                    std::double_t theta_old = this->particles_[i].getTheta();
                    
                    // 移動後の位置計算
                    std::double_t x_ = this->particles_[i].getX() + std::cos(theta_old) * dx - std::sin(theta_old) * dy;
                    std::double_t y_ = this->particles_[i].getY() + std::sin(theta_old) * dx + std::cos(theta_old) * dy;
                    std::double_t z_ = this->particles_[i].getZ();

                    // 向きの更新
                    std::double_t theta_new;
                    if (has_external_quat_) {
                        // 外部姿勢に、パーティクルごとの微小なバラつき(ノイズ)を乗せる
                        // これにより、IMUに僅かな誤差があってもスキャンマッチングで補正しやすくなる
                        theta_new = ext_yaw + randNormal(0.005); // 0.005rad程度の微小ノイズ
                    } else {
                        // 外部姿勢がない場合は従来のオドメトリ
                        std::double_t sigma_theta = std::sqrt(odomNoise3_ * dd2 + odomNoise4_ * dy2);
                        theta_new = theta_old + delta.angular.z + randNormal(sigma_theta);
                    }

                    particles_[i].setPose(x_, y_, z_, theta_new);
                }
            }
            void resampleParticles(void) {
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

            void printTrajectoryOnRviz2() {
                geometry_msgs::msg::PoseStamped stamped;
                stamped.header.stamp = this->now();
                stamped.header.frame_id = path_.header.frame_id;
                
                stamped.pose.position.x = mclPose_.position.x;
                stamped.pose.position.y = mclPose_.position.y;
                stamped.pose.position.z = mclPose_.position.z;
                stamped.pose.orientation = mclPose_.orientation;

                path_.poses.push_back(stamped);
                path_.header.stamp = stamped.header.stamp;

                pubPath_->publish(path_);
            }
            void printParticlesMakerOnRviz2() {
                sensor_msgs::msg::PointCloud2 cloud_;
                cloud_.header.stamp = this->get_clock()->now();
                cloud_.header.frame_id = "map";
                cloud_.height = 1;
                cloud_.width = particleNum_;
                cloud_.is_dense = false;
                cloud_.is_bigendian = false;

                sensor_msgs::PointCloud2Modifier modifier(cloud_);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(particleNum_);

                sensor_msgs::PointCloud2Iterator<std::float_t> iter_x(cloud_, "x");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_y(cloud_, "y");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_z(cloud_, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_r(cloud_, "r");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_g(cloud_, "g");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_b(cloud_, "b");

                // 色を強調するために、現在のパーティクル群の中での最大重みを探す
                double max_w = 0.0;
                for (const auto& p : particles_) {
                    if (p.getW() > max_w) max_w = p.getW();
                }
                if (max_w < 1e-9) max_w = 1.0; // ゼロ除算防止

                for (const Particle &p : particles_) {
                    *iter_x = p.getX();
                    *iter_y = p.getY();
                    *iter_z = p.getZ(); 

                    // 尤度の相対的な強さを 0.0 ~ 1.0 で計算
                    double intensity = p.getW() / max_w;

                    // 簡易的なカラーマップ (青 -> 緑 -> 赤)
                    if (intensity < 0.5) {
                        // 低尤度: 青から緑へ
                        *iter_r = 0;
                        *iter_g = static_cast<uint8_t>(255 * (intensity * 2.0));
                        *iter_b = static_cast<uint8_t>(255 * (1.0 - intensity * 2.0));
                    } else {
                        // 高尤度: 緑から赤へ
                        *iter_r = static_cast<uint8_t>(255 * ((intensity - 0.5) * 2.0));
                        *iter_g = static_cast<uint8_t>(255 * (1.0 - (intensity - 0.5) * 2.0));
                        *iter_b = 0;
                    }

                    ++iter_x, ++iter_y, ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                particleMarker_->publish(cloud_);
            }
          
            
            void publishVelocityMarker() {
                if (!cmdVel_) return;

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "velocity";
                marker.id = 1;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = mclPose_.position.x;
                marker.pose.position.y = mclPose_.position.y;
                marker.pose.position.z = mclPose_.position.z; 

                double vel_angle = std::atan2(cmdVel_->linear.y, cmdVel_->linear.x);
                double speed = std::sqrt(cmdVel_->linear.x * cmdVel_->linear.x + cmdVel_->linear.y * cmdVel_->linear.y);

                tf2::Quaternion q_mcl(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                tf2::Matrix3x3 m(q_mcl);
                double r, p, current_yaw;
                m.getRPY(r, p, current_yaw);

                tf2::Quaternion q_marker;
                q_marker.setRPY(0, 0, current_yaw + vel_angle);
                marker.pose.orientation.x = q_marker.x();
                marker.pose.orientation.y = q_marker.y();
                marker.pose.orientation.z = q_marker.z();
                marker.pose.orientation.w = q_marker.w();

                marker.scale.x = speed * 0.5 + 0.05; 
                marker.scale.y = 0.05; 
                marker.scale.z = 0.05; 

                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                vel_marker_pub_->publish(marker);
            }

            void readMap() {
                try {
                    RCLCPP_INFO(this->get_logger(), "Loading 3D map metadata from: %s", this->mapFile_.c_str());
                    
                    // 1. マップのサイズや原点などのメタデータは常にHDF5から読み込む
                    H5File file(this->mapFile_, H5F_ACC_RDONLY);
                    DataSet dataset = file.openDataSet("map_data");
                    DataSpace dataspace = dataset.getSpace();
                    
                    int rank = dataspace.getSimpleExtentNdims();
                    std::vector<hsize_t> dims(rank);
                    dataspace.getSimpleExtentDims(dims.data(), NULL);

                    dim_x_ = dims[0];
                    dim_y_ = dims[1];
                    dim_z_ = dims[2];
                    mapWidth_  = dim_x_;
                    mapHeight_ = dim_y_;

                    RCLCPP_INFO(this->get_logger(), "3D Map Shape: (%llu, %llu, %llu)", dim_x_, dim_y_, dim_z_);

                    if (mapResolution_ <= 0.0) mapResolution_ = 0.01;
                    if (dataset.attrExists("origin")) {
                        Attribute attr = dataset.openAttribute("origin");
                        float origin_buf[3];
                        attr.read(PredType::NATIVE_FLOAT, origin_buf);
                        mapOrigin_ = {(double)origin_buf[0], (double)origin_buf[1], (double)origin_buf[2]};
                    } else {
                        mapOrigin_ = {0.0, 0.0, 0.0};
                    }

                    size_t total_size = dim_x_ * dim_y_ * dim_z_;

                    // --- 追加: SDFキャッシュの読み書き処理 ---
                    // キャッシュファイル名の生成 (元のHDF5ファイル名 + ".sdf.bin")
                    std::string cacheFile = this->mapFile_ + ".sdf.bin";
                    bool cache_loaded = false;

                    // キャッシュファイルをバイナリモードで開き、存在確認とサイズ確認を行う
                    std::ifstream ifs(cacheFile, std::ios::binary | std::ios::ate);
                    if (ifs.is_open()) {
                        std::streamsize size = ifs.tellg(); // ファイルサイズを取得
                        ifs.seekg(0, std::ios::beg);
                        
                        // ファイルサイズが現在のマップ設定と一致するか確認
                        if (size == static_cast<std::streamsize>(total_size * sizeof(float))) {
                            RCLCPP_INFO(this->get_logger(), "Found SDF cache! Loading from: %s", cacheFile.c_str());
                            distField3D_.resize(total_size);
                            if (ifs.read(reinterpret_cast<char*>(distField3D_.data()), size)) {
                                cache_loaded = true;
                                RCLCPP_INFO(this->get_logger(), "SDF cache loaded successfully (Lightning fast!).");
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Failed to read SDF cache. Recalculating...");
                            }
                        } else {
                            RCLCPP_WARN(this->get_logger(), "SDF cache size mismatch (Map might have changed). Recalculating...");
                        }
                    }

                    // キャッシュが読めなかった場合（初回起動時、またはマップが更新された時）は計算する
                    if (!cache_loaded) {
                        RCLCPP_INFO(this->get_logger(), "Computing 3D SDF from scratch... This may take a few seconds.");

                        std::vector<uint8_t> map_data(total_size);
                        dataset.read(map_data.data(), PredType::NATIVE_UINT8);

                        const float INF = 1e9;
                        std::vector<float> distFieldOut(total_size, 0.0f);
                        std::vector<float> distFieldIn(total_size, 0.0f);

                        for (size_t i = 0; i < total_size; ++i) {
                            if (map_data[i] == 1) {
                                distFieldOut[i] = 0.0f;
                                distFieldIn[i]  = INF;
                            } else {
                                distFieldOut[i] = INF;
                                distFieldIn[i]  = 0.0f;
                            }
                        }

                        compute3DEDT(distFieldOut);
                        compute3DEDT(distFieldIn);

                        distField3D_.resize(total_size);
                        for (size_t i = 0; i < total_size; ++i) {
                            float d_out = std::sqrt(distFieldOut[i]);
                            float d_in  = std::sqrt(distFieldIn[i]);
                            distField3D_[i] = (d_out - d_in) * mapResolution_;
                        }

                        // 計算結果をバイナリファイルとして保存
                        std::ofstream ofs(cacheFile, std::ios::binary);
                        if (ofs.is_open()) {
                            ofs.write(reinterpret_cast<const char*>(distField3D_.data()), total_size * sizeof(float));
                            RCLCPP_INFO(this->get_logger(), "Saved computed SDF to cache: %s", cacheFile.c_str());
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Failed to save SDF cache. Check directory permissions.");
                        }

                        RCLCPP_INFO(this->get_logger(), "3D Distance Field successfully created.");
                    }
                    
                    publishSDFCloud();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error in readMap: %s", e.what());
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Unknown Error in readMap");
                }
            }

            void compute3DEDT(std::vector<float>& grid) {
                for (size_t z = 0; z < dim_z_; z++) {
                    for (size_t y = 0; y < dim_y_; y++) {
                        std::vector<float> f(dim_x_), d(dim_x_);
                        for (size_t x = 0; x < dim_x_; x++) f[x] = grid[getIdx3D(x, y, z)];
                        dt1d(f, d, dim_x_);
                        for (size_t x = 0; x < dim_x_; x++) grid[getIdx3D(x, y, z)] = d[x];
                    }
                }
                for (size_t z = 0; z < dim_z_; z++) {
                    for (size_t x = 0; x < dim_x_; x++) {
                        std::vector<float> f(dim_y_), d(dim_y_);
                        for (size_t y = 0; y < dim_y_; y++) f[y] = grid[getIdx3D(x, y, z)];
                        dt1d(f, d, dim_y_);
                        for (size_t y = 0; y < dim_y_; y++) grid[getIdx3D(x, y, z)] = d[y];
                    }
                }
                for (size_t y = 0; y < dim_y_; y++) {
                    for (size_t x = 0; x < dim_x_; x++) {
                        std::vector<float> f(dim_z_), d(dim_z_);
                        for (size_t z = 0; z < dim_z_; z++) f[z] = grid[getIdx3D(x, y, z)];
                        dt1d(f, d, dim_z_);
                        for (size_t z = 0; z < dim_z_; z++) grid[getIdx3D(x, y, z)] = d[z];
                    }
                }
            }

            void caculateMeasurementModel() {
                if (local_points_.empty()) return;

                // 各パーティクルの合計対数尤度を格納する配列 (サイズはパーティクル数のみ)
                std::vector<double> log_weights(particleNum_);
                double max_log_weight = -std::numeric_limits<double>::infinity();

                // 1. 各パーティクルの対数尤度を直接計算
                for (std::size_t i = 0; i < particles_.size(); i++) {
                    // ここで vector を返さず、double を受け取る
                    log_weights[i] = caculateLogLikelihood(particles_[i].getPose(), local_points_);
                    
                    if (log_weights[i] > max_log_weight) {
                        max_log_weight = log_weights[i];
                    }
                }

                // 2. 正規化処理 (Log-Sum-Expトリック)
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

                // 有効サンプルサイズの更新
                effectiveSampleSize_ = 1.0 / w_sq_sum;
            }

            double caculateLogLikelihood(const geometry_msgs::msg::Pose& pose, const std::vector<Point3D>& local_points) {
                double total_log_p = 0.0;
                double var = lfmSigma_ * lfmSigma_;
                double normConst = 1.0 / (std::sqrt(2.0 * M_PI * var));
                
                // パラメータ
                const double scan_range_max = 30.0; 
                const double pRand_const = (1.0 / scan_range_max) * mapResolution_;

                // パーティクルの姿勢から回転行列の要素を計算 (ループ外で1回だけ)
                tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                tf2::Matrix3x3 m(q);
                double r, p, yaw;
                m.getRPY(r, p, yaw);
                double cos_theta = std::cos(yaw);
                double sin_theta = std::sin(yaw);

                for (const auto& pt : local_points) {
                    // ロボット座標系からマップ座標系への変換
                    double map_x = pt.x * cos_theta - pt.y * sin_theta + pose.position.x;
                    double map_y = pt.x * sin_theta + pt.y * cos_theta + pose.position.y;
                    double map_z = pt.z + pose.position.z; 

                    int u, v, w;
                    xyz2uvw(map_x, map_y, map_z, &u, &v, &w);

                    double prob = zRand_ * pRand_const; 

                    if (u >= 0 && u < static_cast<int>(dim_x_) && 
                        v >= 0 && v < static_cast<int>(dim_y_) && 
                        w >= 0 && w < static_cast<int>(dim_z_)) {
                        
                        double sdf_val = static_cast<double>(distField3D_[getIdx3D(u, v, w)]);
                        
                        // 動的障害物除外 (壁から遠すぎる点は無視)
                        if (sdf_val > 0.5) {
                            total_log_p += std::log(prob);
                            continue;
                        }

                        // 壁の中(マイナス)ならペナルティを付与
                        double d = (sdf_val >= 0.0) ? sdf_val : (std::abs(sdf_val) + 1.0);
                        
                        double pHit = normConst * std::exp(-(d * d) / (2.0 * var)) * mapResolution_;
                        prob = std::min(1.0, zHit_ * pHit + zRand_ * pRand_const);
                    }
                    
                    // 対数尤度を加算 (log(0) 回避のために微小値を std::max で保証)
                    total_log_p += std::log(std::max(prob, 1e-10));
                }

                return total_log_p;
            }

            void publishSDFCloud() {
                if (distField3D_.empty()) return;

                RCLCPP_INFO(this->get_logger(), "Generating PointCloud for 3D SDF visualization (1/1000 downsampled)...");

                struct PointRGB {
                    float x, y, z;
                    uint8_t r, g, b;
                };
                std::vector<PointRGB> viz_points;

                const float MAX_DIST = 1.0f;

                for (size_t z = 0; z < dim_z_; ++z) {
                    for (size_t y = 0; y < dim_y_; ++y) {
                        for (size_t x = 0; x < dim_x_; ++x) {
                            
                            if (x % 10 != 0 || y % 10 != 0 || z % 10 != 0) {
                                continue;
                            }

                            float d = distField3D_[getIdx3D(x, y, z)];

                            PointRGB p;
                            p.x = static_cast<float>(x) * mapResolution_ + mapOrigin_[0];
                            p.y = static_cast<float>(y) * mapResolution_ + mapOrigin_[1];
                            p.z = static_cast<float>(z) * mapResolution_ + mapOrigin_[2];

                            if (std::abs(d) < 0.05f) {
                                p.r = 0; p.g = 255; p.b = 0;
                            } else if (d < 0.0f) {
                                float ratio = std::max(0.0f, 1.0f - (std::abs(d) / MAX_DIST));
                                p.r = static_cast<uint8_t>(100 + 155 * ratio); 
                                p.g = 0;
                                p.b = 0;
                            } else {
                                float ratio = std::max(0.0f, 1.0f - (d / MAX_DIST));
                                p.r = 0;
                                p.g = 0;
                                p.b = static_cast<uint8_t>(100 + 155 * ratio); 
                            }
                            viz_points.push_back(p);
                        }
                    }
                }

                sensor_msgs::msg::PointCloud2 cloud_msg;
                cloud_msg.header.stamp = this->now();
                cloud_msg.header.frame_id = "map";
                cloud_msg.height = 1;
                cloud_msg.width = viz_points.size();
                cloud_msg.is_dense = true;
                cloud_msg.is_bigendian = false;

                sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(viz_points.size());

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

                for (const auto& vp : viz_points) {
                    *iter_x = vp.x; *iter_y = vp.y; *iter_z = vp.z;
                    *iter_r = vp.r; *iter_g = vp.g; *iter_b = vp.b;
                    
                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                sdf_cloud_pub_->publish(cloud_msg);
                RCLCPP_INFO(this->get_logger(), "Published Downsampled SDF PointCloud with %zu points.", viz_points.size());
            }

            //  変数 
            int particleNum_;
            std::vector<Particle> particles_;
            MeasurementModel measurementModel_;
            std::double_t totalLikelihood_;
            int maxLikelihoodParticleIdx_;
            std::double_t effectiveSampleSize_;

            std::string mapFile_;
            double mapResolution_;
            int mapWidth_, mapHeight_;
            std::vector<double> mapOrigin_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

            hsize_t dim_x_, dim_y_, dim_z_;
            std::vector<float> distField3D_; 

            inline size_t getIdx3D(int x, int y, int z) const {
                return static_cast<size_t>(x) * (dim_y_ * dim_z_) + static_cast<size_t>(y) * dim_z_ + z;
            }

            inline void xyz2uvw(double x, double y, double z, int *u, int *v, int *w) const {
                *u = static_cast<int>((x - mapOrigin_[0]) / mapResolution_);
                // int v_relative = static_cast<int>((y - mapOrigin_[1]) / mapResolution_);
                // *v = mapHeight_ - 1 - v_relative;
                *v = static_cast<int>((y - mapOrigin_[1]) / mapResolution_);
                *w = static_cast<int>((z - mapOrigin_[2]) / mapResolution_);
            }

            // パブリッシャ / サブスクライバ / タイマー
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_cloud_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubPose_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particleMarker_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subCloud_;
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subInitialPose_;
            rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subExtQuat_;
            rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr zaxissub_;
             
            // 状態変数
            geometry_msgs::msg::Pose mclPose_;
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            nav_msgs::msg::Path path_;
            std::vector<Point3D> local_points_;
            std::mt19937 gen_; 
            std_msgs::msg::Int32MultiArray::SharedPtr zaxics_ = nullptr; 


           
            geometry_msgs::msg::Quaternion external_quat_;
            bool has_external_quat_ = false;

            // 各種パラメータ
            std::double_t zHit_, zShort_, zMax_, zRand_;
            std::double_t lfmSigma_;
            double odomNoise1_, odomNoise2_, odomNoise3_, odomNoise4_;
            double resampleThreshold_;
            bool is_sim_; 
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL_3D>());
    rclcpp::shutdown();
    return 0;
}