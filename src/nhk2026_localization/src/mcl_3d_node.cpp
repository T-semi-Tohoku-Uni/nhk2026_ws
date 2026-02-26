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

using namespace H5;
using namespace std::chrono_literals;

namespace mcl {
    struct Point3D {
        double x, y, z;
    };

    class Particle {
        public:
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

            // 修正: RollとPitchの引数を削除し、内部で完全に 0.0 に固定する
            void setPose(double x, double y, double z, double yaw) {
                pose_.position.x = x;
                pose_.position.y = y;
                pose_.position.z = z;
                
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw); // ここでRollとPitchを0にロック！
                
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
                this->declare_parameter<std::double_t>("zHit", 0.9);
                this->declare_parameter<std::double_t>("zRand", 0.1);
                this->declare_parameter<double>("odomNoise1", 1.0);
                this->declare_parameter<double>("odomNoise2", 1.0);
                this->declare_parameter<double>("odomNoise3", 1.0);
                this->declare_parameter<double>("odomNoise4", 1.0);
                this->declare_parameter<double>("resampleThreshold", 0.5);

                this->mapFile_ = this->get_parameter("mapFile").as_string();
                this->mapResolution_ = this->get_parameter("mapResolution").as_double();
                this->lfmSigma_ = this->get_parameter("lfmSigma").as_double();
                this->particleNum_ = this->get_parameter("particleNum").as_int();
                this->zHit_ = this->get_parameter("zHit").as_double();
                this->zRand_ = this->get_parameter("zRand").as_double();

                this->odomNoise1_ = this->get_parameter("odomNoise1").as_double();
                this->odomNoise2_ = this->get_parameter("odomNoise2").as_double();
                this->odomNoise3_ = this->get_parameter("odomNoise3").as_double();
                this->odomNoise4_ = this->get_parameter("odomNoise4").as_double();
                this->resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();

                particles_.resize(particleNum_);
                measurementModel_ = MeasurementModel::LikelihoodFieldModel;

                
                mclPose_.position.x = 0.0; mclPose_.position.y = 0.0; mclPose_.position.z = 0.0;
                mclPose_.orientation.x = 0.0; mclPose_.orientation.y = 0.0; mclPose_.orientation.z = 0.0; mclPose_.orientation.w = 1.0;

                // 各種パブリッシャの初期化
                rclcpp::QoS qos_transient(rclcpp::KeepLast(1));
                qos_transient.transient_local().reliable(); 
                sdf_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_cloud", qos_transient);
                
                rclcpp::QoS qos_default(10);
                pubPose_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", qos_default);
                pubPath_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", qos_default);
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", qos_default);
                vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity_marker", qos_default);

                //サブスクライバの処理
                rclcpp::QoS cmdVelQos(rclcpp::KeepLast(10));
                subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel_feedback", cmdVelQos, std::bind(&MCL_3D::cmdVelCallback, this, std::placeholders::_1)
                );

                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
                path_.header.frame_id = "map";

                // 地図の読み込み
                MCL_3D::readMap();

                
                timer_ = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&MCL_3D::loop, this));

                RCLCPP_INFO(this->get_logger(), "Success initialize");
            }

        private:
            //コールバック関数群
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVel_ = msg;
            }
            double randNormal(double sigma) {
                if (sigma <= 0.0) return 0.0;
                std::normal_distribution<double> dist(0.0, sigma);
                return dist(gen_);
            }

            void loop() {
                rclcpp::Time current_time = this->get_clock()->now();
                
                // cmdVel_ が受信されていなければスキップ
                if (!cmdVel_) return;

                double dt = 0.100; // 簡易的に固定 (実際の時間差分を取るのが望ましい)
                            
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
                // 現在のmclPose_からYawを取得
                tf2::Quaternion q_old(mclPose_.orientation.x, mclPose_.orientation.y, mclPose_.orientation.z, mclPose_.orientation.w);
                tf2::Matrix3x3 m(q_old);
                double roll, pitch, tmpTheta;
                m.getRPY(roll, pitch, tmpTheta);

                std::double_t x = 0.0, y = 0.0, z = 0.0, theta = 0.0;
                for (size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t w = particles_[i].getW();
                    x += particles_[i].getX() * w;
                    y += particles_[i].getY() * w;
                    z += particles_[i].getZ() * w; // 3D対応
                    
                    std::double_t dTheta = tmpTheta - particles_[i].getTheta();
                    while (dTheta < -M_PI) dTheta += 2.0*M_PI;
                    while (dTheta > M_PI) dTheta -= 2.0*M_PI;
                    theta += dTheta * w;
                }
                theta = tmpTheta - theta;

                // mclPose_ (geometry_msgs::msg::Pose) の更新
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
                    tf_msg.transform.translation.z = z; // 3D対応
                    tf_msg.transform.rotation = mclPose_.orientation;

                    tf_broadcaster_->sendTransform(tf_msg);
                }
            }

            void updateParticles(geometry_msgs::msg::Twist delta) {
                std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y * delta.linear.y; 
                std::double_t dy2 = delta.angular.z * delta.angular.z;

                std::double_t sigma_xy = std::sqrt(odomNoise1_*dd2 + odomNoise2_*dy2);
                std::double_t sigma_theta = std::sqrt(odomNoise3_ * dd2 + odomNoise4_ * dy2);

                for (size_t i = 0; i < this->particles_.size(); i++ ) {
                    std::double_t dx = delta.linear.x + randNormal(sigma_xy);
                    std::double_t dy = delta.linear.y + randNormal(sigma_xy);
                    std::double_t dtheta = delta.angular.z + randNormal(sigma_theta);

                    std::double_t theta_ = this->particles_[i].getTheta();
                    std::double_t x_ = this->particles_[i].getX() + std::cos(theta_)*dx - std::sin(theta_)*dy;
                    std::double_t y_ = this->particles_[i].getY() + std::sin(theta_)*dx + std::cos(theta_)*dy;
                    std::double_t z_ = this->particles_[i].getZ(); // Zはそのまま維持
                    
                    theta_ += dtheta;
                    
                    // 修正: 引数が x, y, z, yaw の4つだけになる
                    particles_[i].setPose(x_, y_, z_, theta_);
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
                            // 修正: 引数が x, y, z, yaw の4つだけになる
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
                
                // Pose2DではなくPoseを使用
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

                for (const Particle &p: particles_) {
                    *iter_x = p.getX();
                    *iter_y = p.getY();
                    *iter_z = p.getZ(); // 3D対応

                    *iter_r = 0;
                    *iter_g = 0;
                    *iter_b = int(p.getW()*255);

                    ++iter_x, ++iter_y, ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                particleMarker_->publish(cloud_);
            }
            
            
            void publishVelocityMarker() {
                // visualization_msgs::msg::Marker などの処理
            }

            void readMap() {
                try {
                    RCLCPP_INFO(this->get_logger(), "Loading 3D map from: %s", this->mapFile_.c_str());
                    
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

                    RCLCPP_INFO(this->get_logger(), "Computing 3D SDF...");
                    compute3DEDT(distFieldOut);
                    compute3DEDT(distFieldIn);

                    distField3D_.resize(total_size);
                    for (size_t i = 0; i < total_size; ++i) {
                        float d_out = std::sqrt(distFieldOut[i]);
                        float d_in  = std::sqrt(distFieldIn[i]);
                        distField3D_[i] = (d_out - d_in) * mapResolution_;
                    }

                    RCLCPP_INFO(this->get_logger(), "3D Distance Field successfully created.");
                    
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
                totalLikelihood_ = 0.0;
                std::double_t maxLikelihood = 0.0;

                std::vector<std::vector<double>> likelihood_table;
                likelihood_table.reserve(particleNum_);
                
                for (std::size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t likelihood = 0.0;
                    
                    if (measurementModel_ == MeasurementModel::LikelihoodFieldModel) {
                        likelihood_table.push_back(std::move(caculateLikelihoodFieldModel(particles_[i].getPose(), local_points_)));
                    }
                    if (i == 0) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = 0;
                    } else if (maxLikelihood < likelihood) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = i;
                    }
                }

                std::vector<double> log_weights(particles_.size(),0.0);
                double max_log_weight = -std::numeric_limits<double>::infinity();

                for (std::size_t i = 0; i < likelihood_table.size(); i++) {
                    for (std::size_t k = 0; k < likelihood_table[i].size(); k++) {
                        log_weights[i] += std::log(likelihood_table[i][k]); 
                    }
                    
                    if (log_weights[i] > max_log_weight) {
                        max_log_weight = log_weights[i];
                    }

                    std::double_t w_sum = 0.0;
                    std::vector<double> linear_weights(particles_.size(), 0.0);

                    for (std::size_t i = 0; i < particles_.size(); i++) {
                        linear_weights[i] = std::exp(log_weights[i] - max_log_weight);
                        w_sum += linear_weights[i];
                    }

                    std::double_t w_sq_sum = 0.0;
                    for (std::size_t i = 0; i < particles_.size(); i++) {
                        double normalized_w = linear_weights[i] / w_sum;
                        particles_[i].setW(normalized_w);
                        w_sq_sum += normalized_w * normalized_w;
                    }

                    effectiveSampleSize_ = 1.0 / w_sq_sum;
                }
            }

            std::vector<double> caculateLikelihoodFieldModel(const geometry_msgs::msg::Pose& pose, const std::vector<Point3D>& local_points) {
                std::vector<double> p_vector;
                p_vector.reserve(local_points.size());

                double var = lfmSigma_ * lfmSigma_;
                double normConst = 1.0 / (std::sqrt(2.0 * M_PI * var));
                
                double scan_range_max = 30.0; 
                double pRand = 1.0 / scan_range_max * mapResolution_;

                for (const auto& pt : local_points) {
                    double map_x, map_y,map_z;
                    map_x = pt.x + pose.position.x;
                    map_y = pt.y + pose.position.y;
                    map_z = pt.z + pose.position.z; 

                    int u, v, w;
                    xyz2uvw(map_x, map_y, map_z, &u, &v, &w);

                    if (u >= 0 && u < static_cast<int>(dim_x_) && v >= 0 && v < static_cast<int>(dim_y_) && w >= 0 && w < static_cast<int>(dim_z_)) {
                        double sdf_val = static_cast<double>(distField3D_[getIdx3D(u, v, w)]);
                        
                        double d = 0.0;
                        if (sdf_val >= 0.0) {
                            d = sdf_val;
                        } else {
                            double penetration_penalty = 1.0; 
                            d = std::abs(sdf_val) + penetration_penalty;
                        }

                        double pHit = normConst * std::exp(-(d * d) / (2.0 * var)) * mapResolution_;
                        double p = zHit_ * pHit + zRand_ * pRand;
                        p_vector.push_back(std::min(p, 1.0)); 
                    } else {
                        p_vector.push_back(zRand_ * pRand);
                    }
                }

                return p_vector;
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
                int v_relative = static_cast<int>((y - mapOrigin_[1]) / mapResolution_);
                *v = mapHeight_ - 1 - v_relative;
                *w = static_cast<int>((z - mapOrigin_[2]) / mapResolution_);
            }

            // パブリッシャ / サブスクライバ / タイマー
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_cloud_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubPose_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particleMarker_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;

            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;

            // 状態変数
            geometry_msgs::msg::Pose mclPose_;
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            nav_msgs::msg::Path path_;
            std::vector<Point3D> local_points_;
            std::mt19937 gen_; // 乱数ジェネレータ
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            

            // 各種パラメータ
            std::double_t zHit_, zShort_, zMax_, zRand_;
            std::double_t lfmSigma_;
            double odomNoise1_, odomNoise2_, odomNoise3_, odomNoise4_;
            double resampleThreshold_;
            bool is_sim_ = false; // シミュレータフラグ
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL_3D>());
    rclcpp::shutdown();
    return 0;
}