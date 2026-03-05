#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <cmath>
#include <cstdlib>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <H5Cpp.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nhk2026_msgs/msg/multi_laser_scan.hpp"

using namespace std::chrono_literals; 
using namespace H5; // HDF5 namespace

struct Point2D {
  double x;
  double y;
};

struct SensorTransform {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool valid = false; // TFが取得できたかどうかのフラグ
};

namespace mcl {
    class Particle {
        public:
            const std::double_t& getX() const& { return pose_.x; }
            const std::double_t& getY() const& { return pose_.y; }
            const std::double_t& getTheta() const& { return pose_.theta; }
            const geometry_msgs::msg::Pose2D& getPose() const& { return pose_; }
            const std::double_t& getW() const& { return w_; }
            void setPose(std::double_t x, std::double_t y, std::double_t theta) {
                pose_.set__x(x);
                pose_.set__y(y);
                pose_.set__theta(theta);
            }
            void setW(std::double_t w) {
                w_ = w;
            }
        private:
            geometry_msgs::msg::Pose2D pose_;
            std::double_t w_;
    };

    // TODO: iterを実装してforで回してあげたい
    class probability {
        public:
            float p[1000];
            int num_scan;
    };

    enum class MeasurementModel { LikelihoodFieldModel };

    class MCL: public rclcpp::Node {
        public:
            explicit MCL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("mcl_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
                this->declare_parameter<std::int32_t>("particleNum", 100);
                this->declare_parameter<std::float_t>("initial_x", -1.0);
                this->declare_parameter<std::float_t>("initial_y", 1.0);
                this->declare_parameter<std::float_t>("initial_theta", M_PI/2);
                this->declare_parameter<std::float_t>("resampleThreshold", 0.5);
                this->declare_parameter<std::float_t>("odomNoise1", 1.0);
                this->declare_parameter<std::float_t>("odomNoise2", 1.0);
                this->declare_parameter<std::float_t>("odomNoise3", 1.0);
                this->declare_parameter<std::float_t>("odomNoise4", 1.0);
                // this->declare_parameter<std::float_t>("mapResolution", 0.01); // readMap内で上書きするためコメントアウト
                this->declare_parameter<std::string>("mapFile", "src/nhk2026_localization/map/nhk2026_field.h5"); // HDF5ファイルのパス
                this->declare_parameter<std::int32_t>("scanStep", 50);
                this->declare_parameter<std::double_t>("lfmSigma", 0.03);
                this->declare_parameter<std::double_t>("zHit", 1.0);
                this->declare_parameter<std::double_t>("zMax", 0.0);
                this->declare_parameter<std::double_t>("zRand", 1.0);
                this->declare_parameter<double>("lidar_threshold", 1.0/5.0*M_PI);
                this->declare_parameter<int>("mapZIndex", 4); 
                this->declare_parameter("filter_threshold", 0.98);
                
                particleNum_ = this->get_parameter("particleNum").as_int();
                double initial_x = this->get_parameter("initial_x").as_double();
                double initial_y = this->get_parameter("initial_y").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();
                this->resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();
                this->odomNoise1_ = this->get_parameter("odomNoise1").as_double();
                this->odomNoise2_ = this->get_parameter("odomNoise2").as_double();
                this->odomNoise3_ = this->get_parameter("odomNoise3").as_double();
                this->odomNoise4_ = this->get_parameter("odomNoise4").as_double();
                // this->mapResolution_ = this->get_parameter("mapResolution").as_double();
                this->mapFile_ = this->get_parameter("mapFile").as_string();
                this->scanStep_ = this->get_parameter("scanStep").as_int();
                this->lfmSigma_ = this->get_parameter("lfmSigma").as_double();
                this->zHit_ = this->get_parameter("zHit").as_double();
                this->zMax_ = this->get_parameter("zMax").as_double();
                this->zRand_ = this->get_parameter("zRand").as_double();
                this->mapZIndex_ = this->get_parameter("mapZIndex").as_int();
                this->get_parameter("lidar_threshold", LIDAR_THTRSHOLD_);

                particles_.resize(particleNum_);
                pro_.resize(particleNum_);
                mapOrigin_.resize(3, 0.0);

                measurementLikelihoods_.resize(particleNum_);
            
                // init robot pos
                geometry_msgs::msg::Pose2D pose;
                pose.set__x(initial_x);
                pose.set__y(initial_y);
                pose.set__theta(initial_theta);
                // initalize mclPose
                setMCLPose(pose);
                velOdom_.set__x(initial_x);
                velOdom_.set__y(initial_y);
                velOdom_.set__theta(initial_theta);
                
                // initialize particle
                geometry_msgs::msg::Pose2D initialNoise;
                auto cloud_qos = rclcpp::SensorDataQoS();
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", cloud_qos);
                initialNoise.set__x(0.3); // var of x
                initialNoise.set__y(0.3); // var of y
                initialNoise.set__theta(M_PI/18.0); // var of theta
                resetParticlesDistribution(initialNoise);
                //printParticlesMakerOnRviz2();
                

                // set mesurementModel
                measurementModel_ = MeasurementModel::LikelihoodFieldModel;

                //  mapの読み込みパブリッシュ
                rclcpp::QoS map_qos(rclcpp::KeepLast(1));
                map_qos.transient_local().reliable();
                pubMapCloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("voxel_map_cloud", map_qos);
                MCL::readMap();
                
                last_timestamp_ = this->get_clock()->now();
                // setup subscriper
                rclcpp::QoS cmdVelQos(rclcpp::KeepLast(10));
                subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel_feedback", cmdVelQos, std::bind(&MCL::cmdVelCallback, this, std::placeholders::_1)
                );

               
                auto laserScanQos = rclcpp::SensorDataQoS();
                const char *lidar = std::getenv("WITH_lidar");
                const char *sim = std::getenv("WITH_SIM");
                //0:LD 1:hokuyo1 2:hokuyou2 lidar_select
                // 0: LD-Lidar の場合
                // if (!lidar || std::string(lidar) == "0") {
                //     lidar_select = 0;
                //     subScanFront_ = create_subscription<sensor_msgs::msg::LaserScan>("/ldlidar_node/scan", laserScanQos,[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {this->laserScanCallback(msg, 0);});
                // }else if(std::string(lidar) == "1"){
                //     lidar_select = 1;
                //     subScanFront_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan_front", laserScanQos,[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {this->laserScanCallback(msg, 1);});
                // } else if(std::string(lidar) == "2"){
                //     lidar_select = 2;
                //     // Front用
                //     subScanFront_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan_front", laserScanQos,[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {this->laserScanCallback(msg, 1);});
                //     // Back用
                //     subScanBack_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan_back", laserScanQos,[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {this->laserScanCallback(msg, 2);});
                // }else if(std::string(lidar) == "3"){
                //     lidar_select = 3;
                //     subScanBack_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan_back", laserScanQos,[this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {this->laserScanCallback(msg, 2);});
                // }

                subScan_ = create_subscription<nhk2026_msgs::msg::MultiLaserScan>("/multi_scan",laserScanQos,std::bind(&MCL::laserScanCallback,this,std::placeholders::_1));
                
                // rclcpp::QoS callbackQos(rclcpp::KeepLast(10));
                // subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
                //     "/odom", callbackQos, std::bind(&MCL::odomCallback, this, std::placeholders::_1)
                // );

                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

                pubPath_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
                path_.header.frame_id = "map";
                pubPose_ = create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);
                
                rclcpp::QoS marker_qos(1); 
                marker_qos.transient_local(); 
                marker_qos.reliable();        
                vel_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity_marker", 10);

                origin_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("origin_marker", marker_qos);
                // s_odom_ = create_subscription<nav_msgs::msg::Odometry>(
                //     "/odom", tmp_qos, std::bind(&MCL::odomCallback, this, std::placeholders::_1)
                // );

                  
                scan_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);

                
                // RCLCPP_INFO(this->get_logger(), "freofkprekfore");
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

                // setup publisher
                iter_=0;
                timer_ = rclcpp::create_timer(
                    this,
                    this->get_clock(),
                    25ms,
                    std::bind(&MCL::loop, this)
                );
                publishOriginMarker();
                RCLCPP_INFO(this->get_logger(), "Success initialize");
                RCLCPP_INFO(this->get_logger(), "initial x:%f,y:%f",initial_x,initial_y);

              
            } 

        private:
            void setMCLPose(geometry_msgs::msg::Pose2D pose) { mclPose_=pose; }
            std::double_t randNormal(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }
            
            // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            //     auto &q = msg->pose.pose.orientation;
            //     // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            //     double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            //     double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            //     double yaw = std::atan2(siny_cosp, cosy_cosp);
                
            //     // if (!cmdVel_) {
            //     //     return;
            //     // }
            //     yaw_ = yaw;
            //     odom_twist_ = msg->twist.twist.angular.z;
            //     // RCLCPP_INFO(this->get_logger(), "%.3f", yaw_);
                // RCLCPP_INFO(this->get_logger(), "Yaw (rad): %.3f %.3f %.3f", yaw, msg->twist.twist.angular.z, particles_[0].getTheta());
            // }

            // void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg, int lidar_id) {
            //     if (lidar_id == 0) {
            //         // LD-Lidar: そのまま保存
            //         scanFront_ = msg;
            //     } 
            //     else if (lidar_id == 1) {
            //         // Front Lidar
            //         filterScan(msg, scanFront_, -M_PI / 2.6, M_PI / 2.3);
            //     } 
            //     else if (lidar_id == 2) {
            //         // Back Lidar
            //         filterScan(msg, scanBack_, -M_PI * 3/ 4, -M_PI / 6);
            //     }
            // }
            void laserScanCallback (const nhk2026_msgs::msg::MultiLaserScan::SharedPtr msg){
                scan_ = msg;
            }

          
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVel_=msg;
            }
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_=msg;
            }
            
            
            void readMap() {
                try {
                    RCLCPP_INFO(this->get_logger(), "Loading map from: %s", this->mapFile_.c_str());
                    
                    
                    H5File file(this->mapFile_, H5F_ACC_RDONLY);
                    DataSet dataset = file.openDataSet("map_data");

                    
                    DataSpace dataspace = dataset.getSpace();
                    int rank = dataspace.getSimpleExtentNdims();
                    std::vector<hsize_t> dims(rank);
                    dataspace.getSimpleExtentDims(dims.data(), NULL);

                    hsize_t dim_x = dims[0];
                    hsize_t dim_y = dims[1];
                    hsize_t dim_z = dims[2];

                    mapWidth_ = dim_x;
                    mapHeight_ = dim_y;

                    RCLCPP_INFO(this->get_logger(), "Map Shape: (%llu, %llu, %llu)", dim_x, dim_y, dim_z);

                    if (this->mapZIndex_ >= (int)dim_z) {
                        RCLCPP_ERROR(this->get_logger(), "Z index %d is out of bounds (Max: %llu)", this->mapZIndex_, dim_z - 1);
                        return;
                    }

                    
                    float voxel_size = 0.01f;

                    try {
                        
                        if (mapResolution_ <= 0.0) mapResolution_ = 0.01;

                        if (dataset.attrExists("origin")) {
                            Attribute attr = dataset.openAttribute("origin");
                            
                            float origin_buf[3];
                            
                            attr.read(PredType::NATIVE_FLOAT, origin_buf);

                            mapOrigin_ = {(double)origin_buf[0], (double)origin_buf[1], (double)origin_buf[2]};
                            
                        
                            RCLCPP_INFO(this->get_logger(), "origin x:%f, y:%f, z:%f", mapOrigin_[0], mapOrigin_[1], mapOrigin_[2]);
                        } else {
                            RCLCPP_INFO(this->get_logger(), "origin attribute not found. Using calculated defaults.");
                            
                            
                            double u_target = 0; 
                            double v_target = 0;
                            double pixels_from_bottom = (double)(mapHeight_ - 1) - v_target;
                            
                           
                            // mapOrigin_ = {-0.32, -0.02, 0.0};
                        }
                    } catch (H5::Exception& e) {
                        
                        e.printErrorStack();
                        RCLCPP_WARN(this->get_logger(), "HDF5 Error in reading attributes. Using defaults.");
                        mapResolution_ = 0.01;
                    } catch (std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "Standard Exception: %s", e.what());
                    } catch(...) {
                        RCLCPP_WARN(this->get_logger(), "Unknown Error in reading attributes. Using defaults.");
                        mapResolution_ = 0.01;
                    }


                    
                    size_t total_size = dim_x * dim_y * dim_z;
                    std::vector<uint8_t> map_data(total_size);
                    dataset.read(map_data.data(), PredType::NATIVE_UINT8);

                    // OpenCV画像 (2値マップ) の作成 & 距離場計算
                    // OpenCV Mat: (rows=height=y, cols=width=x)
                    cv::Mat binary_img(dim_y, dim_x, CV_8UC1);
                    cv::Mat raw_slice_img(dim_y, dim_x, CV_8UC1);

                  
                    for (int y = 0; y < (int)dim_y; ++y) {
                        for (int x = 0; x < (int)dim_x; ++x) {
                            unsigned long idx = x * (dim_y * dim_z) + y * dim_z + this->mapZIndex_;
                            uint8_t val = map_data[idx];

                            
                            int v_img = (int)dim_y - 1 - y; 

                            if (val == 1) {
                                binary_img.at<uint8_t>(v_img, x) = 0;   // 障害物
                            } else {
                                binary_img.at<uint8_t>(v_img, x) = 255; // 自由空間
                            }
                        }
                    }
                    
                    cv::Mat debug_binary_color;
                    cv::cvtColor(binary_img, debug_binary_color, cv::COLOR_GRAY2BGR);

                    
                    std::int32_t u_origin, v_origin;
                    xy2uv(0.0, 0.0, &u_origin, &v_origin);

                    
                    if (u_origin >= 0 && u_origin < (int)dim_x && v_origin >= 0 && v_origin < (int)dim_y) {
                        
                        cv::circle(debug_binary_color, cv::Point(u_origin, v_origin), 5, cv::Scalar(0, 255, 0), -1);
                        RCLCPP_INFO(this->get_logger(), "Physical origin (0,0) is at image pixel: u=%d, v=%d", u_origin, v_origin);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Physical origin (0,0) is outside the map boundaries!");
                    }

                    
                    std::int32_t u_init, v_init;
                    
                    xy2uv(0.25, 0.25, &u_init, &v_init); 

                    if (u_init >= 0 && u_init < binary_img.cols && v_init >= 0 && v_init < binary_img.rows) {
                
                        cv::circle(debug_binary_color, cv::Point(u_init, v_init), 5, cv::Scalar(255, 0, 0), -1);
                    }

                    drawMarkerOnImage(debug_binary_color, 0.32, 0.02, cv::Scalar(0, 0, 255));

                
                    cv::imwrite("debug_binary_map.png", debug_binary_color);
                    

        
                    cv::imwrite("debug_raw_slice.png", raw_slice_img);


                    RCLCPP_INFO(this->get_logger(), "Saved raw Z-slice image to debug_raw_slice.png");
                
                    mapImg_ = binary_img.clone();

                    // [MODIFIED] SDF (Signed Distance Field) の導入
                    cv::Mat distFieldOut;
                    cv::distanceTransform(binary_img, distFieldOut, cv::DIST_L2, 5);

                    cv::Mat inverted_binary_img;
                    cv::bitwise_not(binary_img, inverted_binary_img);

                    cv::Mat distFieldIn;
                    cv::distanceTransform(inverted_binary_img, distFieldIn, cv::DIST_L2, 5);

                    
                    distField_ = cv::Mat(dim_y, dim_x, CV_64FC1); // メモリ確保
                    for (int y = 0; y < (int)dim_y; ++y) {
                        for (int x = 0; x < (int)dim_x; ++x) {
                            float d_out = distFieldOut.at<float>(y, x);
                            float d_in  = distFieldIn.at<float>(y, x);
                            // 障害物の外ならプラス、中ならマイナスになるようにする
                            double sdf_pixel = d_out - d_in;
                            distField_.at<double>(y, x) = sdf_pixel * mapResolution_;
                        }
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Distance Field created. (11, 50) = %lf m", distField_.at<double>(11, 50));

                
                double minVal, maxVal;
                cv::minMaxLoc(distField_, &minVal, &maxVal);
                // 0除算回避のため小さな値を足しておく
                double maxAbsVal = std::max(std::abs(minVal), std::abs(maxVal)) + 1e-6;

                // 2. カラー画像用のMatを作成 (BGR順)
                cv::Mat sdf_color_img(dim_y, dim_x, CV_8UC3);

                for (int y = 0; y < (int)dim_y; ++y) {
                    for (int x = 0; x < (int)dim_x; ++x) {
                        double d = distField_.at<double>(y, x);
                        
                        // 境界からの距離の絶対値を 0.0〜1.0 に正規化
                        double ratio = std::abs(d) / maxAbsVal;
                        // 色の濃さを計算 (0〜255)。境界(d=0)付近は薄く(255)、遠ざかるほど濃く(0)する
                        int intensity = 255 - static_cast<int>(ratio * 255.0);
                        // 安全のため範囲制限
                        intensity = std::max(0, std::min(255, intensity));

                        cv::Vec3b color;
                        if (d > 1e-9) { // 壁の外側 (正の値)
                            // 白 -> 青 のグラデーション (RとGを減らしていく)
                            color = cv::Vec3b(255, intensity, intensity);
                        } else if (d < -1e-9) { // 壁の内側 (負の値)
                            // 白 -> 赤 のグラデーション (BとGを減らしていく)
                            color = cv::Vec3b(intensity, intensity, 255);
                        } else { // 境界付近 (ほぼ0)
                            // 真っ白
                            color = cv::Vec3b(255, 255, 255);
                        }
                        sdf_color_img.at<cv::Vec3b>(y, x) = color;
                    }
                }

                // デバッグ用の点描画（既存コードの維持）
                if (11 < dim_y && 50 < dim_x) {
                    // OpenCVのPointは(x, y)の順
                    cv::circle(sdf_color_img, cv::Point(50, 11), 3, cv::Scalar(0, 255, 0), -1); 
                }

                // 画像を保存
                cv::imwrite("sdf_color_debug.png", sdf_color_img);
                RCLCPP_INFO(this->get_logger(), "Saved colored SDF image to sdf_color_debug.png");


                publishVoxelMap(map_data, dim_x, dim_y, dim_z);

                } catch (FileIException& error) {
                    error.printErrorStack();
                    RCLCPP_ERROR(this->get_logger(), "HDF5 File Error");
                } catch (DataSetIException& error) {
                    error.printErrorStack();
                    RCLCPP_ERROR(this->get_logger(), "HDF5 DataSet Error");
                } catch (DataSpaceIException& error) {
                    error.printErrorStack();
                    RCLCPP_ERROR(this->get_logger(), "HDF5 DataSpace Error");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error in readMap: %s", e.what());
                }
            }
            void drawMarkerOnImage(cv::Mat& img, double x, double y, cv::Scalar color) {
                std::int32_t u, v;
                xy2uv(x, y, &u, &v);

                
                if (u >= 0 && u < img.cols && v >= 0 && v < img.rows) {
                    cv::circle(img, cv::Point(u, v), 1, color, -1);
                }
            }

            void plotPoseOnMap() {
                if (mapImg_.empty()) return;

                // 1. マップ画像をカラーに変換してコピーを作成
                cv::Mat debug_pose_img;
                cv::cvtColor(mapImg_, debug_pose_img, cv::COLOR_GRAY2BGR);

                // 2. 現在の推定位置を画像座標に変換
                std::int32_t u, v;
                xy2uv(mclPose_.x, mclPose_.y, &u, &v);

                // 3. 画像の範囲内かチェックして描画
                if (u >= 0 && u < debug_pose_img.cols && v >= 0 && v < debug_pose_img.rows) {
                    // ロボットの位置を赤色の円で描画 (BGR: 0, 0, 255)
                    cv::circle(debug_pose_img, cv::Point(u, v), 5, cv::Scalar(0, 0, 255), -1);

                    // ロボットの向き (theta) を短い線で描画
                    double line_len = 20.0; // 線の長さ (ピクセル)
                    int u_end = u + static_cast<int>(line_len * std::cos(mclPose_.theta));
                    // 画像座標系では v の向きが物理座標の theta と逆転するため、サインにマイナスをかける
                    // (修正後の readMap を使っている場合、上方向が y 正なので、画像座標 v は上ほど小さい)
                    int v_end = v - static_cast<int>(line_len * std::sin(mclPose_.theta));
                    
                    cv::line(debug_pose_img, cv::Point(u, v), cv::Point(u_end, v_end), cv::Scalar(0, 0, 255), 2);
                }

                // 4. ファイルに保存
                cv::imwrite("debug_pose_on_map.png", debug_pose_img);
            }

            void publishVoxelMap(const std::vector<uint8_t>& map_data, hsize_t dim_x, hsize_t dim_y, hsize_t dim_z) {
                // インデックスの範囲チェック
                if (this->mapZIndex_ < 0 || this->mapZIndex_ >= (int)dim_z) {
                    RCLCPP_ERROR(this->get_logger(), "mapZIndex %d is out of bounds (0 to %llu)", this->mapZIndex_, dim_z - 1);
                    return;
                }

                int target_z = this->mapZIndex_;
  // 実距離(メートル)へ変換して格納 (CV_64F)
                // 1. 指定されたZ面にある障害物(値が0以外)の数をカウントする
                size_t num_points = 0;
                
                // データ配列のインデックス計算: x * (dim_y * dim_z) + y * dim_z + z
                // ここでは z を target_z に固定して x, y だけ回します
                for (hsize_t x = 0; x < dim_x; ++x) {
                    for (hsize_t y = 0; y < dim_y; ++y) {
                        unsigned long idx = x * (dim_y * dim_z) + y * dim_z + target_z;
                        
                        // 範囲外アクセス防止（念のため）
                        if (idx >= map_data.size()) continue;

                        if (map_data[idx] != 0) { 
                            num_points++;
                        }
                    }
                }

                if (num_points == 0) {
                    RCLCPP_WARN(this->get_logger(), "No occupied voxels found at Z index %d.", target_z);
                    return;
                }

                // 2. PointCloud2 メッセージの作成
                sensor_msgs::msg::PointCloud2 cloud_msg;
                cloud_msg.header.stamp = this->now();
                cloud_msg.header.frame_id = "map";
                cloud_msg.height = 1;
                cloud_msg.width = num_points;
                cloud_msg.is_dense = false;
                cloud_msg.is_bigendian = false;

                sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
                modifier.setPointCloud2FieldsByString(1, "xyz");
                modifier.resize(num_points);

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

                // 3. データを走査して座標を埋める
                for (hsize_t x = 0; x < dim_x; ++x) {
                    for (hsize_t y = 0; y < dim_y; ++y) {
                        unsigned long idx = x * (dim_y * dim_z) + y * dim_z + target_z;

                        if (idx >= map_data.size()) continue;

                        if (map_data[idx] != 0) {
                            // 物理座標への変換
                            // 修正後：可視化用クラウドも原点を反映させる
                            float phys_x = (float)x * mapResolution_ + mapOrigin_[0];
                            float phys_y = (float)y * mapResolution_ + mapOrigin_[1];
                            float phys_z = 1 * mapResolution_; // Z高さは固定

                            *iter_x = phys_x;
                            *iter_y = phys_y;
                            *iter_z = phys_z;

                            ++iter_x;
                            ++iter_y;
                            ++iter_z;
                        }
                    }
                }

                pubMapCloud_->publish(cloud_msg);
                RCLCPP_INFO(this->get_logger(), "Published Z-slice (z=%d, height=%.3fm) map with %zu points", target_z, (float)target_z * mapResolution_, num_points);
            }

            void loop() {
                rclcpp::Time current_time = this->get_clock()->now();
                // 並進速度・回転速度を取得
                if (!cmdVel_) {
                    return;
                }

                if (!scan_ || scan_->scans.empty()) {
                    return; 
                }


                if (last_timestamp_.nanoseconds() == 0) {
                    last_timestamp_ = current_time;
                    return;
                }

                //updateSensorTransforms();

                // double dt = (current_time - last_timestamp_).seconds();
                // if (dt <= 0.0) dt = 0.025; // 例外処理としてのフォールバック
                double dt = 0.025;
                last_timestamp_ = current_time;

                // rclcpp::Time t_front = scanFront_->header.stamp; // 1. Front LiDARの発行時刻
                // double delay_front = (current_time - t_front).seconds(); // 遅延時間

                // if (lidar_select == 2) {
                //     // Dual LiDARの場合
                //     rclcpp::Time t_back = scanBack_->header.stamp; // 2. Back LiDARの発行時刻
                //     double delay_back = (current_time - t_back).seconds();

                //     // 秒単位で表示（読みやすいように下4桁などを表示するか、そのまま秒数を出す）
                //     RCLCPP_INFO(this->get_logger(), 
                //         "\n[Times] Est: %.4f\n        Front: %.4f (Delay: %.4fs)\n        Back : %.4f (Delay: %.4fs)",
                //         current_time.seconds(),
                //         t_front.seconds(), delay_front,
                //         t_back.seconds(), delay_back
                //     );
                // } else {
                //     // Single LiDARの場合
                //     RCLCPP_INFO(this->get_logger(), 
                //         "\n[Times] Est: %.4f\n        Front: %.4f (Delay: %.4fs)",
                //         current_time.seconds(),
                //         t_front.seconds(), delay_front
                //     );
                // }
                            
                
                // ロボットから見た座標系
                std::double_t vx_ = cmdVel_->linear.x;
                std::double_t vy_ = cmdVel_->linear.y;
                std::double_t omega_ = cmdVel_->angular.z;
                
                //RCLCPP_INFO(this->get_logger(), "cmd_vel -> vx: %.3f, vy: %.3f", vx_, vy_);
                geometry_msgs::msg::Twist delta_;
                delta_.linear.x = vx_*dt;
                delta_.linear.y = vy_*dt;
                delta_.angular.z = omega_*dt;


                updateParticles(delta_);
                printParticlesMakerOnRviz2();
                caculateMeasurementModel(*scan_);
                estimatePose();
                resampleParticles();
                printTrajectoryOnRviz2();
                publishOriginMarker();
                //plotPoseOnMap();
                publishVelocityMarker();
                //RCLCPP_INFO(this->get_logger(), " Z-Index : %d", this->mapZIndex_);
                // publishScanEndpoints();
                // TODO: printTrajectory
            }

            void getRobotInitialPos() {
                // TODO: split into robot_manager
            }

            void resetParticlesDistribution(geometry_msgs::msg::Pose2D noise) {
                std::double_t wo = 1.0 / (std::double_t)particles_.size();
                for (std::size_t i=0; i<particles_.size(); i++ ) {
                    // TODO: フィールドの中に入っていない場合はリサンプリングする
                    std::double_t x = mclPose_.x + randNormal(noise.x);
                    std::double_t y = mclPose_.y + randNormal(noise.y);
                    // std::double_t x = mclPose_.x;
                    // std::double_t y = mclPose_.y;
                    std::double_t theta = mclPose_.theta + randNormal(noise.theta);
                    particles_[i].setPose(x, y, theta);
                    particles_[i].setW(wo);
                }
            }

            void updateParticles(geometry_msgs::msg::Twist delta) {
                std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y * delta.linear.y; 
                std::double_t dy2 = delta.angular.z * delta.angular.z;
                // std::double_t dd2 = 0;
                // std::double_t dy2 = 0;
                // RCLCPP_INFO(this->get_logger(), "odomNoise1=%lf", odomNoise1_);
                std::double_t sigma_xy = std::sqrt(odomNoise1_*dd2 + odomNoise2_*dy2);
                std::double_t sigma_theta = std::sqrt(odomNoise3_ * dd2 + odomNoise4_ * dy2);

                for (size_t i = 0; i < this->particles_.size(); i++ ) {
                    std::double_t dx = delta.linear.x + randNormal(sigma_xy);
                    std::double_t dy = delta.linear.y + randNormal(sigma_xy);
                    std::double_t dtheta = delta.angular.z + randNormal(sigma_theta);

                    geometry_msgs::msg::Pose2D pose_ = this->particles_[i].getPose();
                    std::double_t theta_ = pose_.theta;
                    std::double_t x_ = pose_.x + std::cos(theta_)*dx - std::sin(theta_)*dy;
                    std::double_t y_ = pose_.y + std::sin(theta_)*dx + std::cos(theta_)*dy;
                    theta_ += dtheta;
                    particles_[i].setPose(x_, y_, theta_);
                }
            }

            // void updateParticles(std::double_t deltaDist, std::double_t deltaTheta) {
            //     std::double_t dd2 = deltaDist * deltaDist;
            //     std::double_t dtheta2 = deltaTheta * deltaTheta;
            //     std::normal_distribution<std::double_t> distd_(0, odomNoise1_*dd2 + odomNoise2_*dtheta2);
            //     std::normal_distribution<std::double_t> distt_(0, odomNoise3_*dd2 + odomNoise4_*dtheta2);
            //     for (size_t particle_idx_=0; particle_idx_ < particles_.size(); particle_idx_++) {
            //         std::double_t dd = deltaDist + distd_(gen_);
            //         std::double_t dtheta = deltaTheta + distt_(gen_);
            //         std::double_t theta = particles_[particle_idx_].getTheta();
            //         std::double_t x = particles_[particle_idx_].getX() + dd*std::cos(theta);
            //         std::double_t y = particles_[particle_idx_].getY() + dd*std::sin(theta);
            //         theta += dtheta;
            //         particles_[particle_idx_].setPose(x, y, theta);
            //     }
            // }

            sensor_msgs::msg::PointCloud2 layerScan2PointCloud(sensor_msgs::msg::LaserScan scan) {
                // TODO : ノード分ける, 色変えたい
                // sensor_msgs::msg::LaserScan out = scan;
                // out.ranges.clear();
                // out.intensities.clear();
                // out.angle_increment = scan.angle_increment * std::float_t(this->scanStep_);

                // for (std::size_t i=0; i<scan.ranges.size(); i+=scanStep_){
                //     out.ranges.push_back(scan.ranges[i]);
                //     if (scan.intensities.empty()) {
                //         RCLCPP_ERROR(this->get_logger(), "index out of range.");
                //     }
                //     out.intensities.push_back(scan.intensities[i]);
                // }
                
                sensor_msgs::msg::PointCloud2 cloud_in, cloud_out;
                projector_.projectLaser(scan, cloud_in);

                // RCLCPP_INFO(this->get_logger(), "%s", scan.header.frame_id.c_str());

                geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
                    "ldlidar_base",
                    scan.header.frame_id,
                    tf2::TimePointZero  // 最新の transform を使う
                );
                tf2::doTransform(cloud_in, cloud_out, tf);

                return cloud_out;
            }

            void caculateMeasurementModel(nhk2026_msgs::msg::MultiLaserScan multi_scan) {
                totalLikelihood_ = 0.0;
                std::double_t maxLikelihood = 0.0;

                std::vector<std::vector<double>> likelihood_table;
                likelihood_table.reserve(particleNum_);
                
                for (std::size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t likelihood = 0.0;
                    // 尤度場モデル
                    if (measurementModel_ == MeasurementModel::LikelihoodFieldModel) {
                        likelihood_table.push_back(std::move(caculateLikelihoodFieldModel(particles_[i].getPose(), scan_)));
                    }
                    if (i == 0) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = 0;
                    } else if (maxLikelihood < likelihood) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = i;
                    }
                    // RCLCPP_INFO(this->get_logger(), "%lf", maxLikelihood);
                }
                // RCLCPP_INFO(this->get_logger(), "%lf", maxLikelihood);
                //昔の計算方法
                // std::double_t w_sum = 0;
                // for(std::size_t i=0; i<likelihood_table.size(); i++ ) {
                //     std::double_t w = 0;
                //     for (std::size_t j=0; j<likelihood_table.size(); j++ ) {
                //         std::double_t loglikefood_sum=0;
                //         for (std::size_t k=0; k<likelihood_table[i].size(); k++ ) {
                //             // if (std::isnan(likelihood_table[j][k]) || std::isnan(likelihood_table[i][k])) continue;
                //             // if (likelihood_table[j][k]<1e-12 || likelihood_table[i][k]<1e-12) continue;
                //             loglikefood_sum += std::log(likelihood_table[j][k]/likelihood_table[i][k]);
                //             // RCLCPP_INFO(this->get_logger(), "j=%d k=%d %.4f", j, k, likelihood_table[j][k]);
                //         }
                //         w += std::exp(loglikefood_sum);
                //     }
                //     w = 1/w;
                //     particles_[i].setW(w);
                //     w_sum += w*w;
                // }
                // effectiveSampleSize_ = 1.0 / w_sum;

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

            std::vector<std::double_t> caculateLikelihoodFieldModel (geometry_msgs::msg::Pose2D pose, const nhk2026_msgs::msg::MultiLaserScan::SharedPtr multi_scan) {

                //
                scan_endpoints_.clear();
                //

                std::double_t var = lfmSigma_*lfmSigma_;
                std::double_t normConst = 1.0 / (sqrt(2.0*M_PI*var));
                std::vector<double> p_vector;

                std::double_t pMax = 1.0 / mapResolution_; // <- mapResolution_で割る必要なくない？
                std::double_t w = 0.0;

                // sensor_msgs::msg::PointCloud2 pointCloud = layerScan2PointCloud(scan);
                // sensor_msgs::PointCloud2ConstIterator<float> it_x(pointCloud, "x");
                // sensor_msgs::PointCloud2ConstIterator<float> it_y(pointCloud, "y");
                // sensor_msgs::PointCloud2ConstIterator<float> it_z(pointCloud, "z");

                for (const auto& scan : multi_scan->scans) {
                    std::double_t pRand = 1.0 / scan.range_max * mapResolution_;
                    std::string frame_id = scan.header.frame_id;
                    // ★追加: frame_idからどのLiDARのデータかを判定 (0:LD, 1:Front, 2:Back)
                    RCLCPP_INFO(this->get_logger(), "Received frame_id: '%s'", frame_id.c_str());
                    int current_lidar_pose = 1; // デフォルトはFront
                    if (scan.header.frame_id.find("lidar_back") != std::string::npos) {
                        current_lidar_pose = 2;
                    } else if (scan.header.frame_id.find("ldlidar") != std::string::npos) {
                        current_lidar_pose = 0;
                    }

                    for (std::size_t i = 0; i < scan.ranges.size(); i += scanStep_) {
                        std::double_t r = scan.ranges[i];
                        
                        // ※修正: 無効なデータの場合はここでcontinueして無駄な計算を省く
                        if (std::isnan(r)  || r < scan.range_min || scan.range_max < r) {
                            p_vector.push_back(zRand_*pRand); 
                        }

                        std::double_t theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment;
                        
                        // LD-Lidarの場合の角度補正
                        if (current_lidar_pose == 0 && !is_sim_) {
                            theta_lidar -= 3.0*M_PI/2.0;
                        }

                        double x_odom, y_odom;
                        int u, v;
                        // 判定した current_lidar_pose を渡す
                        lidarpose2uv(r, theta_lidar, pose, &x_odom, &y_odom, &u, &v, current_lidar_pose);

                        if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_) {
                            std::double_t sdf_val = (std::double_t)distField_.at<std::double_t>(v, u);
                            std::double_t d = 0.0;
                            if (sdf_val >= 0) {
                                d = sdf_val;
                            } else {
                                std::double_t penetration_penalty = 1; 
                                d = std::abs(sdf_val) + penetration_penalty;
                            }
                            
                            std::double_t pHit = normConst * exp(-(d*d)/(2.0*var))*mapResolution_;
                            std::double_t p = zHit_*pHit + zRand_*pRand;

                            if (p > 1.0) p = 1.0;
                            p_vector.push_back(p);
                        } else {
                            p_vector.push_back(zRand_*pRand);
                        }
                    }
                }
                return p_vector;


               
              
            }

            void lidarpose2uv(double range, double theta, geometry_msgs::msg::Pose2D pose, double *x_odom, double *y_odom, int *u, int *v,int lidar_pose) {
                std::double_t x_lidar;
                std::double_t y_lidar;
                if (lidar_pose == 0) {
                    x_lidar = range*cos(theta) + 0.084;
                    y_lidar = range*sin(theta) + 0.013 - 0.013;
                } else if(lidar_pose == 1){
                    x_lidar = range * cos(-1 * theta + M_PI/2) - 0.094036;
                    y_lidar = range * sin(-1 * theta + M_PI/2) + 0.2255;
                    //RCLCPP_INFO(this->get_logger(), "fpeafreafkoera");
                }else if(lidar_pose == 2){
                    
                    //x_lidar = range * cos(theta );
                    //y_lidar = range * sin(theta );
                    x_lidar = range * cos(theta ) + 0.2084;
                    y_lidar = range * sin(theta ) - 0.2999;
                    //RCLCPP_INFO(this->get_logger(), "fpeafreafkoera");
                }

                
                std::double_t x = x_lidar*cos(pose.theta) - y_lidar*sin(pose.theta) + pose.x;
                std::double_t y = x_lidar*sin(pose.theta) + y_lidar*cos(pose.theta) + pose.y;

                *x_odom = x;
                *y_odom = y;

                xy2uv(x, y, u, v);
            }

            // void lidarpose2uv(double range, double theta, geometry_msgs::msg::Pose2D pose, double *x_odom, double *y_odom, int *u, int *v, int lidar_pose) {
    
                
            //     if (lidar_pose < 0 || lidar_pose >= (int)lidar_transforms_.size()) return;

              
            //     double sensor_x   = lidar_transforms_[lidar_pose].x;
            //     double sensor_y   = lidar_transforms_[lidar_pose].y;
            //     double sensor_yaw = lidar_transforms_[lidar_pose].yaw;

                
            //     // x_lidar = 距離 * cos(レーザー角度 + センサの取り付け角度) + センサの取り付け位置X
            //     std::double_t x_lidar = range * cos(theta + sensor_yaw) + sensor_x;
            //     std::double_t y_lidar = range * sin(theta + sensor_yaw) + sensor_y;

                
            //     std::double_t x = x_lidar * cos(pose.theta) - y_lidar * sin(pose.theta) + pose.x;
            //     std::double_t y = x_lidar * sin(pose.theta) + y_lidar * cos(pose.theta) + pose.y;

            //     *x_odom = x;
            //     *y_odom = y;

            //     xy2uv(x, y, u, v);
            // }

            void publishScanEndpoints()
            {
                sensor_msgs::msg::PointCloud2 cloud;
                cloud.header.stamp    = this->get_clock()->now();
                cloud.header.frame_id = "map";
                cloud.height          = 1;
                cloud.width           = scan_endpoints_.size();
                cloud.is_dense        = false;
                cloud.is_bigendian    = false;

                // xyz フィールドだけを使う
                sensor_msgs::PointCloud2Modifier mod(cloud);
                mod.setPointCloud2FieldsByString(1, "xyz");
                mod.resize(scan_endpoints_.size());

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

                for (const auto &pt : scan_endpoints_) {
                    *iter_x = pt.x;
                    *iter_y = pt.y;
                    *iter_z = pt.z;  // 0 で OK
                    ++iter_x; ++iter_y; ++iter_z;
                }

                particleMarker_->publish(cloud);
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

                // 矢印の付け根をロボットの現在位置に設定
                marker.pose.position.x = mclPose_.x;
                marker.pose.position.y = mclPose_.y;
                marker.pose.position.z = 0.1; // 少し浮かせる

                // 速度ベクトルの向きを計算
                // ロボットの向き(theta) + 速度成分(vx, vy)による角度
                double vel_angle = std::atan2(cmdVel_->linear.y, cmdVel_->linear.x);
                double speed = std::sqrt(cmdVel_->linear.x * cmdVel_->linear.x + 
                                        cmdVel_->linear.y * cmdVel_->linear.y);

                tf2::Quaternion q;
                q.setRPY(0, 0, mclPose_.theta + vel_angle);
                marker.pose.orientation = tf2::toMsg(q);

                // 矢印の長さ(x)を速度に比例させる
                marker.scale.x = speed * 0.5 + 0.05; // 最小サイズを少し確保
                marker.scale.y = 0.05; // 矢印の太さ
                marker.scale.z = 0.05; // 矢印の高さ

                // 色（黄色などが見やすい）
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                vel_marker_pub_->publish(marker);
            }

            void estimatePose() {
                std::double_t tmpTheta = mclPose_.theta;
                std::double_t x = 0.0, y = 0.0, theta = 0.0;
                for (size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t w = particles_[i].getW();
                    x += particles_[i].getX() * w;
                    y += particles_[i].getY() * w;
                    std::double_t dTheta = tmpTheta - particles_[i].getTheta();
                    // RCLCPP_INFO(this->get_logger(), "%.4f %.4f %.4f", w, particles_[i].getX(), particles_[i].getY());
                    while (dTheta < -M_PI) dTheta += 2.0*M_PI;
                    while (dTheta > M_PI) dTheta -= 2.0*M_PI;
                    theta += dTheta * w;
                }
                theta = tmpTheta - theta;
                mclPose_.set__x(x);
                mclPose_.set__y(y);
                mclPose_.set__theta(theta);
                pubPose_->publish(mclPose_);

                // TODO: publish odom
                if (!is_sim_) {
                    geometry_msgs::msg::TransformStamped tf_msg;
                    tf_msg.header.stamp = this->get_clock()->now();
                    tf_msg.header.frame_id = "odom";
                    tf_msg.child_frame_id = "base_footprint";

                    tf_msg.transform.translation.x = x;
                    tf_msg.transform.translation.y = y;
                    tf_msg.transform.translation.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, theta );
                    tf_msg.transform.rotation = tf2::toMsg(q);

                    tf_broadcaster_->sendTransform(tf_msg);
                }

                // RCLCPP_INFO(this->get_logger(), "%.4f %.4f %.4f", x, y, theta);
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
                            geometry_msgs::msg::Pose2D tmpPos = tmpParticles[j].getPose();
                            particles_[i].setPose(tmpPos.x, tmpPos.y, tmpPos.theta);
                            particles_[i].setW(wo);
                            break;
                        }
                    }
                }
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
                    *iter_z = 0;

                    *iter_r = 0;
                    *iter_g = 0;
                    *iter_b = int(p.getW()*255);

                    ++iter_x, ++iter_y, ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                particleMarker_->publish(cloud_);
            }

            void printTrajectoryOnRviz2() {
                geometry_msgs::msg::PoseStamped stamped;
                stamped.header.stamp = this->now();
                stamped.header.frame_id = path_.header.frame_id;
                stamped.pose.position.x = mclPose_.x;
                stamped.pose.position.y = mclPose_.y;
                stamped.pose.position.z = 0.0;
                //RCLCPP_INFO(this->get_logger(), "%lf %lf", mclPose_.x, mclPose_.y);

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, mclPose_.theta);
                stamped.pose.orientation = tf2::toMsg(q);

                path_.poses.push_back(stamped);
                path_.header.stamp = stamped.header.stamp;

                pubPath_->publish(path_);
            }

            // void vel2OdomCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
            //     rclcpp::Time now = this->get_clock()->now();

            //     double dt = (now - last_timestamp_).seconds();
            //     if (dt <= 0.0) {
            //         return;
            //     }
            //     last_timestamp_ = now;
                
            //     std::double_t theta = velOdom_.theta;
            //     std::double_t dx = (msg->linear.x) * dt;
            //     std::double_t dy = (msg->linear.y) * dt;
            //     velOdom_.x += std::cos(theta)*dx - std::sin(theta)*dy;
            //     velOdom_.y += std::sin(theta)*dx + std::cos(theta)*dy;
            //     velOdom_.theta += msg->angular.z*dt;

            //     RCLCPP_INFO(this->get_logger(), "Yaw (rad): %.5f %.5f %.5f %.5f %.5f", yaw_, odom_twist_, msg->angular.z, velOdom_.theta, now.seconds());
            // }
            
            // TODO
            // グリッドマップ上の点へ変換する
            // void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {

            //     *u = (std::int32_t)(x / mapResolution_);
            //     *v = mapHeight_ - 1 - (std::int32_t)(y / mapResolution_);
            // }

            void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {
                // 物理座標からマップの左下端（原点）を引いてから解像度で割る
                *u = (std::int32_t)((x - mapOrigin_[0]) / mapResolution_);
                
                // Y軸は画像座標系では上ほど値が小さいため、逆転させる
                // mapOrigin_[1] はマップの最下部のY座標
                std::int32_t v_relative = (std::int32_t)((y - mapOrigin_[1]) / mapResolution_);
                *v = mapHeight_ - 1 - v_relative;
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

            int computeOutCode(int u, int v) {
                int code = 0;

                int win_u_min = 60;
                int win_u_max = 60 + 28 + 1;
                int win_v_min = 90;
                int win_v_max = 90 + 90;

                if (v > win_v_max) code |= 8;
                if (v < win_v_min) code |= 4;
                if (u > win_u_max) code |= 2;
                if (u < win_u_min) code |= 1;
                return code;
            }


            // mclクラスのprivate関数に追加
            void updateSensorTransforms() {
                auto get_tf = [&](std::string target, std::string source, int index) {
                    try {
                        // 最新のTFを取得
                        geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
                            target, source, tf2::TimePointZero);

                        lidar_transforms_[index].x = t.transform.translation.x;
                        lidar_transforms_[index].y = t.transform.translation.y;
                        
                        // クォータニオンからYaw角（回転）を取得
                        tf2::Quaternion q(
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w);
                        double roll, pitch, yaw;
                        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                        
                        lidar_transforms_[index].yaw = yaw;
                        lidar_transforms_[index].valid = true;
                    } catch (tf2::TransformException &ex) {
                        // TFがまだ来ていない場合などは警告を出してスキップ
                        // RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
                        lidar_transforms_[index].valid = false;
                    }
                };

                // LiDAR 0 (LD-LiDAR) - 例: ldlidar_link
                // get_tf("base_footprint", "ldlidar_link", 0); 

                // LiDAR 1 (Front)
                // get_tf("base_footprint", "lidar_front", 1);

                // LiDAR 2 (Back)
                // get_tf("base_footprint", "lidar_back", 2);
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

            void publishOriginMarker() {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map"; // map座標系
                marker.header.stamp = this->now();
                marker.ns = "origin";
                marker.id = 0;
                
                // 形状（球体：SPHERE, 立方体：CUBE, 点：POINTS など）
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // 座標 (原点)
                marker.pose.position.x = 1.0;
                marker.pose.position.y = 0.0;
                marker.pose.position.z = 0.0;
                marker.pose.orientation.w = 1.0;

                // サイズ (直径 0.1m = 10cm)
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;

                // 色 (緑色)
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f; // 透明度 (1.0で不透明)

                origin_marker_pub_->publish(marker);
            }

            // parameter for map
            std::string mapDir_;
            std::string mapFile_; // 追加
            int mapZIndex_;       // 追加
            std::double_t mapResolution_;
            std::int32_t mapWidth_, mapHeight_;
            std::vector<std::double_t> mapOrigin_;
            cv::Mat mapImg_;
            cv::Mat distField_;

            //pose

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr origin_marker_pub_;

            // particle settings
            int particleNum_;
            // 絶対座標
            std::vector<Particle> particles_;
            geometry_msgs::msg::Pose2D mclPose_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particleMarker_;

            // likelihood
            int maxLikelihoodParticleIdx_;
            std::double_t totalLikelihood_;
            std::double_t averageLikelihood_;
            std::vector<std::double_t> measurementLikelihoods_;
            std::vector<probability> pro_;

            std::double_t effectiveSampleSize_;
            std::double_t resampleThreshold_;

            // model for mesurement
            mcl::MeasurementModel measurementModel_;
            std::int32_t scanStep_;

            // parameter for measurement model
            std::double_t zHit_, zShort_, zMax_, zRand_;
            std::double_t lfmSigma_;

            // parameter for lidar relitive position
            // lidarの絶対座標はtfが変換してpublishしてくれている可能性があるので，もしかしたらいらないかも
            std::double_t x_lidar_, y_lidar_, theta_lidar_;

            // noize when updateing particle
            std::double_t odomNoise1_, odomNoise2_, odomNoise3_, odomNoise4_;
            std::mt19937 gen_;

            std::string base_footprint_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;

            sensor_msgs::msg::LaserScan::SharedPtr scanFront_;
            sensor_msgs::msg::LaserScan::SharedPtr scanBack_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLayerScan_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScanFront_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScanBack_;

            rclcpp::Subscription<nhk2026_msgs::msg::MultiLaserScan>::SharedPtr subScan_;
            nhk2026_msgs::msg::MultiLaserScan::SharedPtr scan_;

            bool is_sim_;
            int lidar_select;
            double LIDAR_THTRSHOLD_;

            // print trajectory on rviz
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
            nav_msgs::msg::Path path_;
            
            rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubPose_;

            // cmd_velのみから現在のodometryを計算する
            // last_time_に前回差分を取得したときの時刻
            // last_odom_に前回のodometryを保存
            rclcpp::Time last_timestamp_;
            geometry_msgs::msg::Pose2D velOdom_;
            geometry_msgs::msg::Pose2D last_odom_;

            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;

            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapCloud_;

            // TODO: delete
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_;
            std::float_t iter_;
            std::double_t yaw_;
            std::double_t odom_twist_;
            std::vector<geometry_msgs::msg::Point> scan_endpoints_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            
            laser_geometry::LaserProjection projector_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
            nav_msgs::msg::Odometry::SharedPtr odom_;

            // TODO: delete
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

            std::vector<SensorTransform> lidar_transforms_ = std::vector<SensorTransform>(3);

            // private 変数として定義
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_cloud_pub_; // LaserScanから変更
            void timer_callback() {
                RCLCPP_INFO(this->get_logger(), "In timer loop");
            }
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL>());
    rclcpp::shutdown();
    return 0;
}
