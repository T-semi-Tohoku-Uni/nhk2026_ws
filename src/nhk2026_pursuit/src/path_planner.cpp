#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>
#include <inrof2025_ros_type/srv/waypoint.hpp>
#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <H5Cpp.h>

using namespace H5;

namespace path {
    class PathGenerator: public rclcpp::Node {
        public:
            explicit PathGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
                : Node("path_generator", options), waypoint_array_{} {
                // TODO: get from launch file
                this->declare_parameter<std::float_t>("initial_x", 0.25);
                this->declare_parameter<std::float_t>("initial_y", 0.25);
                this->declare_parameter<std::float_t>("initial_theta", M_PI/2);
                this->declare_parameter<std::float_t>("sample_parameter", 15.0);
                this->declare_parameter<int>("mapZIndex", 4);

                double initial_x = this->get_parameter("initial_x").as_double();
                double initial_y = this->get_parameter("initial_y").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();
                sample_parameter_ = this->get_parameter("sample_parameter").as_double();
                this->mapZIndex_ = this->get_parameter("mapZIndex").as_int();

                this->curOdom_.x = initial_x;
                this->curOdom_.y = initial_y;
                this->curOdom_.theta = initial_theta;

                this->mapResolution_ = 0.01;
                this->mapWidth_ = 182;
                this->mapHeight_ = 232;
                this->mapFile_ = "src/nhk2026_localization/map/nhk2026_field.h5";
                

                readMap();

                // initialize publisher
                rclcpp::QoS pathQos = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliable()
                                  .transient_local();
                rclcpp::QoS test_pathQos = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliable()
                                  .transient_local();
                pubPath_ = create_publisher<nav_msgs::msg::Path>("route", pathQos);
                pubSamplePath_ = create_publisher<nav_msgs::msg::Path>("test_route", test_pathQos);

                rclcpp::QoS markerQos = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliable()
                                  .transient_local();
                pubMarker_ = create_publisher<visualization_msgs::msg::MarkerArray>("path_orientations", markerQos);


                // initialize subscriber
                rclcpp::QoS sOdomQos(rclcpp::KeepLast(10));
                subOdom_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                    "pose", sOdomQos, std::bind(&PathGenerator::odomCallback, this, std::placeholders::_1)
                );

                // initialize service server
                srvOdom_= this->create_service<inrof2025_ros_type::srv::GenRoute>(
                    "generate_route", std::bind(&PathGenerator::poseCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
                srvWaypoint_ = this->create_service<inrof2025_ros_type::srv::Waypoint>(
                    "waypoint", std::bind(&PathGenerator::waypointCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
                
                RCLCPP_INFO(this->get_logger(), "Success initialze");
            }
        private:
        struct mapNode {
            int r, c;
            double width;
            double cost;
        };

        std::pair<int, int> xy2uv(double x, double y){
            int u = static_cast<int>(x / mapResolution_);
            int v = mapHeight_ -1 - static_cast<int>(y / mapResolution_);
            return std::make_pair(u, v);
        }

        struct Cell {
            int u, v;
            double cost;

            bool operator>(const Cell& other) const {
                return cost > other.cost;
            }
        };

        void odomCallback(geometry_msgs::msg::Pose2D msgs) {
            // TODO lock
            curOdom_.x = msgs.x;
            curOdom_.y = msgs.y;
            curOdom_.theta = msgs.theta; // null ok
        }

        // void poseCallback(inrof2025_ros_type::srv::GenRoute srvs) {
        //     RCLCPP_INFO(this->get_logger(), "jfoejrifojerifjiorejfoierjfierjfojer");
        //     // TODO lock
        //     goalOdom_.x = srvs->x;
        //     goalOdom_.y = srvs->y;
        //     goalOdom_.theta = 0.0; // null ok

        //     generator();
        // }

        void waypointCallback(
            const std::shared_ptr<inrof2025_ros_type::srv::Waypoint::Request> request,
            const std::shared_ptr<inrof2025_ros_type::srv::Waypoint::Response> response
        ) {
            // Add array
            waypoint_array_.push_back(std::make_pair(request->x, request->y));
        }

        void poseCallback(
            const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Request> request,
            const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Response> response
        ) {
            RCLCPP_INFO(this->get_logger(), "%.4f %.4f", request->x, request->y);

            // Add goal odom
            waypoint_array_.push_back(std::make_pair(request->x, request->y));

            std::vector<std::pair<double, double>> path;
            std::pair<double, double> start_point = std::make_pair(curOdom_.x, curOdom_.y); 
            for (size_t i=0; i<waypoint_array_.size(); i++ ) {
                std::vector<std::pair<double, double>> partial_pass = generator(start_point, waypoint_array_[i]);
                path.insert(path.end(), partial_pass.begin(), partial_pass.end());
                start_point = waypoint_array_[i];
            }

            
            path = splineSmoothEigen(path);

            // create path message
            nav_msgs::msg::Path pathMsg;
            pathMsg.header.frame_id = "map";
            pathMsg.header.stamp    = this->now();
            for (size_t i=0; i<path.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = pathMsg.header;

                pose.pose.position.x = path[i].first;
                pose.pose.position.y = path[i].second;
                pose.pose.position.z = 0.0;

                tf2::Quaternion q;
                if (i+30 > path.size()) {
                    q.setRPY(0, 0, request->theta);
                } else {
                    q.setRPY(0, 0, curOdom_.theta);
                }
                pose.pose.orientation = tf2::toMsg(q);
                pathMsg.poses.push_back(pose);
            }
            pubPath_->publish(pathMsg);

            // create arrow message

            visualization_msgs::msg::MarkerArray markerArray;
            // delete old marker
            visualization_msgs::msg::Marker del;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            markerArray.markers.push_back(del);
            // add marker
            for (size_t i=0; i<pathMsg.poses.size(); i+=10) {
                visualization_msgs::msg::Marker arrow;
                arrow.header = pathMsg.header;
                arrow.ns = "path_orientations";
                arrow.id = static_cast<int>(i);
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose = pathMsg.poses[i].pose;
                arrow.scale.x = 0.05;
                arrow.scale.y = 0.01;
                arrow.scale.z = 0.01;

                arrow.color.r = 1.0f;
                arrow.color.g = 0.0f;
                arrow.color.b = 0.0f;
                arrow.color.a = 1.0f;
                markerArray.markers.push_back(arrow);
            }
            pubMarker_->publish(markerArray);

            // clear waypoint array
            waypoint_array_.clear();
        }
        
        std::vector<std::pair<double, double>> splineSmoothEigen(const std::vector<std::pair<double, double>> &original_path) {
            using Spline2d = Eigen::Spline<double, 2>;
            using Vec2 = Eigen::Matrix<double, 2, 1>;

            // sampling point from original path
            std::vector<std::pair<double, double>> sampled_path;
            for (size_t i=0; i<original_path.size(); i+=sample_parameter_ ) {
                sampled_path.push_back(original_path[i]);
            }
            sampled_path.push_back(original_path.back());

            const int degree = 3;
            if (sampled_path.size() < 4) return original_path;
            // sampleする点はdegree+1以上
            if (sampled_path.size() <= degree) return original_path;

            Eigen::Matrix<double, 2, Eigen::Dynamic> points(2, sampled_path.size());
            for (size_t i = 0; i < sampled_path.size(); i++) {
                points(0, i) = sampled_path[i].first;
                points(1, i) = sampled_path[i].second;
            }

            Eigen::RowVectorXd u(sampled_path.size());
            for (int i = 0; i < sampled_path.size(); ++i) {
               u(i) = static_cast<double>(i) / double(sampled_path.size() - 1);
            }
 
            // const int degree = 3; // 3次元のspline
            // // 注意: 点数 N は degree+1 以上であること
            // if (N <= degree) return original_path;

            Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points, degree, u);
            std::vector<std::pair<double, double>> smoothed_path;
            int dense = sampled_path.size() * sample_parameter_;
            for (int i = 0; i <= dense; i++) {
                
                double t = static_cast<double>(i) / dense; // 0..1
                
                Eigen::Vector2d pv = spline(t); // p(t)

                geometry_msgs::msg::PoseStamped pose;
                smoothed_path.push_back(std::make_pair(pv.x(), pv.y()));
            }

            return smoothed_path;
        }

        std::vector<std::pair<double, double>> generator(std::pair<double, double> start_point, std::pair<double, double> goal_point) {
            double sx = start_point.first;
            double sy = start_point.second;
            double gx = goal_point.first;
            double gy = goal_point.second;

            std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> q;
            std::map<std::pair<int, int> ,double> distances;
            for (int v = 0; v < this->mapHeight_; v++) {
                for (int u = -mapWidth_; u < this->mapWidth_; u++) {
                    distances[{v, u}] = std::numeric_limits<double>::infinity();
                }
            }

            // std::vector<std::vector<std::pair<int, int>>> previous(
            //     this->mapHeight_, std::vector<std::pair<int, int>>(this->mapWidth_, {-1, -1})
            // );
            std::map<std::pair<int, int>, std::pair<int, int>> previous;
            for (int v = 0; v < this->mapHeight_; v++) {
                for (int u = -mapWidth_; u < this->mapWidth_; u++) {
                    previous[{v, u}] = std::make_pair(std::numeric_limits<int>::infinity(), std::numeric_limits<int>::infinity());
                }
            }

            int su, sv, gu, gv;
            std::pair<int, int> uv_start = xy2uv(sx, sy);
            std::pair<int, int> uv_goal  = xy2uv(gx, gy);
            su = uv_start.first;
            sv = uv_start.second;
            gu = uv_goal.first;
            gv = uv_goal.second;


            // RCLCPP_INFO(this->get_logger(), "start %d %d", su, sv);

            distances[{sv,su}] = 0;
            q.push({su, sv, 0});

            const int du[4] = {-1, 1, 0, 0};
            const int dv[4] = {0, 0, -1, 1};

            while(!q.empty()) { 
                Cell cur = q.top(); q.pop();
                if (cur.u == gu && cur.v == gv) break;

                for (int dir = 0; dir < 4; dir++ ) {
                    int nu = cur.u + du[dir];
                    int nv = cur.v + dv[dir];

                    if (nu >= -mapWidth_ && nu < this->mapWidth_ && nv >= 0 && nv < this->mapHeight_) {
                        double cost = cur.cost + distField_.at<double>(nv, std::abs(nu));
                        if (cost < distances[{nv,nu}]) {
                            distances[{nv,nu}] = cost;
                            previous[{nv,nu}] = std::make_pair(cur.u, cur.v);
                            q.push({nu, nv, cost});
                        }
                    }
                }
            }

            // 経路再構築
            std::vector<std::pair<int, int>> path_g;
            int idx=0;
            for (int u = gu, v = gv; u != std::numeric_limits<int>::infinity() && v != std::numeric_limits<int>::infinity();) {
                path_g.push_back({u, v});
                std::tie(u, v) = previous[{v,u}];
            }

            std::reverse(path_g.begin(), path_g.end());

            // convert grid -> field
            std::vector<std::pair<double, double>> path_f;
            for (std::pair<int, int> &p: path_g) {
                path_f.push_back(
                    std::make_pair(
                        (p.first + 0.5) * mapResolution_,
                        (static_cast<double>(mapHeight_ - p.second - 1) + 0.5) * mapResolution_
                    )
                );
            }

            return path_f;
        }

        void publishVoxelMap(const std::vector<uint8_t>& map_data, hsize_t dim_x, hsize_t dim_y, hsize_t dim_z) {
                // インデックスの範囲チェック
                if (this->mapZIndex_ < 0 || this->mapZIndex_ >= (int)dim_z) {
                    RCLCPP_ERROR(this->get_logger(), "mapZIndex %d is out of bounds (0 to %llu)", this->mapZIndex_, dim_z - 1);
                    return;
                }
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
                    std::pair<int, int> uv_origin = xy2uv(0.0, 0.0);
                    u_origin = uv_origin.first;
                    v_origin = uv_origin.second;

                    
                    if (u_origin >= 0 && u_origin < (int)dim_x && v_origin >= 0 && v_origin < (int)dim_y) {
                        
                        cv::circle(debug_binary_color, cv::Point(u_origin, v_origin), 5, cv::Scalar(0, 255, 0), -1);
                        RCLCPP_INFO(this->get_logger(), "Physical origin (0,0) is at image pixel: u=%d, v=%d", u_origin, v_origin);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Physical origin (0,0) is outside the map boundaries!");
                    }

                    
                    std::int32_t u_init, v_init;
                    std::pair<int, int> uv_init = xy2uv(0.25, 0.25); 
                    u_init = uv_init.first;
                    v_init = uv_init.second;
                    
                    

                    if (u_init >= 0 && u_init < binary_img.cols && v_init >= 0 && v_init < binary_img.rows) {
                
                        cv::circle(debug_binary_color, cv::Point(u_init, v_init), 5, cv::Scalar(255, 0, 0), -1);
                    }

                    // drawMarkerOnImage(debug_binary_color, 0.32, 0.02, cv::Scalar(0, 0, 255));

                
                    cv::imwrite("debug_binary_map.png", debug_binary_color);
                    

        
                    cv::imwrite("debug_raw_slice.png", raw_slice_img);


                    RCLCPP_INFO(this->get_logger(), "Saved raw Z-slice image to debug_raw_slice.png");
                
                    mapImg_ = binary_img.clone();

                    
                    cv::Mat distFieldF;
                    cv::Mat distFieldD;
                    cv::distanceTransform(binary_img, distFieldF, cv::DIST_L2, 5);

                  
                    //  distFieldD = d * mapResolution_
                    distFieldD = cv::Mat(dim_y, dim_x, CV_64FC1); // メモリ確保
                    for (int y = 0; y < (int)dim_y; ++y) {
                        for (int x = 0; x < (int)dim_x; ++x) {
                            float d_pixel = distFieldF.at<float>(y, x);
                            distFieldD.at<double>(y, x) = (double)d_pixel * mapResolution_;
                        }
                    }

                    double max_val;
                    cv::minMaxLoc(distFieldD, nullptr, &max_val, nullptr, nullptr);
                    cv::Mat diff = distFieldD - max_val;   // 要素ごとに引き算
                    cv::Mat absDiff = cv::abs(diff);      // 要素ごとの絶対値
                
                    distField_ = absDiff.clone();

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


        std::array<std::pair<int,int>, 8> directions8_ {{
            {-1,  0},   // 上        (north)
            { 1,  0},   // 下        (south)
            { 0, -1},   // 左        (west)
            { 0,  1},   // 右        (east)
            {-1, -1},   // 左上      (north-west)
            {-1,  1},   // 右上      (north-east)
            { 1, -1},   // 左下      (south-west)
            { 1,  1}    // 右下      (south-east)
        }};
        double sample_parameter_;
        std::string mapFile_;
        std::double_t mapResolution_;
        std::int32_t mapWidth_, mapHeight_;
        std::vector<std::double_t> mapOrigin_;
        cv::Mat mapImg_;
        cv::Mat distField_;
        int mapZIndex_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubSamplePath_;        
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarker_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subOdom_;
        geometry_msgs::msg::Pose2D curOdom_;
        std::vector<std::pair<double, double>> waypoint_array_;

        // connect to behaivorTree
        rclcpp::Service<inrof2025_ros_type::srv::GenRoute>::SharedPtr srvOdom_;
        rclcpp::Service<inrof2025_ros_type::srv::Waypoint>::SharedPtr srvWaypoint_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path::PathGenerator>());
    rclcpp::shutdown();
    return 0;
}