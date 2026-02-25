#include <rclcpp/rclcpp.hpp>
#include <H5Cpp.h>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace H5;

namespace mcl {
    class MCL_3D: public rclcpp::Node{
        public:
            explicit MCL_3D(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("mcl_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
                this->declare_parameter<std::string>("mapFile", "src/nhk2026_localization/map/nhk2026_field.h5"); // HDF5ファイルのパス
                this->declare_parameter<double>("mapResolution", 0.01); // 解像度のパラメータも追加

                this->mapFile_ = this->get_parameter("mapFile").as_string();
                this->mapResolution_ = this->get_parameter("mapResolution").as_double();

                //sdfのrviz設定
                rclcpp::QoS qos(rclcpp::KeepLast(1));
                qos.transient_local().reliable(); // 1回パブリッシュすればRVizで後からでも見られるようにする
                sdf_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sdf_cloud", qos);

                MCL_3D::readMap();
                RCLCPP_INFO(this->get_logger(), "Success initialize");
            }

        private:
            // 1次元の2乗距離変換 (Felzenszwalb & Huttenlocher)
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

            void readMap() {
                try {
                    RCLCPP_INFO(this->get_logger(), "Loading 3D map from: %s", this->mapFile_.c_str());
                    
                    H5File file(this->mapFile_, H5F_ACC_RDONLY);
                    DataSet dataset = file.openDataSet("map_data");
                    DataSpace dataspace = dataset.getSpace();
                    
                    int rank = dataspace.getSimpleExtentNdims();
                    std::vector<hsize_t> dims(rank);
                    dataspace.getSimpleExtentDims(dims.data(), NULL);

                    // クラスメンバに保存
                    dim_x_ = dims[0];
                    dim_y_ = dims[1];
                    dim_z_ = dims[2];
                    mapWidth_  = dim_x_;
                    mapHeight_ = dim_y_;

                    RCLCPP_INFO(this->get_logger(), "3D Map Shape: (%llu, %llu, %llu)", dim_x_, dim_y_, dim_z_);

                    // 原点情報の読み込み (既存のコードをそのまま使用)
                    if (mapResolution_ <= 0.0) mapResolution_ = 0.01;
                    if (dataset.attrExists("origin")) {
                        Attribute attr = dataset.openAttribute("origin");
                        float origin_buf[3];
                        attr.read(PredType::NATIVE_FLOAT, origin_buf);
                        mapOrigin_ = {(double)origin_buf[0], (double)origin_buf[1], (double)origin_buf[2]};
                    }

                    // HDF5から全データを読み込み
                    size_t total_size = dim_x_ * dim_y_ * dim_z_;
                    std::vector<uint8_t> map_data(total_size);
                    dataset.read(map_data.data(), PredType::NATIVE_UINT8);

                    // 3D SDF計算のための初期化
                    const float INF = 1e9;
                    std::vector<float> distFieldOut(total_size, 0.0f);
                    std::vector<float> distFieldIn(total_size, 0.0f);

                    for (size_t i = 0; i < total_size; ++i) {
                        if (map_data[i] == 1) {
                            // 障害物
                            distFieldOut[i] = 0.0f; // 障害物自身は距離0
                            distFieldIn[i]  = INF;  // 内部距離計算用
                        } else {
                            // 自由空間
                            distFieldOut[i] = INF;  // 外部距離計算用
                            distFieldIn[i]  = 0.0f; // 自由空間自身は距離0
                        }
                    }

                    // 両方に対して3D距離変換を適用 (並列処理ができるならomp parallel等を入れると高速化します)
                    RCLCPP_INFO(this->get_logger(), "Computing 3D SDF...");
                    compute3DEDT(distFieldOut);
                    compute3DEDT(distFieldIn);

                    // 最終的なSDFの構築 (内部はマイナス、外部はプラス)
                    distField3D_.resize(total_size);
                    for (size_t i = 0; i < total_size; ++i) {
                        // compute3DEDTは「距離の2乗」を返すため、平方根をとる
                        float d_out = std::sqrt(distFieldOut[i]);
                        float d_in  = std::sqrt(distFieldIn[i]);
                        
                        // SDF = (外部距離 - 内部距離) * 解像度
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

            // 3次元グリッドへの適用
            void compute3DEDT(std::vector<float>& grid) {
                // X軸方向のパス
                for (size_t z = 0; z < dim_z_; z++) {
                    for (size_t y = 0; y < dim_y_; y++) {
                        std::vector<float> f(dim_x_), d(dim_x_);
                        for (size_t x = 0; x < dim_x_; x++) f[x] = grid[getIdx3D(x, y, z)];
                        dt1d(f, d, dim_x_);
                        for (size_t x = 0; x < dim_x_; x++) grid[getIdx3D(x, y, z)] = d[x];
                    }
                }
                // Y軸方向のパス
                for (size_t z = 0; z < dim_z_; z++) {
                    for (size_t x = 0; x < dim_x_; x++) {
                        std::vector<float> f(dim_y_), d(dim_y_);
                        for (size_t y = 0; y < dim_y_; y++) f[y] = grid[getIdx3D(x, y, z)];
                        dt1d(f, d, dim_y_);
                        for (size_t y = 0; y < dim_y_; y++) grid[getIdx3D(x, y, z)] = d[y];
                    }
                }
                // Z軸方向のパス
                for (size_t y = 0; y < dim_y_; y++) {
                    for (size_t x = 0; x < dim_x_; x++) {
                        std::vector<float> f(dim_z_), d(dim_z_);
                        for (size_t z = 0; z < dim_z_; z++) f[z] = grid[getIdx3D(x, y, z)];
                        dt1d(f, d, dim_z_);
                        for (size_t z = 0; z < dim_z_; z++) grid[getIdx3D(x, y, z)] = d[z];
                    }
                }
            }

            void publishSDFCloud() {
                if (distField3D_.empty()) return;

                RCLCPP_INFO(this->get_logger(), "Generating PointCloud for 3D SDF visualization...");

                // 可視化する距離の閾値 (例: 障害物の表面から ±0.15m 以内の点だけを描画)
                const float VIZ_THRESHOLD = 0.15f; 

                // 一時保存用の構造体
                struct PointRGB {
                    float x, y, z;
                    uint8_t r, g, b;
                };
                std::vector<PointRGB> viz_points;

                for (size_t z = 0; z < dim_z_; ++z) {
                    for (size_t y = 0; y < dim_y_; ++y) {
                        for (size_t x = 0; x < dim_x_; ++x) {
                            float d = distField3D_[getIdx3D(x, y, z)];

                            // 表面付近のポイントのみを抽出してRVizを軽くする
                            if (std::abs(d) > VIZ_THRESHOLD) continue;

                            PointRGB p;
                            // ボクセルのインデックスから物理座標(m)へ変換
                            p.x = static_cast<float>(x) * mapResolution_ + mapOrigin_[0];
                            p.y = static_cast<float>(y) * mapResolution_ + mapOrigin_[1];
                            p.z = static_cast<float>(z) * mapResolution_ + mapOrigin_[2];

                            // 距離に応じて色分け (グラデーション)
                            // 障害物内部(マイナス) は赤、表面(0) は白、外部(プラス) は青
                            if (d < 0.0f) {
                                // 内部: 濃い赤 〜 薄い赤
                                float ratio = std::max(0.0f, 1.0f - (std::abs(d) / VIZ_THRESHOLD));
                                p.r = 255;
                                p.g = static_cast<uint8_t>(255 * ratio);
                                p.b = static_cast<uint8_t>(255 * ratio);
                            } else {
                                // 外部: 白 〜 青
                                float ratio = std::max(0.0f, 1.0f - (d / VIZ_THRESHOLD));
                                p.r = static_cast<uint8_t>(255 * ratio);
                                p.g = static_cast<uint8_t>(255 * ratio);
                                p.b = 255;
                            }
                            viz_points.push_back(p);
                        }
                    }
                }

                // PointCloud2 メッセージの作成
                sensor_msgs::msg::PointCloud2 cloud_msg;
                cloud_msg.header.stamp = this->now();
                cloud_msg.header.frame_id = "map"; // 地図座標系
                cloud_msg.height = 1;
                cloud_msg.width = viz_points.size();
                cloud_msg.is_dense = true;
                cloud_msg.is_bigendian = false;

                // xyz と rgb のフィールドを設定
                sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(viz_points.size());

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
                sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

                // データを詰める
                for (const auto& vp : viz_points) {
                    *iter_x = vp.x; *iter_y = vp.y; *iter_z = vp.z;
                    *iter_r = vp.r; *iter_g = vp.g; *iter_b = vp.b;
                    
                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                sdf_cloud_pub_->publish(cloud_msg);
                RCLCPP_INFO(this->get_logger(), "Published SDF PointCloud with %zu points.", viz_points.size());
            }


            std::string mapFile_;
            double mapResolution_;
            int mapWidth_, mapHeight_;
            std::vector<double> mapOrigin_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;

          
            hsize_t dim_x_, dim_y_, dim_z_;
            std::vector<float> distField3D_; 

            // 追加: (x, y, z) からフラット配列のインデックスを取得するヘルパー
            inline size_t getIdx3D(int x, int y, int z) const {
                // 現在のコードのレイアウトに合わせる: x * (dim_y * dim_z) + y * dim_z + z
                return static_cast<size_t>(x) * (dim_y_ * dim_z_) + static_cast<size_t>(y) * dim_z_ + z;
            }

            //sdf用の宣言
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_cloud_pub_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL_3D>());
    rclcpp::shutdown();
    return 0;
}