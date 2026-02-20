#include <memory>
#include <cmath>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// ベクトル計算用の簡易構造体
struct Point2D {
  double x;
  double y;
};

class ScanFilterNode : public rclcpp::Node
{
public:
  ScanFilterNode()
  : Node("scan_filter_node")
  {
    // パラメータの宣言 (しきい値: 1に近いほど厳しく除去)
    // cos(theta)の絶対値がこの値より大きければ除去します
    this->declare_parameter("filter_threshold", 0.95);

    // サブスクライバーとパブリッシャーの設定
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(10), std::bind(&ScanFilterNode::scan_callback, this, std::placeholders::_1));
    
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_filtered", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Scan Filter Node has started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 出力用メッセージを作成（ヘッダなどはコピー）
    auto filtered_scan = *msg;
    double threshold = this->get_parameter("filter_threshold").as_double();

    size_t num_points = msg->ranges.size();
    
    // 計算用に座標変換した点を保持するバッファ
    std::vector<Point2D> points(num_points);
    std::vector<bool> is_valid(num_points, false);

    // 1. 全点を直交座標(x, y)に変換
    for (size_t i = 0; i < num_points; ++i) {
      double r = msg->ranges[i];
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        double angle = msg->angle_min + i * msg->angle_increment;
        points[i].x = r * std::cos(angle);
        points[i].y = r * std::sin(angle);
        is_valid[i] = true;
      } else {
        is_valid[i] = false;
      }
    }

    // 2. 隣り合う2点を評価してフィルタリング
    // i と i+1 を比較します
    for (size_t i = 0; i < num_points - 1; ++i) {
      if (!is_valid[i] || !is_valid[i+1]) {
        continue;
      }

      Point2D pA = points[i];
      Point2D pB = points[i+1];

      // AからBへのベクトル (vec_AB)
      Point2D vec_AB = {pB.x - pA.x, pB.y - pA.y};
      
      // AとBの中点の位置ベクトル (vec_M) ※原点からのベクトル
      Point2D vec_M = {(pA.x + pB.x) / 2.0, (pA.y + pB.y) / 2.0};

      // それぞれの大きさを計算
      double norm_AB = std::hypot(vec_AB.x, vec_AB.y);
      double norm_M = std::hypot(vec_M.x, vec_M.y);

      // ゼロ除算回避
      if (norm_AB < 1e-6 || norm_M < 1e-6) {
        continue;
      }

      // 内積を計算 (正規化してから内積 = cos theta)
      // dot = (AB_x * M_x + AB_y * M_y) / (|AB| * |M|)
      double dot_product = (vec_AB.x * vec_M.x + vec_AB.y * vec_M.y) / (norm_AB * norm_M);
      
      // 絶対値をとる
      double abs_cos = std::abs(dot_product);

      // 指定したしきい値(1に近い値)を超えていたら除去
      if (abs_cos > threshold) {
        // 該当する2点を無効値(infinity)にする
        filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
        filtered_scan.ranges[i+1] = std::numeric_limits<float>::infinity();
        
        // Intensityがある場合もゼロにしておく（任意）
        if (!filtered_scan.intensities.empty()) {
            filtered_scan.intensities[i] = 0;
            filtered_scan.intensities[i+1] = 0;
        }
      }
    }

    // 配信
    scan_pub_->publish(filtered_scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}