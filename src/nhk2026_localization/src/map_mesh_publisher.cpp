#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MapMeshPublisher : public rclcpp::Node
{
public:
  MapMeshPublisher() : Node("map_mesh_publisher")
  {
    // マーカー用のパブリッシャーを作成
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("map_mesh_marker", 10);
    
    // 1秒に1回パブリッシュするタイマー
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMeshPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto marker = visualization_msgs::msg::Marker();
    
    // Rviz上で基準となるフレーム名を指定（環境に合わせて "map" 等に変更してください）
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "map_mesh";
    marker.id = 0;
    
    // メッシュリソースとして指定
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // TODO: ここを実際のファイル名（.stl または .obj）に合わせて変更してください
    marker.mesh_resource = "package://nhk2026_sim/models/field_nhk/meshes/fieldObj_zySwap.obj";

    marker.mesh_use_embedded_materials = true;

    // 位置と姿勢 (必要に応じてモデルの原点を調整)
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // スケール (モデルが小さすぎる/大きすぎる場合はここで調整)
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // 色と透明度 (a=1.0で不透明)
    marker.color.a = 0.0; 
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    publisher_->publish(marker);
  }
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMeshPublisher>());
  rclcpp::shutdown();
  return 0;
}