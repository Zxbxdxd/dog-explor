// cloud_accumulator: accumulates the lidar point cloud over time into a single
// growing map cloud, so RViz shows the building being "mapped" as the dog
// follows a manual waypoint path. Points are appended per frame and downsampled
// to a bounded count. Uses only x/y/z fields.
//
//   /quad_0/cloud (sensor_msgs/PointCloud2)  ->  /map_ros/accumulated_cloud

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class CloudAccumulator : public rclcpp::Node {
 public:
  CloudAccumulator() : Node("cloud_accumulator") {
    max_points_ = declare_parameter<int>("max_points", 1000000);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) { accumulate(msg); });
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "accumulated_cloud", rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "cloud_accumulator ready (max %d points)", max_points_);
  }

 private:
  void accumulate(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    if (msg->fields.size() < 3) return;
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    const size_t n = msg->width * msg->height;
    // Take every 3rd point to bound the growth rate.
    for (size_t i = 0; i < n; i += 3, ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) continue;
      points_.push_back(*it_x);
      points_.push_back(*it_y);
      points_.push_back(*it_z);
    }
    // Downsample if over the cap.
    if (static_cast<int>(points_.size() / 3) > max_points_) {
      std::vector<float> decimated;
      decimated.reserve(max_points_ * 3);
      const size_t stride = (points_.size() / 3) / max_points_;
      for (size_t i = 0; i + 2 < points_.size() && decimated.size() < size_t(max_points_) * 3;
           i += 3 * std::max<size_t>(stride, 1)) {
        decimated.insert(decimated.end(), points_.begin() + i, points_.begin() + i + 3);
      }
      points_.swap(decimated);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "accumulated map: %zu points (decimated)",
                           points_.size() / 3);
    }
    publishCloud();
  }

  void publishCloud() {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = now();
    out.header.frame_id = "world";
    out.height = 1;
    out.width = points_.size() / 3;
    out.fields.resize(3);
    out.fields[0].name = "x"; out.fields[0].offset = 0; out.fields[0].datatype = 7; out.fields[0].count = 1;
    out.fields[1].name = "y"; out.fields[1].offset = 4; out.fields[1].datatype = 7; out.fields[1].count = 1;
    out.fields[2].name = "z"; out.fields[2].offset = 8; out.fields[2].datatype = 7; out.fields[2].count = 1;
    out.point_step = 12;
    out.row_step = 12 * out.width;
    out.is_bigendian = false;
    out.is_dense = false;
    out.data.resize(12 * out.width);
    for (size_t i = 0; i < points_.size(); ++i)
      reinterpret_cast<float*>(&out.data[i * 4])[0] = points_[i];
    cloud_pub_->publish(out);
  }

  int max_points_ = 1000000;
  std::vector<float> points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudAccumulator>());
  rclcpp::shutdown();
  return 0;
}
