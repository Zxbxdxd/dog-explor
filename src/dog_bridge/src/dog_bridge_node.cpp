// Plan B bridge (trajectory mode): relays the RACER exploration trajectory to
// the SCAN-Planner quadruped stack so SCAN follows RACER's continuous
// exploration path (navi_mode=3 / initial_path) instead of discrete goals.
//
//   RACER -> /planning/bspline_1  (Bspline) -> converted to nav_msgs/Path
//   bridge-> /initial_path                     -> SCAN follows continuously
//   SCAN  -> /quad_0/body_pose    (dog odom)  -> /odom_world   (RACER)
//   SCAN  -> /quad_0/cloud        (world cloud)-> /map_ros/cloud
//   SCAN  -> /quad_0/lidar_pose   (sensor pose)-> /map_ros/pose (reliable)
//   bridge-> /plan_b/start                      -> RACER start trigger
//
// Continuous trajectory following mirrors the original RACER behaviour (the
// robot moves through frontiers without parking), which removes the stop-and-go
// circling seen with discrete goal forwarding.

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <bspline/msg/bspline.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

class DogBridgeNode : public rclcpp::Node {
 public:
  DogBridgeNode() : Node("dog_bridge") {
    body_height_ = declare_parameter<double>("body_height", 0.4);
    path_spacing_ = declare_parameter<double>("path_spacing", 0.3);
    path_resend_dist_ = declare_parameter<double>("path_resend_dist", 1.5);
    path_interval_ = declare_parameter<double>("path_interval", 5.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    const std::string bspline_topic =
        declare_parameter<std::string>("bspline_topic", "/planning/bspline_1");
    const std::string body_pose_topic =
        declare_parameter<std::string>("body_pose_topic", "/quad_0/body_pose");
    const std::string cloud_topic = declare_parameter<std::string>("cloud_topic", "/quad_0/cloud");
    const std::string depth_topic = declare_parameter<std::string>("depth_topic", "/quad_0/depth");
    const std::string lidar_pose_topic =
        declare_parameter<std::string>("lidar_pose_topic", "/quad_0/lidar_pose");
    cam_fx_ = declare_parameter<double>("cam_fx", 387.229);
    cam_fy_ = declare_parameter<double>("cam_fy", 387.229);
    cam_cx_ = declare_parameter<double>("cam_cx", 321.046);
    cam_cy_ = declare_parameter<double>("cam_cy", 243.450);
    depth_max_ = declare_parameter<double>("depth_max", 20.0);
    const std::string path_topic = declare_parameter<std::string>("path_topic", "/initial_path");
    const std::string start_topic = declare_parameter<std::string>("start_topic", "/plan_b/start");
    start_delay_ = declare_parameter<double>("start_delay", 3.0);
    start_retry_period_ = declare_parameter<double>("start_retry_period", 1.0);
    max_start_attempts_ = declare_parameter<int>("max_start_attempts", 10);

    // Subscriptions
    racer_bspline_sub_ = create_subscription<bspline::msg::Bspline>(
        bspline_topic, rclcpp::QoS(10),
        std::bind(&DogBridgeNode::racerBsplineCb, this, std::placeholders::_1));
    body_pose_sub_ = create_subscription<Odometry>(
        body_pose_topic, rclcpp::QoS(10),
        std::bind(&DogBridgeNode::bodyPoseCb, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&DogBridgeNode::cloudCb, this, std::placeholders::_1));
    depth_sub_ = create_subscription<Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        std::bind(&DogBridgeNode::depthCb, this, std::placeholders::_1));
    lidar_pose_sub_ = create_subscription<Odometry>(
        lidar_pose_topic, rclcpp::SensorDataQoS(),
        std::bind(&DogBridgeNode::lidarPoseCb, this, std::placeholders::_1));

    // Publishers
    odom_world_pub_ = create_publisher<Odometry>("/odom_world", rclcpp::QoS(10));
    map_cloud_pub_ = create_publisher<PointCloud2>("/map_ros/cloud", rclcpp::SensorDataQoS());
    map_pose_pub_ = create_publisher<PoseStamped>("/map_ros/pose", rclcpp::QoS(10));
    path_pub_ = create_publisher<Path>(path_topic, rclcpp::QoS(10));
    start_pub_ = create_publisher<PoseStamped>(start_topic, rclcpp::QoS(10));
    // Multi-floor (Step 2): forward RACER's B-spline to SCAN's open_loop
    // controller (same message layout) so the dog follows RACER's 3D path
    // directly, bypassing SCAN's local rebound planner (which cannot route
    // ramps). Only used when multi_floor_ is true.
    bspline_out_pub_ =
        create_publisher<scan_planner_msgs::msg::Bspline>("/planning/bspline", rclcpp::QoS(10));

    // Start trigger timer
    start_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(start_retry_period_ * 1000)),
        std::bind(&DogBridgeNode::startTimerCb, this));
    // If RACER restarts (respawn after a crash), it returns to WAIT_TRIGGER.
    // The start trigger only fires while !have_bspline_, so forget a stale
    // bspline after this long without a new one, letting the trigger re-fire.
    bspline_timeout_ = declare_parameter<double>("bspline_timeout", 20.0);
    // Goal-driven mode (Step 1): the task layer sends /plan_b/target and RACER
    // plans straight to it. No frontier exploration means no circling, so the
    // exploration start trigger and the sweep fallback are both disabled.
    goal_mode_ = declare_parameter<bool>("goal_mode", false);
    // Multi-floor mode (Step 2): forward the trajectory's real 3D z (ramp
    // climbing) instead of clamping every waypoint to the flat body height.
    multi_floor_ = declare_parameter<bool>("multi_floor", false);
    // Manual-waypoint + RACER-map mode: when false, do NOT forward RACER's
    // trajectory (the dog follows a hand-authored /initial_path instead); the
    // bridge only relays cloud/odom so RACER builds the map.
    publish_path_ = declare_parameter<bool>("publish_path", true);
    // Sweep fallback timer: when RACER parks the dog with a short/static
    // trajectory, move it to the next boustrophedon waypoint so the map keeps
    // growing.
    stuck_timeout_ = declare_parameter<double>("stuck_timeout", 12.0);
    stuck_min_move_ = declare_parameter<double>("stuck_min_move", 0.3);
    sweep_x0_ = declare_parameter<double>("sweep_x0", -7.0);
    sweep_x1_ = declare_parameter<double>("sweep_x1", 7.0);
    sweep_y0_ = declare_parameter<double>("sweep_y0", -15.0);
    sweep_y1_ = declare_parameter<double>("sweep_y1", 15.0);
    sweep_step_ = declare_parameter<double>("sweep_step", 5.0);
    stuck_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&DogBridgeNode::stuckTimerCb, this));

    RCLCPP_INFO(this->get_logger(),
                "dog_bridge ready: %s -> path -> %s ; relay %s -> /odom_world, "
                "%s/%s -> /map_ros/*",
                bspline_topic.c_str(), path_topic.c_str(), body_pose_topic.c_str(),
                cloud_topic.c_str(), lidar_pose_topic.c_str());
  }

 private:
  // ------------------------------------------------------------------ relay
  void bodyPoseCb(const Odometry::ConstSharedPtr& msg) {
    // Ignore non-finite odom (e.g. the open-loop follower evaluating a bad
    // trajectory) so a NaN never propagates into RACER's planner.
    if (!std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z)) {
      return;
    }
    odom_world_pub_->publish(*msg);
    dog_x_ = msg->pose.pose.position.x;
    dog_y_ = msg->pose.pose.position.y;
    dog_z_ = msg->pose.pose.position.z;
    if (!have_odom_) {
      first_odom_time_ = now();
      last_move_time_ = now();
      have_odom_ = true;
    }
  }

  void cloudCb(const PointCloud2::ConstSharedPtr& msg) {
    map_cloud_pub_->publish(*msg);  // pass-through
  }

  // Depth -> world point cloud (for SCAN depth-camera mode, where the renderer
  // only publishes the depth image, not a world cloud). Uses the cached camera
  // pose and the pinhole intrinsics to back-project every Nth pixel.
  void depthCb(const Image::ConstSharedPtr& msg) {
    if (!have_pose_) return;
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1) return;
    const auto& p = latest_pose_.pose;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
    const uint32_t step = msg->step;
    const float* data = reinterpret_cast<const float*>(msg->data.data());
    std::vector<float> xs, ys, zs;
    xs.reserve(msg->height * msg->width / 4);
    ys.reserve(msg->height * msg->width / 4);
    zs.reserve(msg->height * msg->width / 4);
    for (uint32_t v = 0; v < msg->height; v += 2) {
      for (uint32_t u = 0; u < msg->width; u += 2) {
        const float d = data[v * (step / sizeof(float)) + u];
        if (d <= 0.05f || d > depth_max_) continue;
        const double xc = (static_cast<double>(u) - cam_cx_) * d / cam_fx_;
        const double yc = (static_cast<double>(v) - cam_cy_) * d / cam_fy_;
        const Eigen::Vector3d pw = R * Eigen::Vector3d(xc, yc, d) + t;
        xs.push_back(static_cast<float>(pw.x()));
        ys.push_back(static_cast<float>(pw.y()));
        zs.push_back(static_cast<float>(pw.z()));
      }
    }
    PointCloud2 out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_;
    out.height = 1;
    out.width = xs.size();
    out.fields.resize(3);
    out.fields[0].name = "x"; out.fields[0].offset = 0; out.fields[0].datatype = 7; out.fields[0].count = 1;
    out.fields[1].name = "y"; out.fields[1].offset = 4; out.fields[1].datatype = 7; out.fields[1].count = 1;
    out.fields[2].name = "z"; out.fields[2].offset = 8; out.fields[2].datatype = 7; out.fields[2].count = 1;
    out.point_step = 12;
    out.row_step = 12 * out.width;
    out.is_bigendian = false;
    out.is_dense = true;
    out.data.resize(12 * out.width);
    for (size_t i = 0; i < xs.size(); ++i) {
      float* ptr = reinterpret_cast<float*>(&out.data[i * 12]);
      ptr[0] = xs[i];
      ptr[1] = ys[i];
      ptr[2] = zs[i];
    }
    map_cloud_pub_->publish(out);
  }

  void lidarPoseCb(const Odometry::ConstSharedPtr& msg) {
    PoseStamped p;
    p.header = msg->header;
    p.pose = msg->pose.pose;
    latest_pose_ = p;
    have_pose_ = true;
    map_pose_pub_->publish(p);  // reliable publisher fixes the QoS mismatch
  }

  // ---------------------------------------------------- trajectory -> path
  void racerBsplineCb(const bspline::msg::Bspline::ConstSharedPtr& msg) {
    have_bspline_ = true;
    last_bspline_time_ = now();
    if (!publish_path_) return;  // manual-waypoint mode: RACER only builds the map
    if (!have_odom_) return;
    const auto& pts = msg->pos_pts;
    if (pts.empty()) return;
    // Drop a non-finite trajectory (bad plan) instead of feeding it to SCAN /
    // open_loop, which would propagate NaN into the odom.
    for (const auto& p : pts) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Dropping non-finite RACER trajectory");
        return;
      }
    }
    // Multi-floor: hand RACER's B-spline straight to open_loop_controller
    // (SCAN's 3D follower) on the topic it already subscribes to.
    if (multi_floor_) {
      scan_planner_msgs::msg::Bspline out;
      out.order = msg->order;
      out.traj_id = msg->traj_id;
      out.start_time = msg->start_time;
      out.knots = msg->knots;
      out.pos_pts = msg->pos_pts;
      out.yaw_pts = msg->yaw_pts;
      out.yaw_dt = msg->yaw_dt;
      bspline_out_pub_->publish(out);
    }
    // Throttle path updates: RACER replans every ~1-2 s, and forwarding every
    // trajectory makes SCAN re-plan constantly (the dog spends more time
    // planning than moving). Only forward when the endpoint moved
    // meaningfully or a minimum interval elapsed, so SCAN executes each path.
    const double end_x = pts.back().x;
    const double end_y = pts.back().y;
    const bool moved = std::hypot(end_x - last_pub_x_, end_y - last_pub_y_) > path_resend_dist_;
    const bool interval = last_pub_time_.nanoseconds() == 0 ||
                          (now() - last_pub_time_).seconds() >= path_interval_;
    if (!moved && !interval) return;
    last_pub_x_ = end_x;
    last_pub_y_ = end_y;
    last_pub_time_ = now();
    // Densify the B-spline control polygon into a waypoint path at ~path_spacing
    // resolution. Flat mode clamps z to the robot's body height; multi-floor
    // mode interpolates the trajectory's real z so the path can climb ramps.
    std::vector<PoseStamped> wps;
    for (size_t i = 0; i < pts.size(); ++i) {
      const double x = pts[i].x;
      const double y = pts[i].y;
      const double z = multi_floor_ ? pts[i].z : body_height_;
      if (!wps.empty()) {
        const double last_x = wps.back().pose.position.x;
        const double last_y = wps.back().pose.position.y;
        const double last_z = wps.back().pose.position.z;
        const double seg = std::hypot(x - last_x, y - last_y);
        const int n = std::max(1, static_cast<int>(seg / path_spacing_));
        for (int k = 1; k < n; ++k) {
          PoseStamped s;
          s.header.frame_id = frame_id_;
          s.pose.position.x = last_x + (x - last_x) * k / n;
          s.pose.position.y = last_y + (y - last_y) * k / n;
          s.pose.position.z = last_z + (z - last_z) * k / n;
          s.pose.orientation.w = 1.0;
          wps.push_back(s);
        }
      }
      PoseStamped p;
      p.header.frame_id = frame_id_;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.position.z = z;
      p.pose.orientation.w = 1.0;
      wps.push_back(p);
    }
    Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    path.poses = wps;
    // Route B: in multi-floor mode the open_loop follows the B-spline (already
    // forwarded above) during exploration, and the task layer publishes the
    // hand-authored climb path to /initial_path during the climb. The bridge
    // must NOT also publish /initial_path, or it overwrites the climb path.
    if (!multi_floor_) path_pub_->publish(path);
    // Track whether RACER is skimming: a short trajectory means the dog is
    // being kept near its current position (frontier skimming / circling).
    double plen = 0.0;
    for (size_t i = 1; i < wps.size(); ++i) {
      plen += std::hypot(wps[i].pose.position.x - wps[i - 1].pose.position.x,
                         wps[i].pose.position.y - wps[i - 1].pose.position.y);
    }
    if (plen < 3.0) {
      short_path_count_++;
    } else {
      short_path_count_ = 0;
    }
    // Reset the circling/stuck reference to the current dog position.
    ref_x_ = dog_x_;
    ref_y_ = dog_y_;
    path_length_ = 0.0;
    RCLCPP_INFO(get_logger(), "path published: %zu waypoints, end (%.2f, %.2f)",
                wps.size(), wps.back().pose.position.x, wps.back().pose.position.y);
  }

  // -------------------------------------------------- sweep fallback
  // RACER occasionally parks the dog with a short trajectory ending at its own
  // position. When the dog has been stuck or circling (moved far but barely
  // left the area), publish a path to the next boustrophedon waypoint so the
  // map keeps growing. A fresh real RACER trajectory overrides it (it is
  // published on the same /initial_path topic).
  void stuckTimerCb() {
    if (goal_mode_) return;  // goal-driven: no circling to detect, no sweeping
    if (multi_floor_) return;  // Route B: RACER frontier does the survey, not the sweep
    if (!have_odom_) return;
    const double moved = std::hypot(dog_x_ - last_dog_x_, dog_y_ - last_dog_y_);
    last_dog_x_ = dog_x_;
    last_dog_y_ = dog_y_;
    if (moved > 0.02) path_length_ += moved;
    if (moved > stuck_min_move_) last_move_time_ = now();
    const bool stuck = (now() - last_move_time_).seconds() >= stuck_timeout_;
    const double net = std::hypot(dog_x_ - ref_x_, dog_y_ - ref_y_);
    const bool circling = path_length_ > 6.0 * std::max(net, 0.5) && path_length_ > 8.0;
    // RACER skimming the frontier boundary publishes short trajectories
    // (the dog loops around the explored region). Two consecutive short paths
    // means it is not advancing -> take over with the systematic sweep.
    const bool skimming = short_path_count_ >= 2;
    if (stuck || circling || skimming) {
      publishSweepPath();
      last_move_time_ = now();
      path_length_ = 0.0;
      short_path_count_ = 0;
      ref_x_ = dog_x_;
      ref_y_ = dog_y_;
    }
  }

  void publishSweepPath() {
    const int nrows = std::max(1, static_cast<int>((sweep_y1_ - sweep_y0_) / sweep_step_ + 1));
    int row = sweep_index_ / 2;
    if (row >= nrows) {
      sweep_index_ = 0;
      row = 0;
    }
    const bool second = (sweep_index_ % 2 == 1);
    const double y = sweep_y0_ + row * sweep_step_;
    const bool even_row = (row % 2 == 0);
    const double x = (even_row == second) ? sweep_x0_ : sweep_x1_;
    sweep_index_++;
    Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    PoseStamped a;
    a.header = path.header;
    a.pose.position.x = dog_x_;
    a.pose.position.y = dog_y_;
    a.pose.position.z = dog_z_;
    a.pose.orientation.w = 1.0;
    PoseStamped b;
    b.header = path.header;
    b.pose.position.x = x;
    b.pose.position.y = y;
    b.pose.position.z = dog_z_;
    b.pose.orientation.w = 1.0;
    path.poses = {a, b};
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "sweep fallback path -> (%.2f, %.2f)", x, y);
  }

  // --------------------------------------------------------- start trigger
  void startTimerCb() {
    if (goal_mode_) return;  // goal-driven: task_node drives RACER, no trigger
    // A respawned RACER has no map and no bspline; forget the stale one so the
    // start trigger re-fires and exploration resumes.
    if (have_bspline_ && bspline_timeout_ > 0.0 &&
        (now() - last_bspline_time_).seconds() > bspline_timeout_) {
      RCLCPP_WARN(get_logger(), "No bspline for %.0f s; resetting (RACER may have restarted)",
                  bspline_timeout_);
      have_bspline_ = false;
    }
    if (have_bspline_ || !have_odom_) return;
    const double elapsed = (now() - first_odom_time_).seconds();
    if (elapsed < start_delay_) return;
    if (start_attempts_ >= max_start_attempts_) {
      start_timer_->cancel();
      return;
    }
    PoseStamped s;
    s.header.stamp = now();
    s.header.frame_id = frame_id_;
    s.pose.position.x = dog_x_;
    s.pose.position.y = dog_y_;
    s.pose.position.z = 0.4;
    s.pose.orientation.w = 1.0;
    start_pub_->publish(s);
    start_attempts_++;
  }

  // ------------------------------------------------------------------- data
  rclcpp::Subscription<bspline::msg::Bspline>::SharedPtr racer_bspline_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr body_pose_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr lidar_pose_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_world_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr map_pose_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_out_pub_;
  rclcpp::TimerBase::SharedPtr start_timer_;

  bool goal_mode_ = false;
  bool multi_floor_ = false;
  bool publish_path_ = true;
  double body_height_ = 0.4;
  double path_spacing_ = 0.3;
  double path_resend_dist_ = 1.5;
  double path_interval_ = 5.0;
  double last_pub_x_ = 0.0;
  double last_pub_y_ = 0.0;
  rclcpp::Time last_pub_time_;
  bool have_pose_ = false;
  PoseStamped latest_pose_;
  double cam_fx_ = 387.229;
  double cam_fy_ = 387.229;
  double cam_cx_ = 321.046;
  double cam_cy_ = 243.450;
  double depth_max_ = 20.0;
  double start_delay_ = 3.0;
  double start_retry_period_ = 1.0;
  int max_start_attempts_ = 10;
  std::string frame_id_ = "world";

  bool have_odom_ = false;
  bool have_bspline_ = false;
  double bspline_timeout_ = 20.0;
  rclcpp::Time last_bspline_time_;
  double dog_x_ = 0.0;
  double dog_y_ = 0.0;
  double dog_z_ = 0.0;
  rclcpp::Time first_odom_time_;
  int start_attempts_ = 0;

  rclcpp::TimerBase::SharedPtr stuck_timer_;
  double stuck_timeout_ = 12.0;
  double stuck_min_move_ = 0.3;
  double sweep_x0_ = -7.0;
  double sweep_x1_ = 7.0;
  double sweep_y0_ = -15.0;
  double sweep_y1_ = 15.0;
  double sweep_step_ = 5.0;
  int sweep_index_ = 0;
  int short_path_count_ = 0;
  double ref_x_ = 0.0;
  double ref_y_ = 0.0;
  double path_length_ = 0.0;
  double last_dog_x_ = 0.0;
  double last_dog_y_ = 0.0;
  rclcpp::Time last_move_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
