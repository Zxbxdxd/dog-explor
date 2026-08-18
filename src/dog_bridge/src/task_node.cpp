// task_node: Plan B Step 1 goal-driven target sequencer.
//
// Reads a target list [{x, y, z, yaw}, ...] from a YAML params file, publishes
// one target at a time on /plan_b/target for RACER (goal mode) to plan to, and
// advances to the next target when RACER reports arrival on /plan_b/reached.
// No frontier exploration, so there is nothing for RACER to circle around.
//
//   task_node -> /plan_b/target (PoseStamped) -> RACER goalCallback
//   RACER     -> /plan_b/reached (Empty)      -> task_node
//
// Params (usually from a YAML params file passed at launch):
//   target_x/y/z/yaw : equal-length vectors; the i-th target.
//   target_topic     : where to publish targets (/plan_b/target).
//   reached_topic    : where RACER reports arrival (/plan_b/reached).
//   start_delay      : seconds of odom before sending the first target (so
//                      RACER's FSM has left INIT and is in WAIT_TRIGGER).
//   target_timeout   : seconds after which a target is skipped (0 = disabled);
//                      guards the demo against an unreachable target.

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Empty;

class TaskNode : public rclcpp::Node {
 public:
  TaskNode() : Node("task_node") {
    const std::vector<double> empty;
    target_x_ = declare_parameter<std::vector<double>>("target_x", empty);
    target_y_ = declare_parameter<std::vector<double>>("target_y", empty);
    target_z_ = declare_parameter<std::vector<double>>("target_z", empty);
    target_yaw_ = declare_parameter<std::vector<double>>("target_yaw", empty);
    const std::vector<std::string> empty_s;
    target_floor_ = declare_parameter<std::vector<std::string>>("target_floor", empty_s);
    // Layered exploration (Plan B Step 3): RACER explores each floor; when the
    // current floor's coverage reaches the threshold, send the staged climb
    // targets to the next floor's ramp entrance, then resume exploration.
    layered_ = declare_parameter<bool>("layered_mode", false);
    coverage_threshold_ = declare_parameter<double>("coverage_threshold", 40.0);
    coverage_hold_sec_ = declare_parameter<double>("coverage_hold_sec", 5.0);
    // Explore each floor for at most this long; the climb triggers earlier if
    // the coverage threshold is reached (whichever comes first). Guards against
    // the frontier explorer circling a floor without covering it.
    explore_time_sec_ = declare_parameter<double>("explore_time_sec", 60.0);
    // Layered exploration: publish the current floor's body height so RACER
    // locks its exploration z to it (manual floor-height control).
    floor_height_topic_ = declare_parameter<std::string>("floor_height_topic", "/plan_b/floor_height");
    const std::vector<double> empty_d;
    floor_height_ = declare_parameter<std::vector<double>>("floor_height", empty_d);
    climb_x_ = declare_parameter<std::vector<double>>("climb_x", empty);
    climb_y_ = declare_parameter<std::vector<double>>("climb_y", empty);
    climb_z_ = declare_parameter<std::vector<double>>("climb_z", empty);
    climb_yaw_ = declare_parameter<std::vector<double>>("climb_yaw", empty);
    climb_floor_ = declare_parameter<std::vector<std::string>>("climb_floor", empty_s);
    const std::vector<long int> empty_i;
    climb_groups_ = declare_parameter<std::vector<long int>>("climb_groups", empty_i);
    // Dynamic climb: approach hops from the dog's CURRENT position to the first
    // fixed climb target of each group, spaced this far apart, so the
    // non-optimistic A* can traverse incrementally (the lidar maps ~6 m ahead,
    // so a 3 m hop stays inside the known-free region) instead of a single far
    // jump into unmapped floor.
    climb_hop_dist_ = declare_parameter<double>("climb_hop_dist", 3.0);
    // Route B approach: if the dog is farther than this from the first climb
    // target (the ramp/stairs base), first ask RACER to A*-route it back through
    // the already-mapped free space (horizontal, non-optimistic), and only then
    // publish the manual vertical-climb waypoints. This replaces the old blind
    // 14-18 m straight line that cut through walls / flew.
    approach_thresh_ = declare_parameter<double>("approach_thresh", 2.0);
    approach_timeout_ = declare_parameter<double>("approach_timeout", 15.0);
    climb_reach_xy_ = declare_parameter<double>("climb_reach_xy", 0.25);
    climb_reach_z_ = declare_parameter<double>("climb_reach_z", 0.08);
    resume_delay_sec_ = declare_parameter<double>("resume_delay_sec", 0.5);
    const std::string coverage_topic =
        declare_parameter<std::string>("coverage_topic", "/map_ros/coverage_floor");
    target_topic_ = declare_parameter<std::string>("target_topic", "/plan_b/target");
    reached_topic_ = declare_parameter<std::string>("reached_topic", "/plan_b/reached");
    start_delay_ = declare_parameter<double>("start_delay", 3.0);
    target_timeout_ = declare_parameter<double>("target_timeout", 90.0);
    const std::string body_pose_topic =
        declare_parameter<std::string>("body_pose_topic", "/quad_0/body_pose");

    const size_t n = target_x_.size();
    if (target_y_.size() != n || target_z_.size() != n || target_yaw_.size() != n) {
      RCLCPP_ERROR(get_logger(),
                   "target_x/y/z/yaw lengths differ (%zu/%zu/%zu/%zu); disabling",
                   target_x_.size(), target_y_.size(), target_z_.size(), target_yaw_.size());
      n_ = 0;
    } else {
      n_ = n;
    }

    target_pub_ = create_publisher<PoseStamped>(target_topic_, rclcpp::QoS(10));
    floor_height_pub_ =
        create_publisher<std_msgs::msg::Float64>(floor_height_topic_, rclcpp::QoS(10));
    lidar_pitch_pub_ =
        create_publisher<std_msgs::msg::Float64>("/plan_b/lidar_pitch", rclcpp::QoS(10));
    // Route B: manual-waypoint climb. The task layer publishes the climb waypoints
    // as a Path that open_loop follows directly (no RACER A* planning), and a
    // bool that switches open_loop from B-spline following (exploration) to path
    // following (climb).
    manual_path_pub_ = create_publisher<nav_msgs::msg::Path>("/initial_path", rclcpp::QoS(10));
    path_follow_pub_ = create_publisher<std_msgs::msg::Bool>("/plan_b/path_follow", rclcpp::QoS(10));
    const std::string manual_climb_topic =
        declare_parameter<std::string>("manual_climb_topic", "/plan_b/manual_climb");
    manual_climb_pub_ =
        create_publisher<std_msgs::msg::Bool>(manual_climb_topic, rclcpp::QoS(10));
    manual_climb_ = declare_parameter<bool>("manual_climb", true);
    reached_sub_ = create_subscription<Empty>(
        reached_topic_, rclcpp::QoS(10),
        [this](const Empty::ConstSharedPtr&) { onReached(); });
    odom_sub_ = create_subscription<Odometry>(
        body_pose_topic, rclcpp::SensorDataQoS(),
        [this](const Odometry::ConstSharedPtr& msg) {
          dog_x_ = msg->pose.pose.position.x;
          dog_y_ = msg->pose.pose.position.y;
          dog_z_ = msg->pose.pose.position.z;
          if (!have_odom_) {
            have_odom_ = true;
            first_odom_time_ = now();
            floor_start_time_ = now();
            floor_start_coverage_ = coverage_;  // baseline for floor 0
            RCLCPP_INFO(get_logger(), "odom ready; %zu target(s) loaded", n_);
            if (layered_) publishFloorHeight(0);  // lock exploration to floor 0
          }
        });
    coverage_sub_ = create_subscription<std_msgs::msg::Float64>(
        coverage_topic, rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64::ConstSharedPtr& msg) {
          coverage_ = msg->data;
          if (layered_ && (now() - first_odom_time_).seconds() >= start_delay_)
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "floor coverage: %.1f%% (threshold %.1f%%)",
                                 coverage_, coverage_threshold_);
        });

    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { tick(); });

    // Initialize all rclcpp::Time members with the node's clock source so the
    // time subtractions in tick() never mix RCL_ROS_TIME with RCL_SYSTEM_TIME.
    first_odom_time_ = now();
    floor_start_time_ = now();
    coverage_hold_start_ = now();
    last_floor_pub_time_ = now();
    sent_time_ = now();
    resume_pending_time_ = now();

    if (layered_) {
      size_t csum = 0;
      for (int g : climb_groups_) csum += static_cast<size_t>(g);
      const size_t cmin = climb_x_.size();
      if (climb_y_.size() != cmin || climb_z_.size() != cmin ||
          climb_groups_.empty() || csum != cmin) {
        RCLCPP_ERROR(get_logger(),
                     "layered_mode: climb_x/y/z lengths must match and sum of climb_groups "
                     "must equal them (x=%zu groups sum=%zu); disabling layered mode",
                     cmin, csum);
        layered_ = false;
      }
    }
    RCLCPP_INFO(get_logger(), "task_node ready: %s mode, %zu target(s) on %s (reached via %s)",
                layered_ ? "layered-exploration" : "goal", n_, target_topic_.c_str(),
                reached_topic_.c_str());
    for (size_t i = 0; i < n_; ++i) {
      const std::string fl = i < target_floor_.size() ? target_floor_[i] : std::string("?");
      RCLCPP_INFO(get_logger(), "  [%zu] %s (%.2f, %.2f, %.2f) yaw %.2f", i, fl.c_str(),
                  target_x_[i], target_y_[i], target_z_[i], target_yaw_[i]);
    }
  }

 private:
  void tick() {
    if (!have_odom_ || done_) return;
    if (!layered_) {
      // Goal mode: send the target list one by one.
      if (n_ == 0) return;
      if (!sent_) {
        if ((now() - first_odom_time_).seconds() < start_delay_) return;
        publishGoal(target_x_[idx_], target_y_[idx_], target_z_[idx_], target_yaw_[idx_],
                    idx_ < target_floor_.size() ? target_floor_[idx_] : "?");
      } else if (target_timeout_ > 0.0 &&
                 (now() - sent_time_).seconds() >= target_timeout_) {
        RCLCPP_WARN(get_logger(), "Target %zu/%zu timed out after %.0f s; skipping",
                    idx_ + 1, n_, target_timeout_);
        advanceGoal();
      }
      return;
    }

    // Layered exploration mode.
    // Re-publish the current floor height periodically (volatile topic: a
    // single message can be missed if RACER subscribes late).
    if (layered_ && !climbing_ &&
        (now() - last_floor_pub_time_).seconds() >= 2.0) {
      publishFloorHeight(group_idx_);
      last_floor_pub_time_ = now();
    }
    if (resume_pending_) {
      // Give the floor-height message a full task tick to reach RACER before
      // sending the resume edge. This makes the first post-climb trajectory use
      // the new layer height even though the messages use different topics.
      if ((now() - resume_pending_time_).seconds() >= resume_delay_sec_) {
        publishManualClimbState(false);
        resume_pending_ = false;
      }
      return;
    }
    if (climbing_) {
      if (manual_climb_) {
        // Wait for RACER's collision-checked plan to return the dog to the
        // ramp/stairs base.  Only after reaching that base do we hand the
        // actual climb to the fixed straight-ramp path follower.
        if (approach_phase_) {
          // Retrying is safe; falling back to a direct dog-to-ramp path is not,
          // because that chord can cross a wall or pillar.
          if (approach_timeout_ > 0.0 && sent_ &&
              (now() - sent_time_).seconds() >= approach_timeout_) {
            RCLCPP_WARN(get_logger(),
                        "RACER approach to ramp base timed out after %.0f s; retrying planned approach",
                        approach_timeout_);
            sent_ = false;
            if (approach_idx_ < approach_x_.size()) {
              publishGoal(approach_x_[approach_idx_], approach_y_[approach_idx_],
                          approach_z_[approach_idx_], 0.0, "approach-retry");
            }
          }
          return;
        }
        // Route B: the climb completes when the dog reaches the last climb
        // target of the group (odom-based; open_loop follows the manual path
        // and RACER does not report /plan_b/reached).
        const int gsz = group_idx_ < climb_groups_.size() ? climb_groups_[group_idx_] : 0;
        const size_t last = group_start_ + gsz - 1;
        if (gsz > 0 && last < climb_x_.size()) {
          const double d = std::hypot(dog_x_ - climb_x_[last], dog_y_ - climb_y_[last]);
          // Also require the dog to have landed on the next floor's plane (the
          // final landing waypoint), not just be horizontally near the last
          // climb target -- otherwise it stalls on the last short hop.
          const double z_next = group_idx_ + 1 < floor_height_.size()
                                    ? floor_height_[group_idx_ + 1] : climb_z_[last];
          const double dz = std::fabs(dog_z_ - z_next);
          if (d < climb_reach_xy_ && dz < climb_reach_z_) {
            RCLCPP_INFO(get_logger(), "Manual climb landed (dist %.2f, dz %.2f)", d, dz);
            finishClimb();
            return;
          }
        }
        if (target_timeout_ > 0.0 &&
            (now() - climb_start_time_).seconds() >= target_timeout_) {
          RCLCPP_WARN(get_logger(), "Manual climb timed out; skipping");
          finishClimb();
        }
        return;
      }
      if (sent_ && target_timeout_ > 0.0 &&
          (now() - sent_time_).seconds() >= target_timeout_) {
        RCLCPP_WARN(get_logger(), "Climb target %zu timed out; skipping", climb_idx_ + 1);
        advanceClimb();
      }
      return;
    }
    // Exploring a floor: trigger the climb when the dog has SURVEYED most of
    // the floor (absolute see-based coverage). The boustrophedon sweep drives
    // the dog back and forth so the lidar scans the whole floor; climb once
    // ~85% of the floor plane is seen.
    const double floor_cov = coverage_;
    const bool cov_ok = floor_cov >= coverage_threshold_;
    if (cov_ok) {
      if (coverage_hold_start_.nanoseconds() == 0) coverage_hold_start_ = now();
    } else {
      coverage_hold_start_ = rclcpp::Time();
    }
    const bool cov_held = cov_ok &&
        (now() - coverage_hold_start_).seconds() >= coverage_hold_sec_;
    const bool time_up = (now() - floor_start_time_).seconds() >= explore_time_sec_;
    if ((cov_held || time_up) && (now() - first_odom_time_).seconds() >= start_delay_) {
      startClimb(cov_held);
    }
  }

  void publishGoal(double x, double y, double z, double yaw, const std::string& label) {
    PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "world";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.orientation.w = std::cos(yaw / 2.0);
    target_pub_->publish(goal);
    sent_ = true;
    sent_time_ = now();
    RCLCPP_INFO(get_logger(), "Sent target [%s]: (%.2f, %.2f, %.2f) yaw %.2f", label.c_str(), x,
                y, z, yaw);
  }

  void onReached() {
    if (done_ || !sent_) return;
    RCLCPP_INFO(get_logger(), "Reached %s", layered_ ? "climb stage" : "target");
    if (layered_) {
      if (approach_phase_) {
        // Advance to the next horizontal approach hop, or hand the vertical
        // climb to the manual-waypoint follower once the dog is at the base.
        approach_idx_++;
        if (approach_idx_ < approach_x_.size()) {
          publishGoal(approach_x_[approach_idx_], approach_y_[approach_idx_],
                      approach_z_[approach_idx_], 0.0, "approach");
        } else {
          approach_phase_ = false;
          sent_ = false;
          publishManualPath();
        }
      } else {
        advanceClimb();
      }
    } else {
      advanceGoal();
    }
  }

  void advanceGoal() {
    sent_ = false;
    idx_++;
    if (idx_ >= n_) {
      done_ = true;
      RCLCPP_INFO(get_logger(), "ALL %zu TARGETS DONE", n_);
      return;
    }
    publishGoal(target_x_[idx_], target_y_[idx_], target_z_[idx_], target_yaw_[idx_],
                idx_ < target_floor_.size() ? target_floor_[idx_] : "?");
  }

  // ---- layered exploration ----
  void publishFloorHeight(size_t layer) {
    if (layer >= floor_height_.size()) {
      RCLCPP_WARN(get_logger(), "No floor_height for layer %zu (have %zu)", layer,
                  floor_height_.size());
      return;
    }
    std_msgs::msg::Float64 msg;
    msg.data = floor_height_[layer];
    floor_height_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Publish floor height [layer %zu]: %.2f (RACER exploration z locked)",
                layer, floor_height_[layer]);
  }

  void publishLidarPitch(double pitch) {
    std_msgs::msg::Float64 msg;
    msg.data = pitch;
    lidar_pitch_pub_->publish(msg);
  }

  void publishManualClimbState(bool active) {
    std_msgs::msg::Bool msg;
    msg.data = active;
    manual_climb_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "RACER floor exploration %s for manual climb",
                active ? "paused" : "resumed");
  }

  void startClimb(bool cov_triggered) {
    if (group_idx_ >= climb_groups_.size()) {
      // Final floor reached and explored.
      done_ = true;
      RCLCPP_INFO(get_logger(),
                  "ALL %zu FLOORS EXPLORED (final floor coverage %.1f%% reached)", climb_groups_.size() + 1,
                  coverage_);
      return;
    }
    climbing_ = true;
    publishLidarPitch(-20.0);  // tilt the lidar UP to map the ramp face
    group_start_ = climb_idx_;
    climb_start_time_ = now();
    RCLCPP_INFO(get_logger(),
                "Trigger=%s (coverage %.1f%% / threshold %.1f%%): climbing group %zu/%zu (%ld targets)",
                cov_triggered ? "coverage" : "time", coverage_, coverage_threshold_,
                group_idx_ + 1, climb_groups_.size(), climb_groups_[group_idx_]);
    if (manual_climb_) {
      // RACER plans only the horizontal transfer from the exploration endpoint
      // to the real ramp base.  Once there, the existing hand-authored straight
      // ramp path performs the climb.
      const int gsz = group_idx_ < climb_groups_.size() ? climb_groups_[group_idx_] : 0;
      if (gsz > 0 && group_start_ < climb_x_.size()) {
        const double d =
            std::hypot(dog_x_ - climb_x_[group_start_], dog_y_ - climb_y_[group_start_]);
        if (d > approach_thresh_) {
          generateManualApproach();
          if (!approach_x_.empty()) {
            approach_phase_ = true;
            approach_idx_ = 0;
            publishGoal(approach_x_[0], approach_y_[0], approach_z_[0], 0.0, "approach");
            return;
          }
        }
      }
      approach_phase_ = false;
      publishManualPath();
    } else {
      generateApproach();
      publishClimbTarget();
    }
  }

  // Give RACER the actual ramp-base pose as one goal.  RACER's A* chooses the
  // obstacle-avoiding route; chord-interpolated intermediate goals are unsafe
  // because an interpolated point itself can lie inside a pillar or wall.
  void generateManualApproach() {
    approach_x_.clear();
    approach_y_.clear();
    approach_z_.clear();
    approach_idx_ = 0;
    if (group_start_ >= climb_x_.size()) return;
    const double tx = climb_x_[group_start_], ty = climb_y_[group_start_];
    const double z_approach =
        group_idx_ < floor_height_.size() ? floor_height_[group_idx_] : dog_z_;
    approach_x_.push_back(tx);
    approach_y_.push_back(ty);
    approach_z_.push_back(z_approach);
    RCLCPP_INFO(get_logger(),
                "RACER approach: plan from (%.1f, %.1f) to ramp base (%.1f, %.1f)",
                dog_x_, dog_y_, tx, ty);
  }

  // Route B: publish the whole climb as ONE Path (dog's current position -> the
  // climb targets of the current group). open_loop follows it directly (no RACER
  // A* planning), so the dog traces the hand-authored ramp waypoints exactly.
  void publishManualPath() {
    // Keep RACER active throughout the obstacle-avoiding approach.  Pause it
    // only at the hand-off point so it cannot cancel the approach goal before
    // the dog reaches the ramp base.
    publishManualClimbState(true);
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "world";
    auto add = [&path](double x, double y, double z) {
      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.position.z = z;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    };
    const int gsz = group_idx_ < climb_groups_.size() ? climb_groups_[group_idx_] : 0;
    // Smooth curve from the dog's current pose to the first climb target (the
    // ramp/stairs base) instead of a sharp straight line across the whole floor.
    // A quadratic Bezier with a control point offset perpendicular to the chord
    // bends the path gently (still open-loop, but visually smooth).
    const bool has_target = gsz > 0 && group_start_ < climb_x_.size();
    if (has_target) {
      const double sx = dog_x_, sy = dog_y_, sz = dog_z_;
      const double ex = climb_x_[group_start_], ey = climb_y_[group_start_],
                   ez = climb_z_[group_start_];
      const double dx = ex - sx, dy = ey - sy;
      const double len = std::hypot(dx, dy);
      add(sx, sy, sz);  // t = 0 (dog pose)
      if (len > 1.0) {
        const double px = -dy / len, py = dx / len;  // left-perpendicular unit vec
        const double off = 0.15 * len;               // gentle bend (15% of chord)
        const double cx = 0.5 * (sx + ex) + px * off;
        const double cy = 0.5 * (sy + ey) + py * off;
        const double cz = 0.5 * (sz + ez);
        for (double t : {0.2, 0.4, 0.6, 0.8}) {
          const double a = (1 - t) * (1 - t), b = 2 * (1 - t) * t, c = t * t;
          add(a * sx + b * cx + c * ex, a * sy + b * cy + c * ey,
              a * sz + b * cz + c * ez);
        }
      }
    } else {
      add(dog_x_, dog_y_, dog_z_);
    }
    for (int i = 0; i < gsz; ++i) {
      const size_t k = group_start_ + i;
      if (k >= climb_x_.size()) break;
      add(climb_x_[k], climb_y_[k], climb_z_[k]);
    }
    // The climb targets stop at the ramp/stairs TOP; the dog can stall on the
    // last short hop onto the next floor's plane. Append one more waypoint that
    // carries it onto the flat floor (same x/y as the last target, at the next
    // floor's body height), so it fully lands before switching back to RACER.
    if (gsz > 0 && group_idx_ + 1 < floor_height_.size()) {
      const size_t last = group_start_ + gsz - 1;
      if (last < climb_x_.size()) {
        add(climb_x_[last], climb_y_[last], floor_height_[group_idx_ + 1]);
      }
    }
    // Enable path-following FIRST, then publish the path, so open_loop's
    // pathCallback (which checks path_follow_enabled_) does not drop the path
    // because the toggle message arrived after it.
    std_msgs::msg::Bool on;
    on.data = true;
    path_follow_pub_->publish(on);
    manual_path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Manual climb path: %zu waypoints (dog -> %d climb targets + landing)",
                path.poses.size(), gsz);
  }

  // Build a dynamic approach: hop points from the dog's CURRENT odom position
  // to the first fixed climb target of the current group, spaced
  // climb_hop_dist_ apart. The non-optimistic A* only walks KNOWN-FREE cells,
  // so a far fixed target is unreachable after exploration moved the dog away;
  // these short hops let the dog map-as-it-goes back to the ramp/stairs.
  void generateApproach() {
    approach_x_.clear();
    approach_y_.clear();
    approach_z_.clear();
    approach_idx_ = 0;
    if (climb_idx_ >= climb_x_.size()) return;
    const double tx = climb_x_[climb_idx_], ty = climb_y_[climb_idx_], tz = climb_z_[climb_idx_];
    const double dx = tx - dog_x_, dy = ty - dog_y_, dz = tz - dog_z_;
    const double dist = std::hypot(dx, dy);
    if (dist <= climb_hop_dist_) return;  // already close; no approach hops needed
    const int nhops = static_cast<int>(std::ceil(dist / climb_hop_dist_));
    for (int i = 1; i <= nhops; ++i) {
      const double t = static_cast<double>(i) / nhops;
      approach_x_.push_back(dog_x_ + dx * t);
      approach_y_.push_back(dog_y_ + dy * t);
      approach_z_.push_back(dog_z_ + dz * t);
    }
    RCLCPP_INFO(get_logger(),
                "Dynamic approach: %d hops of ~%.1f m from (%.1f,%.1f) to (%.1f,%.1f)",
                nhops, climb_hop_dist_, dog_x_, dog_y_, tx, ty);
  }

  void publishClimbTarget() {
    if (approach_idx_ < approach_x_.size()) {
      publishGoal(approach_x_[approach_idx_], approach_y_[approach_idx_],
                  approach_z_[approach_idx_], 0.0, "approach");
      return;
    }
    if (climb_idx_ >= climb_x_.size()) {
      finishClimb();
      return;
    }
    publishGoal(climb_x_[climb_idx_], climb_y_[climb_idx_], climb_z_[climb_idx_],
                climb_idx_ < climb_yaw_.size() ? climb_yaw_[climb_idx_] : 0.0,
                climb_idx_ < climb_floor_.size() ? climb_floor_[climb_idx_] : "climb");
  }

  void advanceClimb() {
    sent_ = false;
    if (approach_idx_ < approach_x_.size()) {
      approach_idx_++;
      publishClimbTarget();
      return;
    }
    climb_idx_++;
    const int group_size = group_idx_ < climb_groups_.size() ? climb_groups_[group_idx_] : 0;
    if (climb_idx_ - group_start_ >= group_size) {
      finishClimb();
      return;
    }
    publishClimbTarget();
  }

  void finishClimb() {
    climbing_ = false;
    publishLidarPitch(0.0);  // back to horizontal for floor exploration
    if (manual_climb_) {
      std_msgs::msg::Bool off;
      off.data = false;
      path_follow_pub_->publish(off);  // resume B-spline following (exploration)
      // Manual mode never calls advanceClimb(), so climb_idx_ must be bumped
      // here to the next group's start. Otherwise group_start_ (= climb_idx_)
      // stays 0 and every group re-uses group 1's targets (the dog keeps
      // climbing the west ramp instead of the next connection).
      const int gsz = group_idx_ < climb_groups_.size() ? climb_groups_[group_idx_] : 0;
      climb_idx_ += gsz;
    }
    group_idx_++;
    coverage_hold_start_ = rclcpp::Time();
    floor_start_time_ = now();  // reset: exploration timer for the new floor
    floor_start_coverage_ = coverage_;  // baseline: coverage delta for this floor
    // Lock RACER's exploration z to the newly-reached floor's height.
    publishFloorHeight(group_idx_);
    // Resume only after the new floor height has been published. RACER uses
    // this edge to discard the pre-climb trajectory, re-localize from odometry,
    // reset the per-floor sweep, and plan a fresh trajectory on the landing.
    if (manual_climb_) {
      resume_pending_ = true;
      resume_pending_time_ = now();
    }
    if (group_idx_ >= climb_groups_.size()) {
      // Last climb done: explore the final floor, then startClimb() finishes.
      RCLCPP_INFO(get_logger(),
                  "Final climb done; exploring final floor (waiting for coverage >= %.1f%%)",
                  coverage_threshold_);
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Climb complete; exploring next floor (waiting for coverage >= %.1f%%)",
                coverage_threshold_);
  }

  std::vector<double> target_x_, target_y_, target_z_, target_yaw_;
  std::vector<std::string> target_floor_;
  std::string target_topic_, reached_topic_;
  double start_delay_ = 3.0;
  double target_timeout_ = 90.0;

  // Layered exploration state.
  bool layered_ = false;
  double coverage_threshold_ = 40.0;
  double coverage_hold_sec_ = 5.0;
  double explore_time_sec_ = 60.0;
  double coverage_ = 0.0;
  double floor_start_coverage_ = 0.0;  // coverage at the start of the current floor
  rclcpp::Time coverage_hold_start_;
  rclcpp::Time floor_start_time_;
  rclcpp::Time last_floor_pub_time_;
  std::vector<double> climb_x_, climb_y_, climb_z_, climb_yaw_;
  std::vector<std::string> climb_floor_;
  std::vector<long int> climb_groups_;
  double climb_hop_dist_ = 3.0;
  double approach_thresh_ = 2.0;
  double approach_timeout_ = 30.0;
  double climb_reach_xy_ = 0.25;
  double climb_reach_z_ = 0.08;
  double resume_delay_sec_ = 0.5;
  bool resume_pending_ = false;
  rclcpp::Time resume_pending_time_;
  bool approach_phase_ = false;  // Route B: A*-returning to the ramp/stairs base
  double dog_x_ = 0.0, dog_y_ = 0.0, dog_z_ = 0.0;
  std::vector<double> approach_x_, approach_y_, approach_z_;
  size_t approach_idx_ = 0;
  std::vector<double> floor_height_;
  std::string floor_height_topic_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr floor_height_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lidar_pitch_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr manual_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_follow_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_climb_pub_;
  bool manual_climb_ = true;
  size_t climb_idx_ = 0;      // global index into the concatenated climb targets
  size_t group_idx_ = 0;      // which climb transition we are on
  size_t group_start_ = 0;    // climb_idx_ where the current group starts
  bool climbing_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr coverage_sub_;
  size_t n_ = 0;
  size_t idx_ = 0;
  bool sent_ = false;
  bool done_ = false;
  bool have_odom_ = false;
  rclcpp::Time first_odom_time_;
  rclcpp::Time sent_time_;
  rclcpp::Time climb_start_time_;

  rclcpp::Publisher<PoseStamped>::SharedPtr target_pub_;
  rclcpp::Subscription<Empty>::SharedPtr reached_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskNode>());
  rclcpp::shutdown();
  return 0;
}
