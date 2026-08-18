"""Plan B: RACER decision (exploration FSM) + SCAN execution (quadruped stack).

Runs both stacks in one launch. RACER-ROS2 and SCAN-Planner-Ros2 share several
package names (plan_env, path_searching, ...) so they cannot be in one colcon
workspace; here we resolve every executable by ABSOLUTE path anchored on a
unique package (exploration_manager / scan_planner / dog_bridge), which is
immune to duplicate-package lookup ambiguity.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def install_root(pkg):
    return get_package_prefix(pkg)


def exe(root, pkg, name):
    return os.path.join(root, "lib", pkg, name)


RACER_ROOT = install_root("exploration_manager")
SCAN_ROOT = install_root("scan_planner")
DOG_ROOT = install_root("dog_bridge")
# Shared-name packages (map_generator, local_sensing_node, odom_visualization)
# exist in BOTH workspaces, so anchor them on SCAN's install dir explicitly.
SCAN_INSTALL = os.path.dirname(get_package_prefix("scan_planner"))
def scan_exe(pkg, name):
    return os.path.join(SCAN_INSTALL, pkg, "lib", pkg, name)
LKH_PREFIX = get_package_prefix("lkh_mtsp_solver")
LKH_RESOURCE = os.path.join(LKH_PREFIX, "share", "lkh_mtsp_solver", "resource")

SCAN_SHARE = get_package_share_directory("scan_planner")
GO2_SHARE = get_package_share_directory("go2_description")

planner_yaml = os.path.join(SCAN_SHARE, "config", "planner.yaml")
controllers_yaml = os.path.join(SCAN_SHARE, "config", "controllers.yaml")
simulator_yaml = os.path.join(SCAN_SHARE, "config", "simulator.yaml")
go2_xacro = os.path.join(GO2_SHARE, "xacro", "robot.xacro")

# Reuse RACER's tuned ground-mode planner params (import from the installed
# launch module; module-level code only defines functions, safe to load).
racer_share = get_package_share_directory("exploration_manager")
_racer_spec = importlib.util.spec_from_file_location(
    "racer_dog_params", os.path.join(racer_share, "launch", "racer_dog.launch.py"))
_racer_mod = importlib.util.module_from_spec(_racer_spec)
_racer_spec.loader.exec_module(_racer_mod)
planner_params = _racer_mod.planner_params


def _racer_params(coverage_metrics, coverage_report_interval_sec, show_grid_text, goal_mode, multi_floor, layered):
    p = planner_params(coverage_metrics, coverage_report_interval_sec, show_grid_text)
    # The single-floor RACER preset allocates only 3.5 m in Z.  Merely raising
    # box_max_z does not resize that backing voxel buffer: L1 sits on its upper
    # boundary and L2/L3 are outside it, so the dog can execute trajectories but
    # their floors, pillars and stairs are never fused into the map.  Allocate
    # the full custom-building height (PCD top is ~11.4 m) in multi-floor mode.
    p["sdf_map.map_size_z"] = PythonExpression(
        ["12.0 if '", multi_floor, "' == 'true' else 3.5"])
    p["map_ros.visualization_truncate_height"] = PythonExpression(
        ["12.0 if '", multi_floor, "' == 'true' else 10.09"])
    # Ground-robot lidar range = "survey" radius. Exploration is defined as
    # "lidar scans a column -> it is added to the obstacle map -> explored".
    # 4.5 m is long enough that the 90-deg forward cone's frontier arc is a
    # healthy size (~70 cells), but still short enough the dog has to walk each
    # region (not see the whole floor from one spot).
    p["sdf_map.max_ray_length"] = 4.5
    # Ground mode: inflate walls/pillars only in XY. Full 3D inflation raises
    # the floor into the dog's z=0.5 body trajectory and makes every open-space
    # path look colliding to the online safety monitor.
    p["sdf_map.planar_inflation"] = True
    # Optimistic A* (UNKNOWN treated as free) so the dog can route to far
    # frontiers across the still-unmapped floor, exactly like the original
    # drone RACER. Vertical safety is preserved by the z-lock: planar search
    # pins the planned path to the current floor's height, so the dog never
    # rises to other floors. Horizontal "flying through" an unmapped pillar is
    # accepted (the custom map has no maze walls, only sparse pillars, so a
    # straight route rarely clips one and the collision check replans if it does).
    p["exploration.init_plan_num"] = 1000000
    # The geometric A* budget (racer_dog.launch.py leaves 1 ms) is too short for
    # the whole-building multi-floor map (19x24x8 m at 0.3 m coarse resolution):
    # a far viewpoint times out -> "No path" -> the dog never moves (stuck at
    # spawn). With a full-ring 360 lidar the dog sees far frontiers all around,
    # so give it a generous budget (1 s) so it can route to them instead of
    # spamming "No path" near the boundary.
    p["astar.max_search_time"] = 1.0
    # Longer trajectory + farther search horizon so the dog walks a longer path
    # and does not stop-and-go (finish a short path and wait for the next plan).
    p["manager.local_segment_length"] = 12.0
    p["search.horizon"] = 8.0
    # A horizon endpoint is useful only if it is measurably closer to the
    # requested viewpoint. This also constrains the relaxed second kino search,
    # which otherwise accepts a dynamically feasible branch on the wrong side.
    p["search.horizon_min_goal_progress"] = 0.25
    # Match the faster dog: raise RACER's velocity/accel limits to the SCAN
    # controller's new max (1.0 m/s) so planned trajectories stay feasible.
    for k in ("exploration.vm", "exploration.am", "search.max_vel", "search.max_acc",
              "optimization.max_vel", "optimization.max_acc",
              "bspline.limit_vel", "bspline.limit_acc", "manager.max_vel",
              "manager.max_acc", "exploration.yd"):
        p[k] = 1.0
    # Ground robot: the drone's MINTIME cost (manager.min_time=True in
    # racer_dog.launch.py) shrinks the B-spline to the yaw-change lower bound
    # (~0.97 s) regardless of path length, so the dog "fully executes" a ~0.5 m
    # stub and replans forever. Disable it so the trajectory obeys max_vel
    # (duration = path length / max_vel) and actually reaches the viewpoint.
    p["manager.min_time"] = False
    # Keep the dog's original (working) single-layer frontier thresholds. The
    # frontier-tour cost is fixed separately (straight-line drone->frontier
    # approximation in getSwarmCostMatrix), so we do NOT coarsen the clusters
    # here — coarsening (cluster_min=100) pushed viewpoints far apart and
    # re-introduced "No path to next viewpoint".
    p["partitioning.min_unknown"] = 300
    p["partitioning.min_frontier"] = 5
    p["partitioning.min_free"] = 300
    # Per-floor voxel-count band (half-height around the current floor body
    # height). Counts unknown/free only on the floor being explored instead of
    # the whole 9.5 m column. TUNE to ~half your floor height; 0 disables.
    p["partitioning.z_band"] = 1.5
    # Frontier clustering + viewpoint visibility, sized for the SHORT 3 m lidar
    # and the 90-deg forward cone. The forward cone's frontier arc is only ~47
    # cells, so cluster_min must stay below that (a 50-cell threshold discarded
    # the whole arc -> no frontier). 20 cells keeps the forward arc while still
    # dropping the ~5-10 cell "reappeared edge" fragments.
    p["frontier.cluster_min"] = 20
    p["frontier.min_visib_num"] = 5
    # Replan less often: RACER alternates between equal-cost frontiers every
    # 1.5 s (default thresh_replan3), which the streaming bridge forwards as
    # oscillation. A longer replan period lets the dog commit to each frontier
    # and stops it from whipping between the east/west viewpoints (the "turn
    # around and wander to the back" symptom).
    p["fsm.thresh_replan3"] = 25.0
    p["fsm.replan_time"] = 0.2
    # The "cluster covered" replan (thresh_replan2) fires once the current
    # frontier cluster is covered; the default 0.2 s lets the lidar (6 m) mark
    # a cluster covered almost immediately so the dog replans without moving.
    # Require it to execute ~3 s first so it actually drives toward the cluster
    # (fewer "cluster covered" replans -> less flash-back-and-rewalk).
    p["fsm.thresh_replan2"] = 3.0
    # Goal-driven mode (Step 1): RACER plans directly to /plan_b/target with no
    # frontier selection. thresh_replan1 enables a near-end replan so the dog
    # closes the last gap to the target; goal_mode itself is resolved at launch.
    p["fsm.goal_mode"] = PythonExpression(["'", goal_mode, "' == 'true'"])
    p["fsm.goal_reach_thresh"] = 0.8
    # Keep the short threshold for frontier/goal hops. Sweep trajectories use a
    # separate lead time below so the next B-spline can be planned and stitched
    # before the current one ends, without reintroducing immediate frontier
    # replans on short trajectories.
    p["fsm.thresh_replan1"] = 0.2
    # Stream the next sweep B-spline after 75% of the current one has executed.
    # The FSM samples the current spline at the new start time and transfers
    # position, velocity and acceleration as boundary conditions.
    p["fsm.sweep_replan_ratio"] = 0.75
    # If newly observed occupancy is already very close, start the detour from
    # measured odom; otherwise preserve the executing trajectory's velocity for
    # a continuous avoidance manoeuvre.
    p["fsm.collision_stop_distance"] = 0.6
    # Layered exploration (Step 3): RACER explores each floor (frontier mode)
    # and a /plan_b/target interrupts it to climb to the next floor's ramp
    # entrance, then exploration resumes. Works with goal_mode=false.
    p["fsm.interrupt_goal"] = PythonExpression(["'", layered, "' == 'true'"])
    # A ground robot's lidar only maps the band it operates in (z < ~0.5 m);
    # the cells above are unreachable and irrelevant. Restrict the search and
    # coverage box to that band so the coverage metric reflects the actually
    # explorable volume (a horizontal lidar cannot map the upper layers, which
    # previously capped coverage near 50%). Multi-floor mode (Step 2) instead
    # maps the whole building height so ramps/stairs between floors are sensed.
    p["sdf_map.box_max_z"] = PythonExpression(["9.5 if '", multi_floor, "' == 'true' else 0.5"])
    # Multi-floor: box tightly bounds the custom 4-floor building (x -6..10,
    # y -6..6, top floor z~4.5). Flat demo keeps the tuned west-half box.
    # Multi-floor box stays INSIDE the building footprint (custom_map.pcd spans
    # x[-9,15] y[-9,9]); the old x[-9.2,15.2] overhung 0.2 m of dead space past
    # the boundary walls, which the coverage metric could never observe (a fixed
    # ~4% coverage ceiling) and which the frontier detector chased as unknown.
    p["sdf_map.box_min_x"] = PythonExpression(["-9.0 if '", multi_floor, "' == 'true' else -7.0"])
    p["sdf_map.box_max_x"] = PythonExpression(["15.0 if '", multi_floor, "' == 'true' else 7.0"])
    p["sdf_map.box_min_y"] = PythonExpression(["-9.0 if '", multi_floor, "' == 'true' else -15.0"])
    p["sdf_map.box_max_y"] = PythonExpression(["9.0 if '", multi_floor, "' == 'true' else 15.0"])
    p["sdf_map.box_min_z"] = PythonExpression(["0.0 if '", multi_floor, "' == 'true' else 0.0"])
    # Multi-floor: let the B-spline optimizer move in z (ramp climbing).
    p["optimization.planar"] = PythonExpression(["False if '", multi_floor, "' == 'true' else True"])
    p["exploration.multi_floor"] = PythonExpression(["'", multi_floor, "' == 'true'"])
    # Layered exploration: the dog starts at the ramp base (z=0.5); exploration
    # on each floor locks to this height (updated to the climb-goal z after each
    # transition). Flat mode keeps the body-height default.
    p["exploration.explore_height"] = PythonExpression(["0.5 if '", multi_floor, "' == 'true' else 0.4"])
    # Layered mode uses one continuous, floor-wide lawn-mower sweep. Splitting
    # the floor into 4x4 blocks created needless U-turns at every block boundary
    # (perceived as "乱转") and left gaps while changing blocks. A 1 m boundary
    # margin keeps endpoints away from the open slab edge; 3 m row spacing still
    # overlaps the 4.5 m lidar range and reaches >80% floor coverage efficiently.
    p["exploration.sweep_mode"] = PythonExpression(["'", layered, "' == 'true'"])
    p["exploration.sweep_step"] = 3.0          # spacing between sweep lines (m)
    p["exploration.sweep_reach_thresh"] = 0.8  # reach distance to advance (m)
    p["exploration.sweep_min_traj_progress"] = 0.25
    p["exploration.sweep_bad_traj_limit"] = 2
    p["exploration.sweep_blocks_per_side"] = 1
    p["exploration.sweep_margin"] = 1.0
    p["exploration.sweep_max_passes"] = 1
    # The drone tuning allowed the optimizer to trade away its endpoint
    # (ld_end=0.5) for smoothness/obstacle cost.  On the dog that produced valid
    # but reverse B-splines.  Keep the endpoint close to the A* segment goal;
    # the odom progress gate above remains the final safety check.
    p["optimization.ld_end"] = 20.0
    # Place viewpoints close to the frontier (just inside the known-free side).
    # Pushing them 2-3 m into the unknown (the old values) makes the
    # non-optimistic A* (which refuses to step into UNKNOWN) fail with
    # "No path to next viewpoint" and forces the dog to keep re-picking far
    # viewpoints. A short radius keeps every viewpoint reachable.
    p["frontier.candidate_rmin"] = 1.0
    p["frontier.candidate_rmax"] = 1.5
    # A cluster is "covered" once this fraction of its cells are no longer
    # frontiers. 0.2 (the drone default) marks a cluster covered too quickly for
    # a ground robot -> the dog oscillates between equal-cost frontier fragments
    # ("pacing back and forth" between two nearby viewpoints). 0.6 makes it sweep
    # a cluster substantially before moving on (proven anti-oscillation value).
    # NOTE: 0.95 (an over-correction) made "cluster covered" fire 0 times and the
    # frontier list stuck at ~170 -> the dog circled forever instead of diving
    # into one direction. Restored to 0.6.
    p["frontier.min_view_finish_fraction"] = 0.6
    # Anti-oscillation weights for the frontier tour (getSwarmCostMatrix).
    # w_reward: bonus for a frontier with more unknown voxels (cells_.size()),
    # so a LARGER cluster is preferred over a smaller one at a comparable
    # distance (distance stays dominant -> "nearest + larger unknown cluster").
    # w_consistency: cost bonus for the frontier committed last replan, so the
    # dog sticks to a frontier until it is actually consumed (60%) instead of
    # shuttling back and forth between equal-cost frontiers without covering any.
    # w_turn: heading-momentum weight -> penalize frontiers that force a turn
    # away from the current heading (0 ahead, pi = 180-degree reversal). Makes
    # the dog "flow" forward instead of doubling back.
    # consistency_radius: how close (m) a frontier's average must be to the
    # previous viewpoint to count as "the same frontier".
    p["frontier.w_reward"] = 0.08
    p["frontier.w_consistency"] = 6.0
    p["frontier.w_turn"] = 1.5
    p["frontier.consistency_radius"] = 2.0
    return p

# Boxes + a z=0 ground grid over the RACER box region, so the lidar (and
# therefore RACER's map) can cover the whole explorable area, not just the
# box cluster.
GROUND_BOXES_PCD = (
    "/home/sigma/dog_racer/RACER-ROS2/src/map_generator/resource/ground_boxes_ground.pcd")

# Multi-floor building map: custom 4-floor building (L0..L3) with straight
# ramps + pillars, hand-defined so the climb waypoints are exact. Generated by
# /home/sigma/dog_racer/SCAN-Planner-Ros2/generate_custom_map.py.
MULTI_FLOOR_PCD = "/home/sigma/dog_racer/SCAN-Planner-Ros2/custom_map.pcd"

# Step 1 target list: flat-ground demo points within the mapped area
# (see config/targets.yaml for the actual list).
TARGETS_YAML = os.path.join(DOG_ROOT, "share", "dog_bridge", "config", "targets.yaml")

# Step 3 layered-exploration config (coverage threshold + staged climb targets).
LAYERED_TARGETS_YAML = os.path.join(DOG_ROOT, "share", "dog_bridge", "config", "layered_targets.yaml")


def generate_launch_description():
    rviz_enabled = LaunchConfiguration("rviz")
    coverage_metrics = LaunchConfiguration("coverage_metrics")
    coverage_report_interval_sec = LaunchConfiguration("coverage_report_interval_sec")
    show_grid_text = LaunchConfiguration("show_grid_text")
    goal_mode = LaunchConfiguration("goal_mode")
    multi_floor = LaunchConfiguration("multi_floor")
    layered = LaunchConfiguration("layered")
    coverage_threshold = LaunchConfiguration("coverage_threshold")
    coverage_hold_sec = LaunchConfiguration("coverage_hold_sec")
    explore_time_sec = LaunchConfiguration("explore_time_sec")
    use_sim_time = False
    # Map selection: multi-floor building vs flat boxes+ground.
    MAP_PCD = PythonExpression(
        ["'", MULTI_FLOOR_PCD, "' if '", multi_floor, "' == 'true' else '", GROUND_BOXES_PCD, "'"])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("coverage_metrics", default_value="true"),
        DeclareLaunchArgument("coverage_report_interval_sec", default_value="2.0"),
        DeclareLaunchArgument("show_grid_text", default_value="false"),
        # goal_mode=true: RACER plans to each /plan_b/target (no frontier
        # exploration, no circling); task_node drives the target list.
        DeclareLaunchArgument("goal_mode", default_value="false"),
        # multi_floor=true: use the SCAN multi-floor building map and the
        # open_loop controller (3D B-spline follower, z follows ramps) instead
        # of the flat closed-loop + kinematic sim stack.
        DeclareLaunchArgument("multi_floor", default_value="false"),
        # layered=true: RACER explores each floor (frontier mode); task_node
        # triggers a ramp climb when the current floor's coverage reaches the
        # threshold, then exploration resumes on the next floor.
        DeclareLaunchArgument("layered", default_value="false"),
        # Exposed for map-specific tuning and accelerated transition tests.
        DeclareLaunchArgument("coverage_threshold", default_value="85.0"),
        DeclareLaunchArgument("coverage_hold_sec", default_value="5.0"),
        DeclareLaunchArgument("explore_time_sec", default_value="360.0"),

        # ---------------- SCAN execution layer ----------------
        Node(executable=exe(SCAN_ROOT, "scan_planner", "scan_planner_node"),
             name="scan_planner_node", output="screen",
             parameters=[
                 planner_yaml,
                 {"use_sim_time": use_sim_time,
                  "fsm.navi_mode": 3,
                  "grid_map.sensor_type": "lidar",
                  "grid_map.cloud_is_world": True,
                  "grid_map.need_extrinsic": False,
                  # prepareReferenceWaypoints adds body_height to every /initial_path
                  # waypoint z. In multi-floor mode the bridge already forwards the
                  # dog-body z from RACER, so set the offset to 0 to avoid a double
                  # lift; flat mode keeps the default 0.4 (unchanged behaviour).
                  "grid_map.body_height":
                      PythonExpression(["0.0 if '", multi_floor, "' == 'true' else 0.4"]),
                 },
             ],
             remappings=[
                 ("body_pose", "/quad_0/body_pose"),
                 ("sensor_pose", "/quad_0/lidar_pose"),
                 ("cloud", "/quad_0/cloud"),
                 ("depth", "/quad_0/depth"),
                 ("move_base_simple/goal", "/move_base_simple/goal"),
                 # Multi-floor: open_loop_controller follows the bridge's
                 # copy of RACER's B-spline on /planning/bspline, so SCAN's own
                 # local-trajectory output is moved aside; and SCAN stops
                 # consuming /initial_path (it would fail to rebound-replan on
                 # ramps). SCAN still builds the map from the lidar cloud.
                 ("planning/bspline", "/planning/bspline_scan"),
                 ("initial_path", PythonExpression(
                     ["'/initial_path_dummy' if '", multi_floor, "' == 'true' else '/initial_path'"])),
             ]),
        # Multi-floor execution: open_loop_controller evaluates SCAN's 3D B-spline
        # and publishes body_pose WITH z following ramps (original mode3 ramp demo).
        # It replaces closed_loop_controller + go2_kinematic_sim, which only
        # integrate x/y/yaw and cannot climb.
        Node(executable=exe(SCAN_ROOT, "scan_planner", "open_loop_controller"),
             name="open_loop_controller", output="screen",
             parameters=[controllers_yaml,
                         {"use_sim_time": use_sim_time,
                          # Start EAST of the west ramp (x=-5.37) so the first
                          # frontier viewpoint lands on open L0 floor (not on the
                          # ramp face), avoiding the "stuck at the stairway" loop.
                          # The manual climb path starts from the dog's current
                          # odom, so it walks back to the ramp before climbing.
                          "init_x": 1.5, "init_y": 1.5, "init_z": 0.5,
                          "init_yaw": 1.5708,
                          # Route B: the manual climb Path carries the dog's BODY
                          # z already, so no extra body-height offset in the
                          # open-loop follower. enable_path_follow makes the
                          # follower trace /initial_path whenever the task layer
                          # publishes it (the bridge is gated off /initial_path in
                          # multi-floor mode).
                          "body_height": 0.0,
                          "max_vel": 1.0,
                          "resume_xy_tolerance": 1.5,
                          "resume_z_tolerance": 0.4,
                          "enable_path_follow": True}],
             remappings=[
                 ("planning/bspline", "/planning/bspline"),
                 ("body_pose", "/quad_0/body_pose"),
             ],
             condition=IfCondition(multi_floor)),
        Node(executable=exe(SCAN_ROOT, "scan_planner", "closed_loop_controller"),
             name="closed_loop_controller", output="screen",
             parameters=[controllers_yaml,
                         {"use_sim_time": use_sim_time,
                          "max_vx": 1.0, "max_vy": 0.5}],
             remappings=[
                 ("body_pose", "/quad_0/body_pose"),
                 ("cmd_vel", "/quad_0/cmd_vel"),
             ],
             condition=UnlessCondition(multi_floor)),
        Node(executable=exe(SCAN_ROOT, "scan_planner", "go2_kinematic_sim"),
             name="go2_kinematic_sim", output="screen",
             parameters=[controllers_yaml,
                         {"use_sim_time": use_sim_time,
                          "init_x": 0.0, "init_y": 0.0, "init_z": 0.4,
                          "publish_tf": False,
                          "max_vx": 1.0, "max_vy": 0.5}],
             remappings=[
                 ("body_pose", "/quad_0/body_pose"),
                 ("cmd_vel", "/quad_0/cmd_vel"),
             ],
             condition=UnlessCondition(multi_floor)),
        Node(executable=exe(SCAN_ROOT, "scan_planner", "go2_gait_publisher"),
             name="go2_gait_publisher", output="screen",
             parameters=[controllers_yaml, {"use_sim_time": use_sim_time}],
             remappings=[("body_pose", "/quad_0/body_pose")]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="go2_robot_state_publisher", output="screen",
             parameters=[
                 {"use_sim_time": use_sim_time},
                 {"robot_description": Command(
                     ["xacro ", go2_xacro, " use_gazebo:=false"])},
             ]),
        Node(executable=scan_exe("map_generator", "map_pub"),
             name="map_pub", namespace="map_generator", output="screen",
             parameters=[
                 {"use_sim_time": use_sim_time,
                  "file_name": MAP_PCD,
                  "frame_id": "world",
                  "publish_rate": 0.2,
                  "downsample_res": 0.1},
             ]),
        Node(executable=scan_exe("local_sensing_node", "pcl_render_node"),
             name="pcl_render_node", output="screen",
             parameters=[
                 simulator_yaml,
                 {"use_sim_time": use_sim_time,
                  "sensor_type": "lidar",
                  "body_pose_topic": "body_pose",
                  "map.x_size": 40.0,
                  "map.y_size": 40.0,
                  # Multi-floor building reaches z~7.2, so the render box must
                  # cover it (flat demo only needs 5 m). A coarser downsample in
                  # multi-floor also lightens the map the renderer has to load.
                  "map.z_size": PythonExpression(["11.0 if '", multi_floor, "' == 'true' else 5.0"]),
                  "downsample_res": PythonExpression(["0.2 if '", multi_floor, "' == 'true' else 0.1"]),
                  "use_global_map_topic": True,
                  "pcd_map_file": MAP_PCD,
                  # Default lidar_pitch is 45 deg downward (mostly ground).
                  # A horizontal lidar sees much farther so RACER's map (fed
                  # from this cloud) covers more area per sweep. The pitch is
                  # switched dynamically: 0 deg during floor exploration, -20 deg
                  # (up) during the climb, via /plan_b/lidar_pitch.
                  "lidar_pitch": 0.0,
                  # The layered lawn-mower path is deterministic, so it no
                  # longer needs a forward cone to bias frontier selection. A
                  # 360-degree scan fills both sides of every sweep row and is
                  # what makes 85-90% floor coverage reachable before timeout.
                  "is_360lidar": PythonExpression(["1 if '", layered, "' == 'true' else 0"]),
                  "yaw_fov": PythonExpression(["360.0 if '", layered, "' == 'true' else 90.0"]),
                  "vertical_fov": 90.0},
             ],
             remappings=[
                 ("global_map", "/map_generator/global_cloud"),
                 ("body_pose", "/quad_0/body_pose"),
                 ("cloud", "/quad_0/cloud"),
                 ("sensor_cloud", "/quad_0/sensor_cloud"),
                 ("depth", "/quad_0/depth"),
                 ("dyn_cloud", "/quad_0/dyn_cloud"),
                 ("uav_cloud", "/quad_0/uav_cloud"),
             ]),
        Node(executable=scan_exe("odom_visualization", "odom_visualization"),
             name="odom_visualization", output="screen",
             parameters=[simulator_yaml, {"use_sim_time": use_sim_time}],
             remappings=[
                 ("body_pose", "/quad_0/body_pose"),
                 ("pose", "/quad_0/pose"),
                 ("path", "/quad_0/path"),
                 ("velocity", "/quad_0/velocity"),
                 ("trajectory", "/quad_0/trajectory"),
                 ("robot", "/quad_0/robot"),
                 ("height", "/quad_0/height"),
             ]),

        # ---------------- RACER decision layer ----------------
        Node(executable=exe(LKH_PREFIX, "lkh_mtsp_solver", "mtsp_node"),
             name="tsp_solver_1", output="screen",
             parameters=[{
                 "exploration.drone_id": 1,
                 "exploration.mtsp_dir": LKH_RESOURCE,
                 "exploration.problem_id": 1,
             }]),
        Node(executable=exe(LKH_PREFIX, "lkh_mtsp_solver", "mtsp_node"),
             name="acvrp_solver_1", output="screen",
             parameters=[{
                 "exploration.drone_id": 1,
                 "exploration.mtsp_dir": LKH_RESOURCE,
                 "exploration.problem_id": 2,
             }]),
        Node(executable=exe(RACER_ROOT, "exploration_manager", "exploration_node"),
             name="exploration_node_1", output="screen",
             respawn=True,  # RACER has an intermittent startup heap abort; restart it
             parameters=[_racer_params(coverage_metrics,
                                       coverage_report_interval_sec,
                                       show_grid_text,
                                       goal_mode,
                                       multi_floor,
                                       layered)],
             remappings=[
                 ("/odom_world", "/odom_world"),
                 ("/map_ros/pose", "/map_ros/pose"),
                 ("/map_ros/cloud", "/map_ros/cloud"),
                 ("/map_ros/depth", "/map_ros/depth_unused"),
                 ("/move_base_simple/goal", "/plan_b/start"),
                 ("/planning/replan", "/planning/replan_1"),
                 ("/planning/new", "/planning/new_1"),
                 ("/planning/bspline", "/planning/bspline_1"),
                 ("/swarm_expl/drone_state_send", "/swarm_expl/drone_state"),
                 ("/swarm_expl/drone_state_recv", "/swarm_expl/drone_state"),
                 ("/swarm_expl/pair_opt_send", "/swarm_expl/pair_opt"),
                 ("/swarm_expl/pair_opt_recv", "/swarm_expl/pair_opt"),
                 ("/swarm_expl/pair_opt_res_send", "/swarm_expl/pair_opt_res"),
                 ("/swarm_expl/pair_opt_res_recv", "/swarm_expl/pair_opt_res"),
                 ("/swarm_expl/grid_tour_send", "/swarm_expl/grid_tour"),
                 ("/swarm_expl/hgrid_send", "/swarm_expl/hgrid"),
                 ("/multi_map_manager/chunk_stamps_send", "/multi_map_manager/chunk_stamps"),
                 ("/multi_map_manager/chunk_data_send", "/multi_map_manager/chunk_data"),
                 ("/multi_map_manager/chunk_stamps_recv", "/multi_map_manager/chunk_stamps"),
                 ("/multi_map_manager/chunk_data_recv", "/multi_map_manager/chunk_data"),
                 ("/planning/swarm_traj_recv", "/planning/swarm_traj"),
                 ("/planning/swarm_traj_send", "/planning/swarm_traj"),
                 ("/planning_vis/trajectory", "/planning_vis/trajectory_1"),
                 ("/planning_vis/frontier", "/planning_vis/frontier_1"),
                 ("/planning_vis/viewpoints", "/planning_vis/viewpoints_1"),
                 ("/sdf_map/occupancy_all", "/sdf_map/occupancy_all_1"),
                 ("/sdf_map/occupancy_local", "/sdf_map/occupancy_local_1"),
                 ("/sdf_map/occupancy_local_inflate", "/sdf_map/occupancy_local_inflate_1"),
                 ("/sdf_map/unknown", "/sdf_map/unknown_1"),
                 ("/sdf_map/free", "/sdf_map/free_1"),
                 ("/sdf_map/update_range", "/sdf_map/update_range_1"),
                 ("/sdf_map/basecoor", "/swarm_sim_tf/basecoor_1"),
             ]),

        # ---------------- bridge ----------------
        Node(executable=exe(DOG_ROOT, "dog_bridge", "dog_bridge_node"),
             name="dog_bridge", output="screen",
             parameters=[{
                 "body_height": 0.4,
                 "path_spacing": 0.3,
                 "path_resend_dist": 1.5,
                 "path_interval": 5.0,
                 "lidar_pose_topic": "/quad_0/lidar_pose",
                 "start_delay": 3.0,
                 "start_retry_period": 1.0,
                 "max_start_attempts": 10,
                 "stuck_timeout": 12.0,
                 "stuck_min_move": 0.3,
                 # Boustrophedon sweep box = the multi-floor building interior
                 # (sdf_map box x[-9,15] y[-9,9] with a small wall margin). Step
                 # < lidar range (3 m) so adjacent rows overlap and the whole
                 # floor gets surveyed without gaps.
                 "sweep_x0": -8.0,
                 "sweep_x1": 14.0,
                 "sweep_y0": -8.0,
                 "sweep_y1": 8.0,
                 "sweep_step": 2.5,
                 "frame_id": "world",
                 "goal_mode": PythonExpression(["'", goal_mode, "' == 'true'"]),
                 "multi_floor": PythonExpression(["'", multi_floor, "' == 'true'"]),
             }]),

        # ---------------- task layer (goal-driven / layered-exploration) -----
        Node(package="dog_bridge", executable="task_node",
             name="task_node", output="screen",
             parameters=[PythonExpression(
                 ["'", LAYERED_TARGETS_YAML, "' if '", layered,
                  "' == 'true' else '", TARGETS_YAML, "'"]),
                 {"target_topic": "/plan_b/target",
                  "reached_topic": "/plan_b/reached",
                  "body_pose_topic": "/quad_0/body_pose",
                  "coverage_threshold": coverage_threshold,
                  "coverage_hold_sec": coverage_hold_sec,
                  "explore_time_sec": explore_time_sec}],
             condition=IfCondition(PythonExpression(
                 ["'true' if ('", goal_mode, "' == 'true') or ('", layered,
                  "' == 'true') else 'false'"]))),

        # ---------------- RViz ----------------
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", os.path.join(DOG_ROOT, "share", "dog_bridge", "launch", "plan_b.rviz")],
             condition=IfCondition(rviz_enabled)),
    ])
