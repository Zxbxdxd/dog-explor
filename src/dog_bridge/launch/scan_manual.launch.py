"""Manual-waypoint multi-floor exploration (SCAN only, no RACER).

Publishes a hand-authored 3D waypoint route (reference_path_publisher) that SCAN
(navi_mode=3) follows through the building (L0 -> west ramp -> L1 -> east -> L2).
The dog maps as it goes. open_loop_controller follows SCAN's B-spline exactly,
so the z follows the manual route (no optimistic A*, no flying through floors).
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def install_root(pkg):
    return get_package_prefix(pkg)

def exe(root, pkg, name):
    return os.path.join(root, "lib", pkg, name)

SCAN_ROOT = install_root("scan_planner")
SCAN_INSTALL = os.path.dirname(get_package_prefix("scan_planner"))
def scan_exe(pkg, name):
    return os.path.join(SCAN_INSTALL, pkg, "lib", pkg, name)

SCAN_SHARE = get_package_share_directory("scan_planner")
DOG_ROOT = install_root("dog_bridge")
GO2_SHARE = get_package_share_directory("go2_description")

planner_yaml = os.path.join(SCAN_SHARE, "config", "planner.yaml")
controllers_yaml = os.path.join(SCAN_SHARE, "config", "controllers.yaml")
simulator_yaml = os.path.join(SCAN_SHARE, "config", "simulator.yaml")
go2_xacro = os.path.join(GO2_SHARE, "xacro", "robot.xacro")

MULTI_FLOOR_PCD = "/home/sigma/dog_racer/SCAN-Planner-Ros2/map.pcd"
REFERENCE_PATH_YAML = os.path.join(DOG_ROOT, "share", "dog_bridge", "config",
                                   "reference_path_multifloor.yaml")

def generate_launch_description():
    rviz_enabled = LaunchConfiguration("rviz")
    use_sim_time = False

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),

        # ---------------- execution: map + lidar renderer ----------------
        Node(executable=scan_exe("map_generator", "map_pub"),
             name="map_pub", namespace="map_generator", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "file_name": MULTI_FLOOR_PCD,
                          "frame_id": "world",
                          "publish_rate": 0.2,
                          "downsample_res": 0.1}]),
        Node(executable=scan_exe("local_sensing_node", "pcl_render_node"),
             name="pcl_render_node", output="screen",
             parameters=[simulator_yaml,
                         {"use_sim_time": use_sim_time,
                          "sensor_type": "lidar",
                          "body_pose_topic": "body_pose",
                          "map.x_size": 40.0, "map.y_size": 40.0, "map.z_size": 8.0,
                          "use_global_map_topic": True,
                          "pcd_map_file": MULTI_FLOOR_PCD,
                          "lidar_pitch": 0.0}],
             remappings=[("global_map", "/map_generator/global_cloud"),
                         ("body_pose", "/quad_0/body_pose"),
                         ("cloud", "/quad_0/cloud"),
                         ("sensor_cloud", "/quad_0/sensor_cloud"),
                         ("depth", "/quad_0/depth"),
                         ("dyn_cloud", "/quad_0/dyn_cloud"),
                         ("uav_cloud", "/quad_0/uav_cloud")]),
        Node(executable=scan_exe("odom_visualization", "odom_visualization"),
             name="odom_visualization", output="screen",
             parameters=[simulator_yaml, {"use_sim_time": use_sim_time}],
             remappings=[("body_pose", "/quad_0/body_pose"),
                         ("pose", "/quad_0/pose"),
                         ("path", "/quad_0/path"),
                         ("velocity", "/quad_0/velocity"),
                         ("trajectory", "/quad_0/trajectory"),
                         ("robot", "/quad_0/robot"),
                         ("height", "/quad_0/height")]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="go2_robot_state_publisher", output="screen",
             parameters=[{"use_sim_time": use_sim_time},
                         {"robot_description": Command(
                             ["xacro ", go2_xacro, " use_gazebo:=false"])}]),

        # ---------------- SCAN planner (mode 3: follow reference path) ------
        Node(executable=exe(SCAN_ROOT, "scan_planner", "scan_planner_node"),
             name="scan_planner_node", output="screen",
             parameters=[planner_yaml,
                         {"use_sim_time": use_sim_time,
                          "fsm.navi_mode": 3,
                          "grid_map.sensor_type": "lidar",
                          "grid_map.cloud_is_world": True,
                          "grid_map.need_extrinsic": False}],
             remappings=[("body_pose", "/quad_0/body_pose"),
                         ("sensor_pose", "/quad_0/lidar_pose"),
                         ("cloud", "/quad_0/cloud"),
                         ("depth", "/quad_0/depth"),
                         ("move_base_simple/goal", "/move_base_simple/goal"),
                         ("initial_path", "/initial_path"),
                         ("planning/bspline", "/planning/bspline")]),

        # ---------------- open-loop follower (3D, z follows the route) -----
        Node(executable=exe(SCAN_ROOT, "scan_planner", "open_loop_controller"),
             name="open_loop_controller", output="screen",
             parameters=[controllers_yaml,
                         {"use_sim_time": use_sim_time,
                          "init_x": -5.5, "init_y": 5.5, "init_z": 0.5,
                          "enable_path_follow": True}],
             remappings=[("planning/bspline", "/planning/bspline"),
                         ("body_pose", "/quad_0/body_pose")]),

        # ---------------- accumulated map (mapping visual) -------------------
        Node(executable=exe(DOG_ROOT, "dog_bridge", "cloud_accumulator"),
             name="cloud_accumulator", output="screen",
             parameters=[{"max_points": 1000000}],
             remappings=[("cloud", "/quad_0/cloud"),
                         ("accumulated_cloud", "/map_ros/accumulated_cloud")]),

        # ---------------- manual waypoint publisher ------------------------
        Node(package="scan_planner", executable="reference_path_publisher.py",
             name="reference_path_publisher", output="screen",
             parameters=[REFERENCE_PATH_YAML, {"use_sim_time": use_sim_time}],
             remappings=[("body_pose", "/quad_0/body_pose"),
                         ("initial_path", "/initial_path")]),

        # ---------------- RViz ----------------
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", os.path.join(DOG_ROOT, "share", "dog_bridge", "launch", "plan_b.rviz")],
             condition=IfCondition(rviz_enabled)),
    ])
