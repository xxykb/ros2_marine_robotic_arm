#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("drone_x", default_value="5.0"),
        DeclareLaunchArgument("drone_y", default_value="0.0"),
        DeclareLaunchArgument("drone_z", default_value="4.0"),
    ]

    world_path = os.path.join(get_package_share_directory("kuka_landing_control"), "worlds", "marine_world.sdf")

    install_prefix = os.path.dirname(os.path.dirname(get_package_share_directory("kuka_gazebo")))
    wamv_share = os.path.join(install_prefix, "wamv_description", "share")
    vrx_gz_share = os.path.join(install_prefix, "vrx_gz", "share", "vrx_gz")
    vrx_gz_lib = os.path.join(install_prefix, "vrx_gz", "lib")
    vrx_urdf_share = os.path.join(install_prefix, "vrx_gazebo", "share")

    def _append_path(env_name, *extra_dirs):
        existing = os.environ.get(env_name, "")
        parts = [existing] + list(extra_dirs) if existing else list(extra_dirs)
        return ":".join(parts)

    extra_dirs = [install_prefix, wamv_share, vrx_gz_share, vrx_urdf_share]
    set_gz_paths = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", value=_append_path("GZ_SIM_RESOURCE_PATH", *extra_dirs)),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", value=_append_path("IGN_GAZEBO_RESOURCE_PATH", *extra_dirs)),
        SetEnvironmentVariable("SDF_PATH", value=_append_path("SDF_PATH", *extra_dirs)),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", value=_append_path("GZ_SIM_SYSTEM_PLUGIN_PATH", vrx_gz_lib)),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", value=_append_path("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", vrx_gz_lib)),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": [world_path, " -r -v1"]}.items(),
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    ros_gz_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="ros_gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                   "--ros-args", "-r", "__node:=ros_gz_bridge"], output="screen")

    # ── WAM-V hull (DART buoyancy) ──
    wamv_desc = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("kuka_landing_control"), "urdf", "wamv_parent.urdf.xacro"]),
        " ", "namespace:=", "wamv",
    ]), value_type=str)

    wamv_state_pub = Node(package="robot_state_publisher", executable="robot_state_publisher",
                          namespace="wamv",
                          parameters=[{"robot_description": wamv_desc, "use_sim_time": True}])

    spawn_wamv = Node(package="ros_gz_sim", executable="create",
                      arguments=["-topic", "/wamv/robot_description", "-name", "wamv",
                                 "-allow_renaming", "-z", "0.0"],
                      output="screen")

    # ── KUKA free-body model (Gazebo visual, no world link) ──
    kuka_gz_desc = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("kuka_landing_control"), "urdf", "kuka_gazebo_free.urdf.xacro"]),
    ]), value_type=str)

    kuka_gz_state_pub = Node(package="robot_state_publisher", executable="robot_state_publisher",
                              namespace="kuka_gz",
                              parameters=[{"robot_description": kuka_gz_desc, "use_sim_time": True}])

    spawn_kuka = Node(package="ros_gz_sim", executable="create",
                      arguments=["-topic", "/kuka_gz/robot_description", "-name", "kuka_kr600",
                                 "-allow_renaming",
                                 "-x", "0", "-y", "0", "-z", "1.0"],
                      output="screen")

    # ── Pose sync: keeps KUKA locked to WAM-V deck ──
    pose_sync = Node(package="kuka_landing_control", executable="pose_sync",
                     name="pose_sync", output="screen")

    # ── Standalone KUKA URDF (MoveIt + ros2_control only) ──
    kuka_desc = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("kuka_landing_control"), "urdf", "kuka_standalone.urdf.xacro"]),
    ]), value_type=str)

    kuka_state_pub = Node(package="robot_state_publisher", executable="robot_state_publisher",
                          namespace="kuka",
                          parameters=[{"robot_description": kuka_desc, "use_sim_time": True}])

    # ── Drone ──
    drone_desc = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("drone_description"), "urdf", "quadrotor.urdf.xacro"]),
    ]), value_type=str)

    drone_state_pub = Node(package="robot_state_publisher", executable="robot_state_publisher",
                          namespace="drone",
                          parameters=[{"robot_description": drone_desc, "use_sim_time": True}])

    spawn_drone = Node(package="ros_gz_sim", executable="create",
                       arguments=["-topic", "/drone/robot_description", "-name", "drone", "-allow_renaming",
                                  "-x", LaunchConfiguration("drone_x"), "-y", LaunchConfiguration("drone_y"),
                                  "-z", LaunchConfiguration("drone_z")],
                       output="screen")

    # ── ros2_control (for MoveIt, not for Gazebo) ──
    controller_config = os.path.join(get_package_share_directory("kuka_resources"), "config",
                                     "fake_hardware_config_6_axis.yaml")
    control_node = Node(package="controller_manager", executable="ros2_control_node",
                        parameters=[{"robot_description": kuka_desc}, controller_config, {"use_sim_time": True}],
                        output="screen")
    jtbc = Node(package="controller_manager", executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"], output="screen")
    jtc = Node(package="controller_manager", executable="spawner",
               arguments=["joint_trajectory_controller", "-c", "/controller_manager"], output="screen")

    # ── MoveIt ──
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("kuka_kr_moveit_config"),
                                      "launch", "moveit_server.launch.py"])),
        launch_arguments={"robot_model": "kr600_r2830", "robot_family": "fortec",
                          "use_sim_time": "true", "world_z": "0", "start_rviz": "false"}.items(),
    )

    # ── Landing Tracker ──
    tracker = Node(package="kuka_landing_control", executable="landing_tracker", name="landing_tracker",
                   output="screen",
                   parameters=[{"use_sim_time": True}, {"group_name": "manipulator"},
                               {"vessel_frame": "wamv/base_link"},
                               {"tracking_rate": 20.0}, {"safety_offset_z": 0.3},
                               {"touchdown_height": 1.4}, {"touchdown_time": 1.0}, {"planning_time": 0.05},
                               {"robot_description": kuka_desc,
                                "robot_description_semantic": ParameterValue(Command([
                                    PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                                    PathJoinSubstitution([FindPackageShare("kuka_landing_control"),
                                                          "srdf", "integrated_arm.srdf.xacro"]),
                                ]), value_type=str)}])

    ld = LaunchDescription(launch_args)
    for p in set_gz_paths: ld.add_action(p)
    ld.add_action(gz_sim)
    ld.add_action(ros_gz_bridge)
    ld.add_action(TimerAction(period=3.0, actions=[wamv_state_pub, drone_state_pub, kuka_gz_state_pub]))
    ld.add_action(TimerAction(period=4.0, actions=[kuka_state_pub]))
    ld.add_action(TimerAction(period=5.0, actions=[spawn_wamv]))
    ld.add_action(TimerAction(period=6.0, actions=[spawn_kuka, control_node]))
    ld.add_action(TimerAction(period=7.0, actions=[spawn_drone, pose_sync]))
    ld.add_action(TimerAction(period=8.0, actions=[jtbc]))
    ld.add_action(TimerAction(period=10.0, actions=[jtc]))
    ld.add_action(TimerAction(period=12.0, actions=[moveit]))
    ld.add_action(TimerAction(period=14.0, actions=[tracker]))
    return ld
