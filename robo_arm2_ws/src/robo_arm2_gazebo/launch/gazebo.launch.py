import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # Workspace paths
    ws_home = '/home/ubuntu22/Desktop/robo_arm2_ws'
    ros_log_dir = os.path.join(ws_home, '.ros', 'log')

    # Pre-create necessary directories
    os.makedirs(os.path.join(ws_home, '.gz', 'sim', '7'), exist_ok=True)
    os.makedirs(os.path.join(ws_home, '.ros', 'log'), exist_ok=True)

    # Set environment variables globally for all launched processes
    # This is critical for Gazebo Transport discovery to work correctly
    set_home = SetEnvironmentVariable('HOME', ws_home)
    set_ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR', ros_log_dir)
    set_ros_home = SetEnvironmentVariable('ROS_HOME', os.path.join(ws_home, '.ros'))

    # Disable FastDDS shared memory to avoid cross-process communication issues
    fastdds_profile = os.path.join(ws_home, 'fastdds_no_shm.xml')
    set_fastdds = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('robo_arm2_description'),
                'urdf', 'robo_arm2.urdf.xacro'
            ]),
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Gazebo Garden (gz sim)
    # gui=true: launch with GUI client; gui=false: server-only mode
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4',
             PathJoinSubstitution([
                 FindPackageShare('robo_arm2_gazebo'),
                 'worlds', 'empty.sdf'
             ]),
        ],
        output='screen',
        condition=IfCondition(gui),
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', ''),
        },
    )

    gz_sim_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '4',
             PathJoinSubstitution([
                 FindPackageShare('robo_arm2_gazebo'),
                 'worlds', 'empty.sdf'
             ]),
        ],
        output='screen',
        condition=UnlessCondition(gui),
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', ''),
        },
    )

    # Spawn robot in Gazebo (with delay to allow Gazebo to start)
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robo_arm2',
            '-z', '0.0',
        ],
        output='screen',
    )

    # Delay spawn to allow Gazebo to fully start
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity_node],
    )

    # Bridge for clock (Gazebo -> ROS)
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Bridge for joint position commands (ROS -> Gazebo)
    bridge_joint_cmds = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/robo_arm2/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/robo_arm2/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/robo_arm2/joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # Controller Manager with ros2_control config
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            PathJoinSubstitution([
                FindPackageShare('robo_arm2_gazebo'),
                'config', 'ros2_control.yaml'
            ]),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # Gazebo bridge node: forwards ros2_control commands to Gazebo
    gz_bridge_node = Node(
        package='robo_arm2_gazebo',
        executable='gz_bridge_node.py',
        name='gz_bridge_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Common environment for ros2 CLI commands
    ros2_cli_env = {
        'HOME': ws_home,
        'ROS_LOG_DIR': ros_log_dir,
    }

    # Load and activate controllers (with delay after controller_manager starts)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
        additional_env=ros2_cli_env,
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen',
        additional_env=ros2_cli_env,
    )

    # Delay controller loading until controller_manager is ready
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(period=3.0, actions=[load_joint_state_broadcaster]),
            ],
        )
    )

    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(period=4.0, actions=[load_joint_trajectory_controller]),
            ],
        )
    )

    return LaunchDescription([
        # Set global environment variables FIRST
        set_home,
        set_ros_log_dir,
        set_ros_home,
        set_fastdds,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI if true, server-only if false'),
        gz_sim_gui,
        gz_sim_server,
        robot_state_publisher_node,
        delayed_spawn,
        bridge_clock,
        bridge_joint_cmds,
        controller_manager_node,
        gz_bridge_node,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
    ])
