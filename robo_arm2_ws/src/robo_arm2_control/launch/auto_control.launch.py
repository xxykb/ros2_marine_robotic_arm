import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robo_arm2_gazebo'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true', 'gui': 'true'}.items(),
    )

    # RViz2
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robo_arm2_description'),
        'rviz', 'control.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Auto control node
    auto_control_node = Node(
        package='robo_arm2_control',
        executable='auto_control.py',
        name='auto_control_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo_launch,
        rviz_node,
        auto_control_node,
    ])
