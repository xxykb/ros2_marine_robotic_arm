from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 读取你之前生成的 MoveIt 配置文件
    moveit_config = MoveItConfigsBuilder("marine_arm", package_name="marine_arm_moveit_config").to_moveit_configs()

    # 启动你的 C++ 控制节点，并将参数喂给它
    follow_node = Node(
        package="arm_control",
        executable="follow_point",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, # 就是它！逆运动学参数！
        ]
    )

    return LaunchDescription([follow_node])