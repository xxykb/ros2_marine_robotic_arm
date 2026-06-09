#!/usr/bin/env python3
"""
Gazebo桥接节点 - 将ros2_control的关节状态转发到Gazebo仿真
订阅/joint_states（来自joint_state_broadcaster），发布到Gazebo的关节位置命令话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GzBridgeNode(Node):
    """将ros2_control关节状态桥接到Gazebo仿真"""

    JOINT_NAMES = ['joint1', 'joint2', 'joint3']
    CMD_TOPICS = {
        'joint1': '/robo_arm2/joint1/cmd_pos',
        'joint2': '/robo_arm2/joint2/cmd_pos',
        'joint3': '/robo_arm2/joint3/cmd_pos',
    }

    def __init__(self):
        super().__init__('gz_bridge_node')

        # 发布到Gazebo关节位置命令话题（通过ros_gz_bridge桥接）
        self.cmd_publishers = {}
        for name, topic in self.CMD_TOPICS.items():
            self.cmd_publishers[name] = self.create_publisher(Float64, topic, 10)

        # 订阅joint_states（由joint_state_broadcaster发布）
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.prev_positions = {name: None for name in self.JOINT_NAMES}

        self.get_logger().info('Gazebo桥接节点已启动，监听 /joint_states')

    def joint_state_callback(self, msg):
        """将关节位置转发到Gazebo"""
        for i, name in enumerate(msg.name):
            if name not in self.CMD_TOPICS:
                continue
            if i >= len(msg.position):
                continue

            pos = msg.position[i]
            prev = self.prev_positions[name]

            # 只在位置变化时发布
            if prev is None or abs(pos - prev) > 1e-6:
                cmd = Float64()
                cmd.data = pos
                self.cmd_publishers[name].publish(cmd)
                self.prev_positions[name] = pos


def main(args=None):
    rclpy.init(args=args)
    node = GzBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
