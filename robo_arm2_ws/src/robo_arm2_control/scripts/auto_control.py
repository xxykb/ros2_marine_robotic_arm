#!/usr/bin/env python3
"""
自动控制节点 - 三轴机械臂
接收目标坐标，通过解析逆运动学计算关节角度，发送轨迹命令移动到目标点。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState


class AutoControlNode(Node):
    """自动控制节点，接收目标坐标并移动机械臂到目标位置"""

    def __init__(self):
        super().__init__('auto_control_node')

        # 机械臂参数（必须与 URDF 一致）
        self.base_height = 0.10   # base_height(0.05) + pillar_height(0.05)
        self.link1_length = 0.05
        self.link2_length = 0.30
        self.link3_length = 0.25

        # 关节限位（必须与 URDF joint limit 一致）
        self.joint1_lower = -math.pi
        self.joint1_upper = math.pi
        self.joint2_lower = -math.pi / 2
        self.joint2_upper = math.pi
        self.joint3_lower = -math.pi
        self.joint3_upper = math.pi

        # Action client for trajectory controller
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Subscriber for target pose
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_callback,
            10
        )

        # Publisher for target marker visualization
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)

        # Current joint states
        self.current_joints = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('三轴机械臂自动控制节点已启动')
        self.get_logger().info('发布目标坐标到 /target_pose 话题来控制机械臂')
        self.get_logger().info('示例: ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped '
                               '"{header: {frame_id: \'base_link\'}, pose: {position: {x: 0.3, y: 0.0, z: 0.4}}}"')

    def joint_state_callback(self, msg):
        """更新当前关节状态"""
        for i, name in enumerate(msg.name):
            if name in self.current_joints and i < len(msg.position):
                self.current_joints[name] = msg.position[i]

    def target_callback(self, msg):
        """接收目标坐标并执行运动"""
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_z = msg.pose.position.z

        self.get_logger().info(f'收到目标坐标: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')

        # 发布目标标记
        self.publish_target_marker(target_x, target_y, target_z)

        # 计算逆运动学
        result = self.inverse_kinematics(target_x, target_y, target_z)

        if result is None:
            self.get_logger().warn('目标坐标超出工作空间范围！')
            return

        joint1, joint2, joint3 = result
        self.get_logger().info(f'IK解: joint1={math.degrees(joint1):.1f}°, '
                               f'joint2={math.degrees(joint2):.1f}°, '
                               f'joint3={math.degrees(joint3):.1f}°')

        # 发送轨迹命令
        self.send_trajectory(joint1, joint2, joint3)

    def inverse_kinematics(self, x, y, z):
        """
        三连杆解析逆运动学求解
        joint1: 绕Z轴旋转（底座旋转）
        joint2: 绕Y轴旋转（大臂俯仰）
        joint3: 绕Y轴旋转（小臂俯仰）

        返回: (joint1, joint2, joint3) 或 None 如果无解
        """
        L1 = self.link1_length
        L2 = self.link2_length
        L3 = self.link3_length
        h = self.base_height  # joint1到地面的高度

        # joint1: 底座旋转角
        joint1 = math.atan2(y, x)

        # 在垂直平面内求解 joint2 和 joint3
        r = math.sqrt(x**2 + y**2)  # 水平距离
        z_rel = z - h - L1  # 相对于joint2的垂直距离（减去link1沿Z轴的偏移）

        # 目标在垂直平面内到joint2的距离
        d_sq = r**2 + z_rel**2
        d = math.sqrt(d_sq)

        # 使用余弦定理求解joint3
        # d^2 = L2^2 + L3^2 + 2*L2*L3*cos(joint3)
        cos_j3 = (d_sq - L2**2 - L3**2) / (2 * L2 * L3)

        if abs(cos_j3) > 1.0:
            if cos_j3 > 1.0:
                # 目标太远，伸直到最远
                self.get_logger().warn(f'目标超出可达范围: d={d:.3f}, 最大可达={L2+L3:.3f}')
                joint3 = 0.0  # 伸直
                # joint2 指向目标方向
                beta = math.atan2(r, z_rel)
                joint2 = beta
                return self._clamp_joints(joint1, joint2, joint3)
            else:
                # 目标太近
                self.get_logger().warn(f'目标太近: d={d:.3f}, 最小可达={abs(L2-L3):.3f}')
                cos_j3 = -1.0  # 折叠

        # 取 elbow-up 解（joint3 < 0 表示向上弯曲）
        joint3 = -math.acos(max(-1.0, min(1.0, cos_j3)))

        # 求解 joint2
        # beta: 目标方向角
        beta = math.atan2(r, z_rel)
        # alpha: 连杆2和连杆3合力的偏移角
        alpha = math.atan2(L3 * math.sin(joint3), L2 + L3 * math.cos(joint3))
        joint2 = beta - alpha

        return self._clamp_joints(joint1, joint2, joint3)

    def _clamp_joints(self, joint1, joint2, joint3):
        """限制关节角度在有效范围内"""
        clamped = [joint1, joint2, joint3]
        limits = [
            (self.joint1_lower, self.joint1_upper),
            (self.joint2_lower, self.joint2_upper),
            (self.joint3_lower, self.joint3_upper),
        ]
        names = ['joint1', 'joint2', 'joint3']

        for i, (lo, hi) in enumerate(limits):
            if clamped[i] < lo or clamped[i] > hi:
                clamped[i] = max(lo, min(hi, clamped[i]))
                self.get_logger().warn(f'{names[i]}被限位到 {math.degrees(clamped[i]):.1f}°')

        return clamped[0], clamped[1], clamped[2]

    def publish_target_marker(self, x, y, z):
        """在RViz中发布目标位置标记"""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def send_trajectory(self, joint1, joint2, joint3):
        """发送轨迹命令到控制器"""
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('轨迹控制器action服务器不可用！')
            return

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        # 计算运动时间（基于角度差）
        delta_j1 = abs(joint1 - self.current_joints['joint1'])
        delta_j2 = abs(joint2 - self.current_joints['joint2'])
        delta_j3 = abs(joint3 - self.current_joints['joint3'])
        max_delta = max(delta_j1, delta_j2, delta_j3)
        duration_sec = max(1.0, max_delta / 0.5)  # 最小1秒，最大速度0.5 rad/s

        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2, joint3]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)

        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'发送轨迹目标: joint1={math.degrees(joint1):.1f}°, '
                               f'joint2={math.degrees(joint2):.1f}°, '
                               f'joint3={math.degrees(joint3):.1f}°, '
                               f'时间={duration_sec:.1f}s')

        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
