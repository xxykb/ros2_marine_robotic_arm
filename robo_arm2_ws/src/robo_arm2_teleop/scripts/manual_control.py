#!/usr/bin/env python3
"""
人工控制GUI节点 - 三轴机械臂
使用tkinter创建滑块控件，通过action client发送轨迹命令控制机械臂。
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk


class ManualControlGUI:
    """tkinter GUI用于手动控制三轴机械臂"""

    def __init__(self, node):
        self.node = node
        self.root = None
        self.sliders = {}
        self.labels = {}
        self.value_labels = {}
        self.current_positions = {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0}
        self.updating_from_state = False

        # 关节配置
        self.joints = {
            'joint1': {
                'name': '底座旋转 (Joint1)',
                'min': -180,
                'max': 180,
                'default': 0,
                'unit': '°'
            },
            'joint2': {
                'name': '大臂俯仰 (Joint2)',
                'min': -90,
                'max': 180,
                'default': 0,
                'unit': '°'
            },
            'joint3': {
                'name': '小臂俯仰 (Joint3)',
                'min': -180,
                'max': 180,
                'default': 0,
                'unit': '°'
            }
        }

        # 机械臂参数（必须与 URDF 一致）
        self.link1_length = 0.05
        self.link2_length = 0.30
        self.link3_length = 0.25
        self.base_height = 0.10  # base_height + pillar_height

        # Action client
        self._action_client = ActionClient(
            node, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Joint state subscriber
        self.joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        """更新当前关节状态"""
        for i, name in enumerate(msg.name):
            if name in self.current_positions and i < len(msg.position):
                self.current_positions[name] = msg.position[i]
                # 更新GUI显示
                if self.root and name in self.sliders:
                    self.updating_from_state = True
                    self.sliders[name].set(math.degrees(msg.position[i]))
                    self.updating_from_state = False

    def run(self):
        """在主线程中运行GUI"""
        self.root = tk.Tk()
        self.root.title("三轴机械臂 - 人工控制")
        self.root.geometry("550x480")
        self.root.resizable(True, True)

        # 标题
        title_frame = ttk.Frame(self.root, padding="10")
        title_frame.pack(fill=tk.X)
        ttk.Label(title_frame, text="三轴机械臂人工控制面板",
                  font=('Arial', 16, 'bold')).pack()

        # 状态栏
        status_frame = ttk.Frame(self.root, padding="5")
        status_frame.pack(fill=tk.X)
        self.status_label = ttk.Label(status_frame, text="状态: 等待连接...",
                                       foreground='orange')
        self.status_label.pack()

        # 关节控制区域
        control_frame = ttk.LabelFrame(self.root, text="关节控制", padding="10")
        control_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        for joint_name, config in self.joints.items():
            joint_frame = ttk.Frame(control_frame)
            joint_frame.pack(fill=tk.X, pady=5)

            # 标签
            label = ttk.Label(joint_frame, text=config['name'], width=18)
            label.pack(side=tk.LEFT)

            # 滑块
            slider = ttk.Scale(joint_frame, from_=config['min'], to=config['max'],
                               orient=tk.HORIZONTAL,
                               command=lambda val, jn=joint_name: self.on_slider_change(jn, val))
            slider.set(config['default'])
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
            self.sliders[joint_name] = slider

            # 数值显示
            value_label = ttk.Label(joint_frame, text=f"{config['default']:.1f}{config['unit']}",
                                     width=10)
            value_label.pack(side=tk.RIGHT)
            self.value_labels[joint_name] = value_label

        # 末端执行器位置显示
        ee_frame = ttk.LabelFrame(self.root, text="末端执行器位置", padding="10")
        ee_frame.pack(fill=tk.X, padx=10, pady=5)

        self.ee_pos_label = ttk.Label(ee_frame, text="X: 0.000  Y: 0.000  Z: 0.400",
                                       font=('Courier', 11))
        self.ee_pos_label.pack()

        # 按钮区域
        button_frame = ttk.Frame(self.root, padding="10")
        button_frame.pack(fill=tk.X)

        home_btn = ttk.Button(button_frame, text="回到零位",
                              command=self.go_home)
        home_btn.pack(side=tk.LEFT, padx=5)

        # 检查action server
        self.root.after(1000, self.check_action_server)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def check_action_server(self):
        """检查action server是否可用"""
        if self._action_client.wait_for_server(timeout_sec=0.5):
            self.status_label.config(text="状态: 已连接控制器", foreground='green')
        else:
            self.status_label.config(text="状态: 等待控制器连接...", foreground='orange')
            self.root.after(2000, self.check_action_server)

    def on_slider_change(self, joint_name, value):
        """滑块值改变回调"""
        if self.updating_from_state:
            return

        value_deg = float(value)
        value_rad = math.radians(value_deg)

        # 更新数值显示
        config = self.joints[joint_name]
        self.value_labels[joint_name].config(text=f"{value_deg:.1f}{config['unit']}")

        # 更新末端执行器位置
        self.update_ee_position()

        # 发送轨迹命令
        self.send_single_joint_trajectory(joint_name, value_rad)

    def update_ee_position(self):
        """计算并更新末端执行器位置（三连杆正运动学）"""
        j1 = math.radians(self.sliders['joint1'].get())
        j2 = math.radians(self.sliders['joint2'].get())
        j3 = math.radians(self.sliders['joint3'].get())

        L1 = self.link1_length
        L2 = self.link2_length
        L3 = self.link3_length
        h = self.base_height

        # 正运动学：link1沿Z轴，joint2绕Y轴旋转link2，joint3绕Y轴旋转link3
        # 在垂直平面内：
        #   x_plane = L2*sin(j2) + L3*sin(j2+j3)
        #   z_plane = L1 + L2*cos(j2) + L3*cos(j2+j3)
        x_plane = L2 * math.sin(j2) + L3 * math.sin(j2 + j3)
        z_plane = L1 + L2 * math.cos(j2) + L3 * math.cos(j2 + j3)

        x = x_plane * math.cos(j1)
        y = x_plane * math.sin(j1)
        z = h + z_plane

        self.ee_pos_label.config(text=f"X: {x:.3f}  Y: {y:.3f}  Z: {z:.3f}")

    def send_single_joint_trajectory(self, joint_name, position_rad):
        """发送单关节轨迹命令"""
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            return

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        # 使用当前所有关节位置
        positions = []
        for jn in ['joint1', 'joint2', 'joint3']:
            if jn == joint_name:
                positions.append(position_rad)
            else:
                positions.append(math.radians(self.sliders[jn].get()))

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self._action_client.send_goal_async(goal_msg)

    def go_home(self):
        """回到零位"""
        self.sliders['joint1'].set(0)
        self.sliders['joint2'].set(0)
        self.sliders['joint3'].set(0)

        if not self._action_client.wait_for_server(timeout_sec=0.5):
            return

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self._action_client.send_goal_async(goal_msg)
        self.status_label.config(text="状态: 回到零位中...", foreground='blue')

    def on_close(self):
        """关闭窗口"""
        if self.root:
            self.root.destroy()
            self.root = None


def main(args=None):
    rclpy.init(args=args)
    node = Node('manual_control_gui')

    gui = ManualControlGUI(node)

    # 在单独线程中运行ROS2 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 在主线程中运行GUI
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
