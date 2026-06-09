# 三轴机械臂仿真系统 (robo_arm2)

基于 ROS 2 Humble + Gazebo Garden 的带底座三轴机械臂仿真项目，支持人工控制和自动控制两种模式。

## 系统架构

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Gazebo      │◄───►│  ros_gz_bridge   │◄───►│  ROS 2 话题     │
│  Garden      │     │  (时钟/关节命令) │     │                 │
└─────────────┘     └──────────────────┘     └────────┬────────┘
                                                       │
┌─────────────┐     ┌──────────────────┐               │
│  gz_bridge   │◄────│  joint_states    │◄──────────────┤
│  _node.py    │     │  broadcaster     │               │
└──────┬──────┘     └──────────────────┘     ┌────────┴────────┐
       │                                      │ ros2_control     │
       │              ┌──────────────────┐    │ (mock hardware)  │
       └─────────────►│  Gazebo 关节     │    └────────┬────────┘
                      │  位置控制器      │             │
                      └──────────────────┘    ┌────────┴────────┐
                                              │ joint_trajectory │
                                              │ _controller      │
                                              └────────┬────────┘
                                                       │
                              ┌─────────────────────────┼─────────────────────┐
                              │                         │                     │
                     ┌────────┴────────┐      ┌────────┴────────┐   ┌────────┴────────┐
                     │  人工控制 GUI    │      │  自动控制节点    │   │  RViz2 可视化   │
                     │  (tkinter)      │      │  (IK求解)       │   │                 │
                     └─────────────────┘      └─────────────────┘   └─────────────────┘
```

### 核心组件

| 组件 | 说明 |
|------|------|
| Gazebo Garden | 物理仿真引擎，运行机械臂模型 |
| ros2_control (mock) | 轨迹控制器框架，使用 mock 硬件 |
| ros_gz_bridge | Gazebo 与 ROS 2 之间的消息桥接 |
| gz_bridge_node.py | 自定义桥接节点，将 joint_states 转发到 Gazebo 关节命令 |
| joint_trajectory_controller | 轨迹跟踪控制器 |
| joint_state_broadcaster | 关节状态广播器 |

## 机械臂参数

- **底座**: 半径 0.10m，高度 0.05m
- **立柱**: 半径 0.05m，高度 0.05m
- **连杆1 (link1)**: 半径 0.04m，长度 0.05m
- **连杆2 (link2)**: 半径 0.03m，长度 0.30m
- **连杆3 (link3)**: 半径 0.025m，长度 0.25m
- **末端执行器**: 半径 0.02m

### 关节配置

| 关节 | 类型 | 轴 | 范围 |
|------|------|-----|------|
| joint1 | 旋转 | Z (底座旋转) | -180° ~ 180° |
| joint2 | 旋转 | Y (大臂俯仰) | -90° ~ 180° |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Garden (gz-sim7)
- ros_gz_sim, ros_gz_bridge (Garden 版本)
- ros2_control (Humble)
- Python 3 + tkinter (人工控制模式)

## 编译

```bash
cd /home/ubuntu22/Desktop/robo_arm2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 启动方式

### 方式一：使用启动脚本（推荐）

```bash
cd /home/ubuntu22/Desktop/robo_arm2_ws
./start.sh
```

脚本会提示选择模式：
- `1` - 仅 Gazebo 仿真
- `2` - 人工控制模式 (Gazebo + RViz + GUI)
- `3` - 自动控制模式 (Gazebo + RViz + Auto)

### 方式二：手动启动

**重要**: 必须先设置环境变量：

```bash
export HOME=/home/ubuntu22/Desktop/robo_arm2_ws
export ROS_LOG_DIR=/home/ubuntu22/Desktop/robo_arm2_ws/.ros/log
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu22/Desktop/robo_arm2_ws/fastdds_no_shm.xml
source /opt/ros/humble/setup.bash
source /home/ubuntu22/Desktop/robo_arm2_ws/install/setup.bash
```

然后启动对应的 launch 文件：

```bash
# 仅仿真
ros2 launch robo_arm2_gazebo gazebo.launch.py

# 人工控制模式
ros2 launch robo_arm2_teleop manual_control.launch.py

# 自动控制模式
ros2 launch robo_arm2_control auto_control.launch.py
```

## 使用说明

### 模式1：人工控制

启动人工控制模式后，会弹出 tkinter GUI 窗口：

- **滑块控制**: 拖动滑块控制 joint1（底座旋转）和 joint2（大臂俯仰）
- **数值显示**: 实时显示当前角度值
- **末端位置**: 显示末端执行器的 XYZ 坐标
- **回到零位**: 点击按钮将机械臂回到初始位置

### 模式2：自动控制

启动自动控制模式后，通过发布目标坐标控制机械臂：

```bash
# 设置环境变量（同上）
# 发布目标坐标
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.1, z: 0.5}}}"
```

自动控制节点会：
1. 接收目标坐标 (x, y, z)
2. 通过解析逆运动学 (IK) 计算关节角度
3. 发送轨迹命令移动到目标位置
4. 在 RViz 中显示目标位置标记

### IK 求解算法

- **joint1** (底座旋转): `atan2(y, x)`
- **joint2** (大臂俯仰): `atan2(r/L2, (z_rel - L1)/L2)`
  - `r = sqrt(x² + y²)` (水平距离)
  - `z_rel = z - base_height` (相对高度)

## 常用 ROS 2 命令

```bash
# 查看关节状态（需要设置环境变量）
ros2 topic echo /joint_states

# 查看控制器列表
ros2 control list_controllers

# 手动发送轨迹命令
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [joint1, joint2], points: [{positions: [0.5, 0.3], velocities: [0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}'

# 查看 Gazebo 中的关节状态
gz topic -e -t /world/empty/model/robo_arm2/joint_state -n 1
```

## 修改机器人模型

机器人模型的参数分散在多个文件中，修改时需要同步更新所有相关文件。

### 1. 修改尺寸参数

编辑 `src/robo_arm2_description/urdf/robo_arm2.urdf.xacro` 顶部的常量定义：

```xml
<!-- 第 8-16 行：尺寸常量 -->
<xacro:property name="base_radius" value="0.10" />   <!-- 底座半径 (m) -->
<xacro:property name="base_height" value="0.05" />   <!-- 底座高度 (m) -->
<xacro:property name="link1_radius" value="0.04" />   <!-- 连杆1半径 (m) -->
<xacro:property name="link1_length" value="0.30" />   <!-- 连杆1长度 (m) -->
<xacro:property name="link2_radius" value="0.03" />   <!-- 连杆2半径 (m) -->
<xacro:property name="link2_length" value="0.30" />   <!-- 连杆2长度 (m) -->
<xacro:property name="ee_radius" value="0.02" />      <!-- 末端执行器半径 (m) -->
<xacro:property name="pillar_radius" value="0.05" />  <!-- 立柱半径 (m) -->
<xacro:property name="pillar_height" value="0.20" />  <!-- 立柱高度 (m) -->
```

修改这些值后，URDF 中所有引用 `${base_radius}` 等变量的地方会自动更新。

### 2. 修改关节限位

在同一个文件中，找到关节定义的 `<limit>` 标签：

```xml
<!-- joint1（第 119 行）-->
<limit lower="-3.14159" upper="3.14159" effort="50.0" velocity="1.0" />

<!-- joint2（第 146 行）-->
<limit lower="-1.5708" upper="3.14159" effort="30.0" velocity="1.0" />
```

| 参数 | 含义 |
|------|------|
| `lower` / `upper` | 关节角度范围（弧度） |
| `effort` | 最大力矩 (N·m) |
| `velocity` | 最大角速度 (rad/s) |

### 3. 修改质量与惯性

每个 link 的 `<inertial>` 标签中包含质量值，惯性矩阵由宏自动计算：

```xml
<!-- 底座质量 -->
<xacro:cylinder_inertia m="1.0" r="${base_radius}" h="${base_height}" />

<!-- 立柱质量 -->
<xacro:cylinder_inertia m="0.5" r="${pillar_radius}" h="${pillar_height}" />

<!-- 连杆1质量 -->
<xacro:cylinder_inertia m="0.3" r="${link1_radius}" h="${link1_length}" />

<!-- 连杆2质量 -->
<xacro:cylinder_inertia m="0.2" r="${link2_radius}" h="${link2_length}" />

<!-- 连杆3质量 -->
<xacro:cylinder_inertia m="0.1" r="${link3_radius}" h="${link3_length}" />

<!-- 末端执行器质量 -->
<xacro:sphere_inertia m="0.02" r="${ee_radius}" />
```

修改 `m` 值即可改变质量，惯性矩阵会根据质量和几何尺寸自动重新计算。

#### 惯性计算原理

惯性矩阵通过 xacro 宏自动计算，公式如下：

**圆柱体**（`cylinder_inertia` 宏）：
- `ixx = iyy = m * (3*r² + h²) / 12`
- `izz = m * r² / 2`

**球体**（`sphere_inertia` 宏）：
- `ixx = iyy = izz = 2 * m * r² / 5`

其中 `m` 为质量，`r` 为半径，`h` 为高度。降低质量 `m` 会线性降低惯性矩阵的所有元素，使机械臂响应更快、运动更灵活。

> **重要**：惯性原点（`<origin>`）必须与 visual/collision 的原点一致，即设在圆柱的质心位置 `xyz="0 0 ${h/2}"`。如果惯性原点设在底部（`xyz="0 0 0"`），则等效惯量会因平行轴定理而大幅增加，导致机械臂运动迟缓。

#### 关节阻尼与摩擦

关节的 `<dynamics>` 标签控制阻尼和摩擦：

```xml
<dynamics damping="0.01" friction="0.01" />
```

| 参数 | 含义 | 调参建议 |
|------|------|---------|
| `damping` | 关节阻尼 (N·m·s/rad) | 值越大运动越慢，0.01 适合轻量连杆 |
| `friction` | 关节摩擦 (N·m) | 值越大静摩擦越大，0.01 适合轻量连杆 |

> **提示**：如果机械臂运动迟缓，优先降低阻尼和摩擦；如果运动不稳定或发散，适当增加阻尼。

#### Gazebo PID 参数

Gazebo 的 `JointPositionController` 使用 PID 控制器跟踪目标位置，在 `robo_arm2.gazebo.xacro` 中配置：

```xml
<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
  <joint_name>joint1</joint_name>
  <topic>/robo_arm2/joint1/cmd_pos</topic>
  <p_gain>100.0</p_gain>
  <i_gain>0.0</i_gain>
  <d_gain>10.0</d_gain>
</plugin>
```

| 参数 | 含义 | 调参建议 |
|------|------|---------|
| `p_gain` | 比例增益 | 越大跟踪越快，但过大会振荡 |
| `i_gain` | 积分增益 | 消除稳态误差，通常设为 0 |
| `d_gain` | 微分增益 | 抑制振荡，通常为 p_gain 的 1/10 |

> **调参方法**：先只调 P（从 50 开始），直到响应速度合适但有小幅振荡；然后增加 D（从 5 开始）消除振荡；最后如有稳态误差再加 I。

### 4. 修改外观颜色

**URDF 中的颜色**（`robo_arm2.urdf.xacro` 第 41-52 行）：

```xml
<material name="dark_grey">
  <color rgba="0.3 0.3 0.3 1.0" />   <!-- 底座、立柱 -->
</material>
<material name="blue">
  <color rgba="0.0 0.0 0.8 1.0" />   <!-- 连杆1 -->
</material>
<material name="orange">
  <color rgba="1.0 0.5 0.0 1.0" />   <!-- 连杆2 -->
</material>
<material name="red">
  <color rgba="0.8 0.0 0.0 1.0" />   <!-- 末端执行器 -->
</material>
```

**Gazebo 中的颜色**（`robo_arm2.gazebo.xacro` 第 7-40 行）：

Gazebo 使用独立的材质定义，需要同步修改 `<ambient>` 和 `<diffuse>` 值：

```xml
<gazebo reference="link1">
  <material>
    <ambient>0.0 0.0 0.8 1.0</ambient>
    <diffuse>0.0 0.0 0.8 1.0</diffuse>
  </material>
</gazebo>
```

> 注意：URDF 的 `rgba` 和 Gazebo 的 `ambient`/`diffuse` 格式相同（R G B A），但需要分别修改。

### 5. 同步更新控制脚本

修改模型参数后，**必须同步更新**以下文件中的硬编码参数：

**自动控制脚本** `src/robo_arm2_control/scripts/auto_control.py`：

```python
# 机械臂参数 —— 必须与 URDF 一致
self.base_height = 0.10      # = base_height(0.05) + pillar_height(0.05)
self.link1_length = 0.05     # = link1_length
self.link2_length = 0.30     # = link2_length
self.link3_length = 0.25     # = link3_length

# 关节限位 —— 必须与 URDF joint limit 一致
self.joint1_lower = -math.pi
self.joint1_upper = math.pi
self.joint2_lower = -math.pi / 2
self.joint2_upper = math.pi
self.joint3_lower = -math.pi
self.joint3_upper = math.pi
```

**人工控制脚本** `src/robo_arm2_teleop/scripts/manual_control.py`（第 32-47 行和第 176-178 行）：

```python
# 滑块范围 —— 必须与 URDF joint limit 一致（单位：度）
'joint1': {'min': -180, 'max': 180, ...}
'joint2': {'min': -90,  'max': 180, ...}

# 正运动学参数（第 176-178 行）
L1 = 0.30    # = link1_length
L2 = 0.30    # = link2_length
h = 0.25     # = base_height + pillar_height
```

### 6. 添加新关节（扩展为三轴机械臂）

如需添加新关节（如腕关节），需要修改以下文件：

| 步骤 | 文件 | 修改内容 |
|------|------|---------|
| 1 | `robo_arm2.urdf.xacro` | 添加新 link 和 joint 定义，在 `ros2_control` 块中添加新关节的 command/state interface |
| 2 | `robo_arm2.gazebo.xacro` | 添加新的 `JointPositionController` 插件，指定新关节名和命令话题 |
| 3 | `ros2_control.yaml` | 在 `joints` 列表中添加新关节名 |
| 4 | `gazebo.launch.py` | 在 `bridge_joint_cmds` 中添加新关节的桥接话题 |
| 5 | `gz_bridge_node.py` | 在 `CMD_TOPICS` 字典中添加新关节的命令话题映射 |
| 6 | `auto_control.py` | 更新 IK 算法，添加新关节参数 |
| 7 | `manual_control.py` | 添加新关节的滑块配置和正运动学计算 |

修改完成后，重新编译：

```bash
cd /home/ubuntu22/Desktop/robo_arm2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 项目结构

```
robo_arm2_ws/
├── start.sh                          # 启动脚本
├── fastdds_no_shm.xml                # FastDDS UDP 配置
└── src/
    ├── robo_arm2_description/        # 机器人模型描述包
    │   ├── urdf/
    │   │   ├── robo_arm2.urdf.xacro      # 主 URDF 模型
    │   │   └── robo_arm2.gazebo.xacro    # Gazebo 插件配置
    │   ├── rviz/
    │   │   ├── display.rviz              # 显示配置
    │   │   └── control.rviz              # 控制配置
    │   └── launch/
    │       └── display.launch.py         # 仅显示 launch
    ├── robo_arm2_gazebo/             # Gazebo 仿真包
    │   ├── launch/
    │   │   └── gazebo.launch.py          # 主仿真 launch
    │   ├── config/
    │   │   └── ros2_control.yaml         # ros2_control 配置
    │   ├── worlds/
    │   │   └── empty.sdf                 # 仿真世界
    │   └── scripts/
    │       └── gz_bridge_node.py         # Gazebo 桥接节点
    ├── robo_arm2_teleop/             # 人工控制包
    │   ├── scripts/
    │   │   └── manual_control.py         # tkinter GUI 控制脚本
    │   └── launch/
    │       └── manual_control.launch.py  # 人工控制 launch
    └── robo_arm2_control/            # 自动控制包
        ├── scripts/
        │   └── auto_control.py           # IK + 自动控制脚本
        └── launch/
            └── auto_control.launch.py    # 自动控制 launch
```

## 注意事项

1. **环境变量**: 由于 HOME 目录是只读文件系统，必须设置 `HOME` 和 `ROS_LOG_DIR` 环境变量到工作空间目录
2. **FastDDS 配置**: 必须设置 `FASTRTPS_DEFAULT_PROFILES_FILE` 禁用共享内存传输，否则跨进程的 ROS 2 话题通信会失败
3. **启动延迟**: Gazebo 启动需要约 5 秒，控制器加载需要额外 3-4 秒，请耐心等待
4. **工作空间**: 机械臂最大水平伸展距离约 0.55m（link2 + link3），joint1 高度为 0.10m
5. **Gazebo PID 跟踪**: Gazebo 中的 JointPositionController 使用 PID 控制，存在少量跟踪误差
