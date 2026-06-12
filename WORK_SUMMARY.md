# 海洋机械臂仿真 — 工作总结

## 当前状态（2026-06-11）

**能运行**: WAM-V 船体在 DART 物理引擎下漂浮（Surface 浮力插件），KUKA KR600 机械臂通过 `pose_sync` 节点跟随甲板运动。

```
ros2 launch kuka_landing_control marine_landing.launch.py
# 启动前清缓存: rm -rf ~/.gz/sim ~/.ignition/gazebo
```

---

## 架构

```
Gazebo World (DART physics, 0.001s step)
├── WAM-V (wamv_parent.urdf.xacro)
│   └── 180kg, 自由刚体, Surface×2 浮力插件, 随波浪浮动
├── KUKA KR600 (kuka_gazebo_free.urdf.xacro)
│   └── static, 无物理, 由 pose_sync 每20ms 设置世界位姿
├── Drone (quadrotor.urdf.xacro)
└── Coast Waves 水面模型

pose_sync (C++, gz-transport):
  订阅 /world/marine_world/pose/info (Pose_V from SceneBroadcaster)
  → 过滤 WAM-V 位姿
  → set_pose KUKA = WAM-V_pose × deck_offset(0,0,1.16)
  → 50Hz

MoveIt (独立进程, fake_hardware):
  kuka_standalone.urdf.xacro (含 world link)
  → landing_tracker: TF 变换 drone→vessel 坐标
  → 规划 KUKA 关节轨迹追踪无人机
```

---

## 关键文件

### 本次对话新增/修改

| 文件 | 作用 |
|------|------|
| `src/kuka_landing_control/src/pose_sync.cpp` | KUKA 位姿同步节点 (gz-transport) |
| `src/kuka_landing_control/urdf/kuka_gazebo_free.urdf.xacro` | KUKA Gazebo 模型 (static, 无 world link) |
| `src/kuka_landing_control/urdf/wamv_kuka_bullet.urdf.xacro` | Bullet 合并模型 (已弃用, Bullet 浮力不工作) |
| `src/kuka_landing_control/scripts/scale_mass.py` | URDF 质量缩放后处理 (为 Bullet 方案准备) |
| `src/kuka_landing_control/launch/marine_landing.launch.py` | 启动文件 |
| `src/kuka_landing_control/CMakeLists.txt` | 新增 gz-transport12/gz-msgs9 依赖 |
| `src/kuka_landing_control/package.xml` | 同上 |
| `src/kuka_landing_control/worlds/marine_world.sdf` | 步长改为 0.001s |

### 之前已有的关键文件

| 文件 | 作用 |
|------|------|
| `src/kuka_landing_control/urdf/wamv_parent.urdf.xacro` | WAM-V 单体 (180kg, Surface×2, number=3) |
| `src/kuka_landing_control/urdf/kuka_standalone.urdf.xacro` | KUKA MoveIt 模型 (含 world link) |
| `src/kuka_landing_control/src/landing_tracker.cpp` | 降落追踪节点 (MoveIt 规划) |
| `src/kuka_landing_control/srdf/integrated_arm.srdf.xacro` | KUKA 运动学配置 |
| `src/kuka_landing_control/urdf/wamv_base_local.urdf.xacro` | WAM-V 本地副本 (含碰撞体) |
| `src/kuka_landing_control/urdf/wamv_light.urdf.xacro` | WAM-V 轻量版 |
| `src/kuka_landing_control/urdf/wamv_kuka_integrated.urdf.xacro` | 合并模型尝试 (DART 拒绝) |
| `src/kuka_landing_control/urdf/wamv_with_kuka.urdf.xacro` | 合并模型另一个尝试 |
| `src/kuka_landing_control/worlds/test_no_water.sdf` | 无水测试世界 |

---

## 已解决的问题

1. ✅ VRX 插件名称修复 (S1→Surface, Pub→PublisherPlugin)  
2. ✅ 资源路径修复 (install_share → install/ 前缀)  
3. ✅ wave number=5→3 解决 PMS NaN 崩溃  
4. ✅ wamv_state_pub 加 namespace 解决 topic 不匹配  
5. ✅ SDF 几何语法修复  
6. ✅ 删除 WAM-V cylinder 碰撞体避免 DART 崩溃  
7. ✅ MoveIt RViz 加 start_rviz:=false  
8. ✅ KUKA 质量过大 → 质量缩放 (Bullet 方案) 或 static (DART 方案)  
9. ✅ 合并模型被 DART 拒绝 → 改用双模型 + pose_sync  
10. ✅ AddWorldForce 不生效 → 放弃, 改用 set_pose  
11. ✅ 无法切换 Bullet → 确认 Bullet 下 Surface 浮力不工作  
12. ✅ pose_sync 收不到位姿 → PosePublisher 不能挂 world, 改用 SceneBroadcaster 的 Pose_V  

---

## 待解决问题（下个对话）

1. **水面渲染**: 视觉效果需改善  
2. **Gazebo 缓存**: ~/.gz/sim/ 清理不彻底, 偶发加载旧模型导致 NaN  
3. **KUKA 关节控制**: 目前 Gazebo 里 KUKA 静止, 需联动 MoveIt 规划结果  
4. **着陆逻辑**: landing_tracker 的 touchdown 检测需调试  
