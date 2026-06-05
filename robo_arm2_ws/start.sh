#!/bin/bash
# 二轴机械臂启动脚本
# 设置必要的环境变量并启动仿真

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

# 设置环境变量（解决只读HOME和FastDDS共享内存问题）
export HOME="$WS_DIR"
export ROS_LOG_DIR="$WS_DIR/.ros/log"
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS_DIR/fastdds_no_shm.xml"

# 创建必要目录
mkdir -p "$WS_DIR/.ros/log"
mkdir -p "$WS_DIR/.gz/sim/7"

# Source ROS 2 和工作空间
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

echo "========================================="
echo "  机械臂仿真系统"
echo "========================================="
echo ""
echo "启动模式:"
echo "  1 - 仅 Gazebo 仿真"
echo "  2 - 人工控制模式 (Gazebo + RViz + GUI)"
echo "  3 - 自动控制模式 (Gazebo + RViz + Auto)"
echo ""
echo -n "请选择模式 [1/2/3]: "
read mode

case $mode in
    1)
        echo "启动 Gazebo 仿真..."
        ros2 launch robo_arm2_gazebo gazebo.launch.py
        ;;
    2)
        echo "启动人工控制模式..."
        ros2 launch robo_arm2_teleop manual_control.launch.py
        ;;
    3)
        echo "启动自动控制模式..."
        ros2 launch robo_arm2_control auto_control.launch.py
        ;;
    *)
        echo "无效选择，启动 Gazebo 仿真..."
        ros2 launch robo_arm2_gazebo gazebo.launch.py
        ;;
esac
