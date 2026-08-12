#!/bin/bash
# ============================================================
# setup.sh — sw01-fastlio2-sim 环境一键初始化
#
# 前提：Ubuntu 22.04 + ROS 2 Humble 已安装（ros-humble-desktop）
# 用法：source /opt/ros/humble/setup.bash && bash setup.sh
# ============================================================
set -e

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "[setup] 工作目录: $WORKSPACE_DIR"

# 1. 安装 ROS 依赖（Gazebo 插件、Velodyne、Nav2、slam_toolbox、FAST-LIO2 依赖）
echo "[setup] 安装系统依赖..."
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    # velodyne Gazebo 插件（来自 ros-humble-velodyne-simulator，含 velodyne_description + velodyne_gazebo_plugins）
    ros-humble-velodyne-simulator \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    libpcl-dev \
    libeigen3-dev \
    python3-colcon-common-extensions \
    python3-rosdep

# 2. rosdep 初始化（幂等）
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

# 3. 在 src/ 内安装所有包的 rosdep 依赖
echo "[setup] rosdep install..."
cd "$WORKSPACE_DIR"
rosdep install --from-paths src --ignore-src -r -y

# 4. 构建
echo "[setup] colcon build..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "[setup] 完成！使用前执行："
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo "  ros2 launch sw01_gazebo full_demo.launch.py"
