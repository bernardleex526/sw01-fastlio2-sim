# SW01 轮足机器狗 FAST-LIO2 / Gazebo / Nav2 仿真运行手册

本工作空间面向 **WSL2 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + WSLg**。它在 30 m × 30 m 迷宫中加载 SW01 固定站姿模型，仿真 16 线 LiDAR、200 Hz IMU 和 50 Hz ground truth，以本地 FAST-LIO2 提供里程计，再由 `pointcloud_to_laserscan`、`slam_toolbox` 和 Nav2 完成二维建图导航。

> 本仓库可在 Windows 上做源码、XML/YAML、网格哈希和 Python 静态验证；Gazebo、ROS 图、TF、频率、导航和精度必须在上述 WSL ROS 环境中验证。Windows 静态验证通过不代表 Gazebo/ROS 运行成功，也不代表达到了 ATE/RTE 指标。

## 1. 架构与实际源码调整

### 1.1 本地 FAST-LIO2 是权威实现

本机算法源目录是：

```text
D:\slam\自研\fast-lio2\slam\src\x30\sensing\FAST_LIO
```

对应 WSL 路径 `/mnt/d/slam/自研/fast-lio2/slam/src/x30/sensing/FAST_LIO`。实际 ROS 2 package 为 `fast_lio`，实际 executable 为 `fastlio_mapping`。本工程不复制、不修改算法仓库，也不把网络参考仓库当作运行代码。

本地源码固定接口如下：

- 输入参数键：`common.lid_topic=/velodyne_points`、`common.imu_topic=/imu/data`；
- `preprocess.lidar_type=2` 选择 Velodyne `sensor_msgs/msg/PointCloud2` 路径；
- 源码创建/固定名称端点：`/cloud_registered`、`/cloud_registered_body`、`/cloud_effected`、`/Laser_map`、`/Odometry`、`/path`；
- 本配置实际启用输出：`/cloud_registered`、`/cloud_registered_body`、`/Odometry`、`/path`；
- 固定动态 TF：`camera_init -> body`；
- executable 名不是 `fast_lio`，而是 `fastlio_mapping`；
- 实际 node 名是 `laser_mapping`（本地源码 `laserMapping.cpp` 中 `LaserMappingNode(...) : Node("laser_mapping")`），本工程 `slam.launch.py` 显式以 `name="laser_mapping"` 固定，避免节点名随 executable 名漂移；
- 本地 `package.xml` 无条件依赖 `livox_ros_driver2`，所以即使仿真使用 Velodyne，也必须让该包在构建时可发现。

### 1.2 SW01 模型取舍

原始 SW01 URDF 有 **17 个 link、16 个 joint**：`base_link` 与四条腿各自的 HIP/THIGH/CALF/WHEEL。原始 16 个 joint 是 revolute，但 effort/velocity 为 0，轮关节上下限也为 0；原文件没有 `<gazebo>`、sensor、plugin、transmission 或 `ros2_control`。

仿真 Xacro 字节级保留 17 个原始 STL 和原 link/joint 名称，将 16 个轮足 joint 固定在零位站姿，再增加 `imu_link`、`lidar_link` 及两个 fixed joint。因此完整仿真描述展开后是 19 个 link、18 个 joint。机器人由 Gazebo **Planar Move** 插件接收 `/cmd_vel` 在 xy/yaw 平面运动；它明确关闭 odom 和 odom TF 发布，不能冒充定位来源。这是“完整轮足外形 + 稳定站姿 + 平面导航”仿真，不是步态、关节伺服或轮电机动力学仿真。

原模型没有 Camera，本工程也未补 Camera：相机不参与 FAST-LIO2 → scan → slam_toolbox → Nav2 闭环。补充设备及理由是：

- LiDAR：本地 FAST-LIO2 的必要主观测，输出 `/velodyne_points`；
- IMU：FAST-LIO2 紧耦合状态估计的必要输入，输出 `/imu/data`；
- P3D：只输出 `/ground_truth/odom` 供评估，不进入定位、建图、控制或 TF；
- Planar Move：在不引入四足控制器的前提下执行 Nav2 `/cmd_vel`。

Gazebo 模型/资源路径由包导出自动建立，不需要手工 `GAZEBO_MODEL_PATH`：`sw01_description` 与 `sw01_gazebo` 的 `package.xml` 导出 `<gazebo_ros gazebo_model_path="${prefix}/../"/>` 与 `<gazebo_ros gazebo_media_path="${prefix}"/>`，Humble `gzserver.launch.py` 的 `GazeboRosPaths` 会读取并自动追加到 `GAZEBO_MODEL_PATH`/`GAZEBO_RESOURCE_PATH`；`simulation.launch.py` 的 `spawn_entity.py` 带 `-package_to_model`，把 17 个 STL 的 `package://` mesh URI 转为 `model://` 后按上述路径解析。

### 1.3 数据流

```text
Gazebo Classic
  ├─ Velodyne 16 (10 Hz) ─ /velodyne_points ─> FAST-LIO2
  ├─ IMU (200 Hz) ──────── /imu/data ─────────> FAST-LIO2
  ├─ P3D (50 Hz) ───────── /ground_truth/odom ─> evaluate_slam.py
  └─ Planar Move <──────── /cmd_vel ──────────── Nav2

FAST-LIO2 ─ /Odometry, /path, /cloud_registered + camera_init -> body
/cloud_registered ─ pointcloud_to_laserscan ─ /scan
/scan + FAST-LIO2 TF ─ slam_toolbox ─ /map + map -> camera_init
/map + /scan + /Odometry + TF ─ Nav2 ─ /cmd_vel
```

## 2. 项目树

`fast_lio2` 和 `src/FAST_LIO` 是在 WSL 中创建的软链，不应提交；`build/`、`install/`、`log/` 也是本地产物并已由仓库 `.gitignore` 排除。

```text
sw01-fastlio2-sim/
├── README.md
├── pytest.ini
├── fast_lio2 -> /mnt/d/slam/自研/fast-lio2
├── src/
│   ├── FAST_LIO -> ../fast_lio2/slam/src/x30/sensing/FAST_LIO
│   ├── sw01_description/
│   │   ├── meshes/                 # 17 个原始 STL + ASSET_PROVENANCE.md
│   │   ├── urdf/sw01_sim.urdf.xacro
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── sw01_gazebo/
│   │   ├── worlds/sw01_maze.world
│   │   ├── launch/simulation.launch.py
│   │   ├── launch/full_demo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── sw01_slam/
│   │   ├── config/sw01_sim.yaml
│   │   ├── config/slam_toolbox_params.yaml
│   │   ├── launch/slam.launch.py
│   │   ├── rviz/sw01_sim.rviz
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── sw01_navigation/
│       ├── config/nav2_params.yaml
│       ├── launch/navigation.launch.py
│       ├── scripts/send_nav_goal.py
│       ├── scripts/evaluate_slam.py
│       ├── scripts/trajectory_metrics.py
│       ├── CMakeLists.txt
│       └── package.xml
└── tests/                          # 静态、接口、网格、迷宫与算法测试
```

## 3. 话题、频率和类型

频率是设计值/运行验收值，不是 Windows 静态测试的实测结果。

| 话题 | 类型 | 期望频率 | 发布者与用途 |
|---|---|---:|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | 随仿真步进 | Gazebo；所有节点 `use_sim_time=true` |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | ≈ 10 Hz | Velodyne Gazebo 插件；FAST-LIO2 输入 |
| `/imu/data` | `sensor_msgs/msg/Imu` | ≈ 200 Hz | Gazebo IMU 插件；FAST-LIO2 输入 |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | ≈ 50 Hz | P3D；仅评估使用 |
| `/Odometry` | `nav_msgs/msg/Odometry` | ≈ 10 Hz | FAST-LIO2；parent `camera_init`、child `body` |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | ≈ 10 Hz | FAST-LIO2；frame `camera_init`，投影和 RViz 使用 |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | ≈ 10 Hz | FAST-LIO2；frame `body` |
| `/cloud_effected` | `sensor_msgs/msg/PointCloud2` | 0 Hz | FAST-LIO2 创建端点；默认禁用（`publish.effect_map_en=false`） |
| `/Laser_map` | `sensor_msgs/msg/PointCloud2` | 0 Hz | FAST-LIO2 创建端点；默认禁用（`publish.map_en=false`），不用于 2D scan |
| `/path` | `nav_msgs/msg/Path` | ≈ 1 Hz | FAST-LIO2 每 10 帧发布一次；10 Hz LiDAR 下约 1 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | ≈ 10 Hz | `pointcloud_to_laserscan`；slam_toolbox/Nav2 输入 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | ≈ 1 Hz | slam_toolbox；Nav2 global costmap 输入 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 控制时最高 ≈ 40 Hz | Nav2 velocity smoother；Planar Move 输入 |

Velodyne 路径的 PointCloud2 预处理契约写作 `x/y/z/intensity/time/ring`。`ring` 必须存在；若 `time` 字段缺失、全零或没有有效逐点时间，本地 FAST-LIO2 的 Velodyne 分支会用 ring、方位角和 `scan_rate=10` 合成相对时间。这个结论只适用于已审计的本地源码，不可推广到任意 FAST-LIO fork。

以上 FAST-LIO2 输出状态来自本地 `laserMapping.cpp` 与 `sw01_sim.yaml`：源码中 `publish.effect_map_en`、`publish.map_en` 默认都是 `false`，本配置未覆盖，因此 `/cloud_effected` 和 `/Laser_map` 当前无消息（0 Hz）；`publish_path()` 每 10 帧才发布一次，所以 10 Hz LiDAR 输入下 `/path` 约为 1 Hz。若确需这两个点云输出，必须在 `publish` 参数中显式配置；其中 `/Laser_map` 的主循环调用当前被注释，且定时器仍受 `map_pub_en` 门控，能否实际发布必须以所用本地源码为准，不能只凭端点存在作判断。

## 4. TF 唯一发布者与参数来源

### 4.1 TF 树与唯一发布者

```text
map
└── camera_init                  slam_toolbox
    └── body                     FAST-LIO2
        └── base_link            static_transform_publisher
            ├── imu_link         robot_state_publisher
            ├── lidar_link       robot_state_publisher
            └── 原始 16-link 轮足子树  robot_state_publisher
```

| 变换 | TF 唯一发布者 | 约束 |
|---|---|---|
| `map -> camera_init` | slam_toolbox | 不启动 AMCL/map_server |
| `camera_init -> body` | `fast_lio/laser_mapping`（executable `fastlio_mapping`） | URDF 不定义 `body` link |
| `body -> base_link` | `tf2_ros/static_transform_publisher` | 标定 `(-0.0065429535, 0.0050846333, -0.016924295)` |
| `base_link ->` 传感器/轮足 links | `robot_state_publisher` | fixed joint，通常发布到 `/tf_static` |

P3D 与 Planar Move 都不发布导航 odom/TF。若 `/tf` 的详细 publisher 列表出现第二个 `camera_init -> body` 或第二个 `map -> camera_init`，必须先修复冲突，不能靠增大 TF tolerance 掩盖。

### 4.2 参数来源

| 参数域 | 文件 | 来源 |
|---|---|---|
| 模型、传感器 pose/rate/topic、Gazebo 插件 | `src/sw01_description/urdf/sw01_sim.urdf.xacro` | 原 SW01 几何、本地 FAST-LIO2 接口、Gazebo 插件契约 |
| FAST-LIO2 topics、LiDAR 类型、外参、滤波 | `src/sw01_slam/config/sw01_sim.yaml` | 本地 `FAST_LIO` 源码及本地 `velodyne.yaml` |
| map/odom/base frame、scan、建图周期 | `src/sw01_slam/config/slam_toolbox_params.yaml` | 本工程 TF 设计与 Nav2 mapping 模式 |
| footprint、planner/controller/costmap/lifecycle | `src/sw01_navigation/config/nav2_params.yaml` | 实际 STL 外廓、迷宫通道与 Humble Nav2 接口 |
| 起点、world、gui、RViz、0/5/12 s 延时 | 四个 launch 文件 | 迷宫起点和任务启动顺序 |
| 默认终点 `(12, 12, 0)` | `send_nav_goal.py` | `sw01_maze.world` 终点标记 |

## 环境搭建

| 项 | 要求 |
|----|------|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Classic 11（随 ros-humble-gazebo-ros-pkgs 安装） |
| 关键 ROS 包 | gazebo_ros, gazebo_plugins, velodyne_gazebo_plugins, slam_toolbox, nav2_bringup |

### 一键安装

```bash
source /opt/ros/humble/setup.bash
bash setup.sh
```

### 手动安装

以下命令与 `setup.sh` 的 apt 清单、rosdep 与 colcon 参数完全一致（`setup.sh` 已自动执行全部步骤）：

```bash
# 安装 Gazebo 和传感器插件
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-velodyne-simulator

# 安装 SLAM 和导航
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup

# 安装 rosdep 依赖（livox_ros_driver2 无 Humble rosdep 键，显式跳过）
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys livox_ros_driver2

# 构建（--base-paths src 防止 colcon 从仓库根递归发现 fast_lio2 软链）
colcon build --symlink-install --base-paths src
source install/setup.bash
```

> `ros-humble-velodyne-simulator` 是包含 `velodyne_description` 与 `velodyne_gazebo_plugins` 两个 ROS 包的 deb 名；不存在名为 `ros-humble-velodyne-gazebo-plugins` 的 deb 包。

## SLAM 后端

当前激活后端为 **slam_toolbox**（`src/sw01_slam/launch/slam.launch.py`），提供 2D 占用栅格地图。

`src/sw01_slam/config/sw01_sim.yaml` 为备用 **FAST-LIO2** 后端的仿真参数（LiDAR-Inertial 3D 建图）。

如需切换到 FAST-LIO2 后端：
1. 在 `slam.launch.py` 中注释掉 slam_toolbox 相关节点，取消注释 fastlio_mapping 节点（若无则参照 FAST-LIO ROS2 分支 launch 文件新增）并指定 `config/sw01_sim.yaml` 为参数文件。
2. 安装 FAST-LIO2 依赖（`livox_ros_driver2` overlay）：参见 README 中 §5.3 WSL2 前置条件。
3. FAST-LIO2 输出 `/Odometry` 需 remap 到 `/odom`，并发布 TF 链路 `camera_init → body`；Nav2 需在导航参数中配合调整 `odom_frame`。

> 仓库名中的 "fastlio2" 指原始设计目标（FAST-LIO2 作为主 SLAM）；当前仿真验证路径以 slam_toolbox 为主，便于快速迭代 2D 导航功能。

## 5. WSL2 前置条件

以下命令是供操作员在自己的环境中执行的安装步骤；本文档不声称这些联网、`apt` 或 `sudo` 命令已在当前会话执行。

在 Windows PowerShell 检查 WSL2/WSLg：

```powershell
wsl --status
wsl --version
wsl --update
```

进入 Ubuntu 22.04 后确认发行版、ROS 和 GUI：

```bash
lsb_release -a
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null
gazebo --version
glxinfo -B
```

目标是 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。Windows 11 的 WSLg 通常直接提供 X/Wayland 和 GPU 转发；若 `$DISPLAY`/`$WAYLAND_DISPLAY` 为空，先修复 WSLg，不要把 Gazebo 黑屏误判成 launch 失败。

### 5.1 apt 依赖

若 ROS 2 Humble 尚未安装，先按 ROS 官方 Ubuntu deb 指南配置软件源，再由操作员手动执行：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-vcstool \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-velodyne-simulator \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  libpcl-dev libeigen3-dev libyaml-cpp-dev \
  mesa-utils
```

本清单是完整手动安装超集；`setup.sh` 自动安装的核心 apt 包（gazebo-ros/-ros-pkgs/-plugins、velodyne-simulator、nav2-bringup、slam-toolbox、robot-state-publisher、xacro、tf2-ros、rviz2、libpcl-dev、libeigen3-dev、python3-colcon-common-extensions、python3-rosdep）与本清单同名一致。`ros-humble-velodyne-simulator` 提供 `velodyne_description` 与 `velodyne_gazebo_plugins` 两个 ROS 包，不存在 `ros-humble-velodyne-gazebo-plugins` 这个 deb 名。

若 `rosdep` 从未初始化，由操作员执行一次；已经初始化时不要重复 `rosdep init`：

```bash
sudo rosdep init
rosdep update
```

### 5.2 Python 工具

运行脚本需要 NumPy/Matplotlib，静态测试还需要 pytest/PyYAML：

```bash
python3 -m pip install --user numpy matplotlib pytest PyYAML
```

系统 apt 中的 `python3-numpy`、`python3-matplotlib` 也由 package manifests 通过 rosdep 声明；不要同时用不同 Python 解释器运行 ROS 和这些库。

### 5.3 官方 Livox-SDK2 与 livox_ros_driver2

本次仿真不连接 Livox 硬件，但本地 `fast_lio/package.xml` 无条件引用 `livox_ros_driver2` 消息，所以必须构建它。按两个官方仓库的 Humble 路径操作：

```bash
cd "$HOME"
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd "$HOME/Livox-SDK2"
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig

mkdir -p "$HOME/ws_livox/src"
git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
  "$HOME/ws_livox/src/livox_ros_driver2"
cd "$HOME/ws_livox/src/livox_ros_driver2"
source /opt/ros/humble/setup.bash
./build.sh humble
source "$HOME/ws_livox/install/setup.bash"
ros2 pkg prefix livox_ros_driver2
```

`sudo make install` 是 Livox-SDK2 官方安装步骤，由操作员明确执行；本仓库没有自动安装脚本。

## 6. 创建本地 FAST-LIO2 软链、rosdep 与构建

下面所有命令与你克隆本仓库的位置无关：把 `$WS` 替换为你的克隆路径即可（`setup.sh` 也会从自身位置推导工作目录，不依赖任何硬编码路径）：

```bash
WS=/path/to/your/clone/sw01-fastlio2-sim   # 例如 /home/user/sw01-fastlio2-sim
cd "$WS"
```

进入后创建软链。软链目标 `/mnt/d/slam/自研/fast-lio2` 是本地 FAST-LIO2 检出的实际 WSL 路径；如果你的检出在其他位置，把该目标换成你的路径即可。路径含空格和中文时引号不可省略：

```bash
cd "$WS"
ln -s "/mnt/d/slam/自研/fast-lio2" "fast_lio2"
ln -s "../fast_lio2/slam/src/x30/sensing/FAST_LIO" "src/FAST_LIO"
readlink -f "fast_lio2"
readlink -f "src/FAST_LIO"
```

第二个目标刻意使用实际嵌套路径 `slam/src/x30/sensing/FAST_LIO`；不要误链到仓库根。若链接已经存在且正确，不要重复创建；先用 `readlink -f` 核对。

构建主工作空间：

```bash
cd "$WS"
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
ros2 pkg prefix livox_ros_driver2
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys livox_ros_driver2
colcon build --symlink-install \
  --base-paths src \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg prefix fast_lio
ros2 pkg prefix sw01_gazebo
```

`livox_ros_driver2` 没有 Humble rosdep 系统键；前一步已用 `ros2 pkg prefix` 验证官方 overlay，因此这里显式 skip 该键，而不是掩盖一个未知依赖。`--base-paths src` 也很重要：根级 `fast_lio2` 软链只用于保持 `src/FAST_LIO` 的目标可读，若让 colcon 从工作空间根递归发现，它会把本地 x30 仓库的其他非本任务包一并加入构建。

每个新终端都要依次 source ROS、Livox overlay 和本工作空间；`ros2 pkg prefix fast_lio` 应指向本工作空间的 `install/fast_lio`。

## 7. 分步启动：三个终端

先完成第 6 节构建。在每个终端设置相同环境：

```bash
cd "$WS"
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source install/setup.bash
```

### 终端 1，T+0 s：仿真和机器人

```bash
ros2 launch sw01_gazebo simulation.launch.py
```

等待 Gazebo 完成 world 和实体加载，确认 `/clock` 开始推进。

### 终端 2，T+5 s：FAST-LIO2、投影、slam_toolbox、RViz

在终端 1 启动约 5 秒后：

```bash
ros2 launch sw01_slam slam.launch.py
```

无 GUI 时可用：

```bash
ros2 launch sw01_slam slam.launch.py rviz:=false
```

### 终端 3，T+12 s：Nav2

在终端 1 启动约 12 秒后：

```bash
ros2 launch sw01_navigation navigation.launch.py
```

0/5/12 秒只是进程顺序，不证明系统就绪；发送目标前还要检查 TF、`/map` 和 lifecycle。

## 8. 一键启动与覆盖参数

默认一键命令：

```bash
ros2 launch sw01_gazebo full_demo.launch.py
```

默认 world 为已安装的 `sw01_maze.world`，起点 `x=-12.0 y=-12.0 z=0.62 yaw=0.0`，`gui=true`、`rviz=true`、`use_sim_time=true`。一键入口可覆盖起点、world、gui、rviz：

```bash
ros2 launch sw01_gazebo full_demo.launch.py \
  world:="/absolute/path/custom.world" \
  x:=-10.0 y:=-11.0 z:=0.62 yaw:=1.5708 \
  gui:=false rviz:=false use_sim_time:=true
```

查看当前代码实际提供的参数，不要猜参数名：

```bash
ros2 launch sw01_gazebo simulation.launch.py --show-args
ros2 launch sw01_slam slam.launch.py --show-args
ros2 launch sw01_navigation navigation.launch.py --show-args
ros2 launch sw01_gazebo full_demo.launch.py --show-args
```

## 9. 发送默认或自定义目标

先确认 `/planner_server`、`/controller_server` 为 `active`。默认终点来自迷宫蓝色终点区 `(12, 12, yaw=0)`：

```bash
ros2 run sw01_navigation send_nav_goal.py --ros-args -p use_sim_time:=true
```

自定义 map 坐标目标：

```bash
ros2 run sw01_navigation send_nav_goal.py --x 8.0 --y 11.0 --yaw 1.5708 --ros-args -p use_sim_time:=true
```

完整默认值显式写法：

```bash
ros2 run sw01_navigation send_nav_goal.py --x 12 --y 12 --yaw 0 --frame map --ros-args -p use_sim_time:=true
```

目标发送节点用自身 ROS clock 生成 goal header stamp；整套仿真以 Gazebo `/clock` 为时间源，因此每条可复制命令都显式传入 `use_sim_time:=true`。自定义参数放在 `--ros-args` 之前，ROS 参数放在其后。脚本会报告 goal accepted/rejected、剩余距离、预计时间、recovery 次数和 succeeded/canceled/aborted 终态，并以不同退出码区分失败。

## 10. 评估 ATE/RTE

让机器人运动后，在独立终端同时采样 FAST-LIO2 `/Odometry` 与 Gazebo `/ground_truth/odom`：

```bash
ros2 run sw01_navigation evaluate_slam.py \
  --duration 120 \
  --output-dir /tmp/sw01_eval
ls -l /tmp/sw01_eval/trajectory_samples.csv \
      /tmp/sw01_eval/trajectory_comparison.png
```

评估器用时间戳最近邻配对（默认最大差 0.05 s），做无尺度 SE(2) SVD 对齐，然后输出：

- ATE RMSE/mean/median/max：全局对齐后的平移误差；
- RTE (1 s) translation RMSE/mean 与 yaw RMSE：约一秒间隔的相对运动误差；
- `trajectory_samples.csv` 和 `trajectory_comparison.png`。

建议调优目标是 **ATE RMSE < 0.5 m**、**1 秒平移 RTE RMSE < 0.2 m**。它们是 120 s 迷宫轨迹的调优目标，不是构建门槛；没有真实运行数据时不得宣称达标。

## 11. 运行检查命令

### 11.1 话题、频率、类型和内容

```bash
ros2 topic list -t
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic hz /ground_truth/odom
ros2 topic hz /Odometry
ros2 topic hz /scan
ros2 topic hz /map

ros2 topic info -v /velodyne_points
ros2 topic info -v /imu/data
ros2 topic info -v /ground_truth/odom
ros2 topic info -v /Odometry
ros2 topic info -v /scan
ros2 topic info -v /map

ros2 topic echo --once /velodyne_points
ros2 topic echo /velodyne_points --once --field fields
ros2 topic echo --once /imu/data
ros2 topic echo --once /ground_truth/odom
ros2 topic echo --once /Odometry
ros2 topic echo --once /scan
ros2 topic echo --once /map
```

期望约 10/200/50 Hz 的三路 Gazebo 传感器数据，且 `/Odometry`、`/scan`、`/map` 非空。PointCloud2 `fields` 至少核对 `x`、`y`、`z`、`intensity`、`ring`，并按第 3 节规则解释 `time`。

### 11.2 TF、仿真时间与 lifecycle

```bash
ros2 topic hz /clock
ros2 topic info -v /tf
ros2 topic info -v /tf_static
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_tools view_frames

ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /waypoint_follower
ros2 lifecycle get /velocity_smoother
```

生命周期由官方 `navigation_launch.py` 创建的 `lifecycle_manager_navigation` 统一拥有；其管理列表为 `controller_server`、`smoother_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother`。`planner_server` 和 `controller_server` 至少必须是 `active`；完整导航还要求其余 Nav2 lifecycle servers 为 `active`。

### 11.3 描述、world 与静态测试

这些 ROS/Gazebo CLI 命令在 WSL 中运行：

```bash
cd "$WS"   # $WS 是你的克隆路径，见第 6 节
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source install/setup.bash
xacro src/sw01_description/urdf/sw01_sim.urdf.xacro > /tmp/sw01_sim.urdf
check_urdf /tmp/sw01_sim.urdf
gz sdf -k src/sw01_gazebo/worlds/sw01_maze.world
python3 -m pytest -v
python3 -m py_compile src/*/launch/*.py src/sw01_navigation/scripts/*.py
```

## 12. 故障排查

### 12.1 TF 丢失或重复

- `map -> camera_init` 缺失：检查 `ros2 node info /slam_toolbox`、`/scan` 是否有数据、slam_toolbox 是否采用仿真时间。
- `camera_init -> body` 缺失：FAST-LIO2 往往尚未完成 IMU 初始化；检查 LiDAR/IMU 频率、时间戳和配置话题。
- `body -> base_link` 缺失：检查 `static_transform_publisher` 进程与 `simulation.launch.py`。
- `body` 出现多个父节点：确认 URDF 没有 `body` link，P3D/Planar Move 没有发布 odom TF，`ros2 topic info -v /tf` 没有额外发布者。
- extrapolation/future/past：先检查 `/clock` 和所有相关节点的 `use_sim_time`，不要只增大 tolerance。

### 12.2 仿真时间不推进或混用墙钟

```bash
ros2 topic hz /clock
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /laser_mapping use_sim_time
ros2 param get /slam_toolbox use_sim_time
ros2 param get /planner_server use_sim_time
ros2 param get /smoother_server use_sim_time
```

若 Gazebo pause，`/clock` 也会停。FAST-LIO2 节点名由 `slam.launch.py` 显式固定为 `/laser_mapping`（与 executable 名 `fastlio_mapping` 不同），因此上面的 `ros2 param get /laser_mapping ...` 无需猜测；如发现不同，说明 launch 或本地源码版本有变。

### 12.3 话题不匹配

- 用 `ros2 topic list -t` 和 `ros2 topic info -v` 同时核对名称、类型、publisher/subscriber 与 QoS。
- 本地 FAST-LIO2 必须读取 `/velodyne_points`、`/imu/data`；检查 `common.lid_topic`、`common.imu_topic` 的实际参数。
- `/cloud_registered` 必须进入投影节点的 `cloud_in`，输出 remap 为 `/scan`。
- 不要把 `/Laser_map` 投影为 `/scan`，否则累积历史点会形成障碍重影。

### 12.4 Velodyne ring/time 字段

```bash
ros2 topic echo /velodyne_points --once --field fields
ros2 topic info -v /velodyne_points
```

- `ring` 缺失：`lidar_type: 2` 契约不成立；不要静默继续，先修复 `velodyne_gazebo_plugins` 输出或明确改造 FAST-LIO2 预处理。
- `time` 缺失或全零：本地已审计代码可按 ring、方位角和 `scan_rate` 合成相对时间；仍需确认 `scan_line=16`、`scan_rate=10` 与传感器一致。
- 日志出现 field mismatch 时，先以实际 `fields` 输出和本地 `preprocess.cpp/.h` 为准，不照搬其他 fork 的字段约定。

### 12.5 LiDAR 插件加载失败

```bash
ros2 pkg prefix velodyne_gazebo_plugins
find "$(ros2 pkg prefix velodyne_gazebo_plugins)" \
  -name libgazebo_ros_velodyne_laser.so -print
ldd "$(find "$(ros2 pkg prefix velodyne_gazebo_plugins)" \
  -name libgazebo_ros_velodyne_laser.so -print -quit)"
printf '%s\n' "$GAZEBO_PLUGIN_PATH"
gzserver --verbose src/sw01_gazebo/worlds/sw01_maze.world
```

若库不存在，核对 `ros-humble-velodyne-gazebo-plugins` 是否安装；若 `ldd` 有 `not found`，修复动态库路径/缺包。不要静默换成 generic ray PointCloud2 插件，因为其字段未必提供 `ring`。

### 12.6 FAST-LIO2、空 scan 或空 map

- FAST-LIO2 无输出：检查 `/imu/data` 约 200 Hz、`/velodyne_points` 约 10 Hz、两者 header stamp 随 `/clock` 单调推进；检查外参数组长度与 `blind=0.30` 没有过滤全部点。
- `/scan` 空：确认 `/cloud_registered` 非空且有 `/scan` subscriber；检查同一时间的 `camera_init -> body`；再检查 `min_height=-0.05`、`max_height=0.45` 是否适配实际点高。
- `/map` 空：先保证 `/scan` 的 `ranges` 包含有限值，再检查 slam_toolbox 的 `map/camera_init/body` TF 链和 lifecycle 状态。
- 用 `ros2 topic echo --once /scan` 与 `ros2 topic echo --once /map` 区分“没有消息”和“消息为空”。

### 12.7 Nav2 lifecycle 或 footprint 堵塞

- `ros2 lifecycle get` 不是 `active`：查看对应 server 日志，并检查 `map -> camera_init -> body -> base_link`、`/map`、`/scan`。
- 有 `/cmd_vel` 但 Gazebo 不动：检查 `libgazebo_ros_planar_move.so` 是否加载以及 `/cmd_vel` remap。
- 没有 `/cmd_vel`：检查 action 是否 accepted、global/local costmap 是否更新、planner/controller 日志。
- footprint 使用实际外廓八点近似（约 0.92 m × 0.54 m）和 0.45 m inflation。若通道被全部膨胀，先查看 costmap；不能把 footprint 缩到小于真实模型以“穿墙”。

迷宫墙线含三处 1.0 m 窄口（墙中心距，净宽 0.8 m）：`wall_h07`/`wall_h08` 之间（y=4）、`wall_v01`/`wall_v02` 之间（x=-8）、`wall_v04`/`wall_v05` 之间（x=-2）。在 0.45 m inflation 与 0.92 m 足迹下这三处不可通行；全局存在东侧绕行路线，由 `wall_h02` 东端、`wall_v08` 顶端、`wall_h06` 东端、`wall_h11` 东端组成，通道净宽 ≥ 2.1 m（静态测试同时用 0.50 m 与 0.91 m 两种膨胀验证起终点连通，见 `tests/test_world_contract.py`）。若全局规划反复绕进窄口或 costmap 在窄口处出现断点，检查 world 是否被覆盖为 `full_demo.launch.py` 的 `world:=` 参数指向的旧版本。

### 12.8 WSL GUI / WSLg

- `gazebo` 或 `rviz2` 无窗口：检查 `$DISPLAY`、`$WAYLAND_DISPLAY`、`wsl --version`，更新 WSLg 后执行 `wsl --shutdown` 再进入。
- OpenGL 报错：用 `glxinfo -B` 检查 renderer；诊断时可临时试 `LIBGL_ALWAYS_SOFTWARE=1 gazebo`，但软件渲染会显著降低传感器实时率。
- 远程/无 GUI 验证：使用 `gui:=false rviz:=false`；这只能验证 headless ROS/Gazebo 路径，不能代替可视化检查模型是否穿地或塌腿。

## 13. Windows 静态验证与 WSL 运行验证边界

### Windows 静态验证能证明

- Python 语法和 pytest 契约通过；
- XML/YAML 可解析，launch/配置含预期接口；
- 17 个 STL 的尺寸与 SHA-256 匹配审计清单，且全部以 `package://sw01_description/meshes/` 引用并真实存在；
- 迷宫墙体拓扑、三处 1.0 m 窄口、东侧绕行通道，以及 0.50 m/0.91 m 两种膨胀后 BFS 路径存在；
- setup.sh 语义（apt 续行无注释、`--skip-keys livox_ros_driver2`、`--base-paths src`、幂等 rosdep init）；
- package.xml 导出 `gazebo_model_path`/`gazebo_media_path`，spawn 使用 `-package_to_model`，安装后无需手工 GAZEBO_MODEL_PATH；
- manifests/CMake 安装规则和直接依赖已声明；
- 源码静态审计未发现 Gazebo odom/TF 重复发布配置。

### 必须在 WSL 运行验证才能证明

- `rosdep` 能解析当前机器已配置的软件源，`colcon build` 能链接本地 FAST-LIO2 与 Livox 消息；
- `xacro`、`check_urdf`、`gz sdf -k` 使用实际 Humble/Classic 11 工具成功；
- 插件真的加载、频率和 PointCloud2 fields 正确；
- TF 在运行时间上连通且无重复 publisher；
- `/scan`、`/map` 非空，Nav2 lifecycle active，目标成功到达；
- 评估确实生成 CSV/PNG 并达到或未达到 ATE/RTE 调优目标。

任何缺包、GUI 不可用或 ROS 环境未 source 导致的非零退出码都应原样记录，不能记为“通过”。

## 14. 联网参考资料与采用结论

本节记录实现时的四类联网检索及采用边界；链接是直接官方文档或原仓库 URL。参考实现可能变化，运行接口、参数、字段和 TF 的最终结论 **最终以本地源码为准**。

1. **ROS 2 FAST-LIO 仓库**
   - <https://github.com/MIT-SPARK/spark-fast-lio>
   - <https://github.com/rohrschacht/FAST_LIO_SLAM_ros2>
   - 采用结论：仅用于交叉核对 ROS 2 launch、PointCloud2/IMU 接口和 Velodyne 时间字段注意事项；本工程仍使用本地 `fast_lio/fastlio_mapping`、固定话题和固定 TF，不复制这些 fork。

2. **Nav2 mapping 官方文档**
   - <https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html>
   - <https://docs.nav2.org/setup_guides/sensors/mapping_localization.html>
   - 采用结论：按“边建图边导航”启动 slam_toolbox 与 `navigation_launch.py`，不启动 AMCL/静态 map_server；`/map` 与 `map -> camera_init` 由 SLAM 提供。

3. **SDFormat world/shapes、Gazebo ROS URDF 与点云投影**
   - <https://sdformat.org/spec?ver=1.6&elem=geometry>
   - <https://sdformat.org/spec?ver=1.6&elem=world>
   - <https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-a-URDF-in-Gazebo.html>
   - <https://github.com/ros-perception/pointcloud_to_laserscan>
   - 采用结论：world 使用 SDF 1.6 的 world/box 结构，机器人由 URDF/Xacro 加 Gazebo 扩展生成；当前注册点云投影到 `body` frame 的 `/scan`，不投影累积 `/Laser_map`。

4. **四足参考 CHAMP / Unitree Go2 ROS 2**
   - <https://github.com/chvmp/champ>
   - <https://github.com/unitreerobotics/unitree_ros2>
   - 采用结论：这些资料说明真实四足通常需要 gait/controller、关节状态和硬件接口。本任务明确不实现该层，只借鉴模型分层思路，采用固定站姿 + Planar Move 来隔离导航闭环与步态控制问题。

另外，Livox 编译步骤直接依据：

- <https://github.com/Livox-SDK/Livox-SDK2>
- <https://github.com/Livox-SDK/livox_ros_driver2>

这些网络资料和命令清单不等于已执行记录；实际执行结果应以本机 WSL 命令、退出码和日志为准。
