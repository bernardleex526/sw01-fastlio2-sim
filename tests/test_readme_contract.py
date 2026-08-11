from conftest import ROOT, SRC, parse_xml


README = ROOT / "README.md"


def test_readme_contains_reproducible_install_build_and_launch_commands():
    """缺少从依赖安装到启动的任一关键步骤时，运行手册契约必须失败。"""
    text = README.read_text(encoding="utf-8")
    required = [
        "ros-humble-gazebo-ros-pkgs",
        "ros-humble-velodyne-gazebo-plugins",
        "ros-humble-navigation2",
        "ros-humble-slam-toolbox",
        "ros-humble-pointcloud-to-laserscan",
        "Livox-SDK2",
        "livox_ros_driver2",
        "colcon build --symlink-install",
        "simulation.launch.py",
        "slam.launch.py",
        "navigation.launch.py",
        "full_demo.launch.py",
        "send_nav_goal.py",
        "evaluate_slam.py",
        "TF 丢失",
        "LiDAR 插件加载失败",
        "话题不匹配",
    ]
    for item in required:
        assert item in text


def test_readme_documents_actual_sources_interfaces_and_runtime_boundary():
    """若手册虚化本地源码事实或混淆 Windows/WSL 验证边界，契约必须失败。"""
    text = README.read_text(encoding="utf-8")
    required = [
        "fast_lio",
        "fastlio_mapping",
        "17 个 link",
        "16 个 joint",
        "Planar Move",
        "Camera",
        "TF 唯一发布者",
        "参数来源",
        "Windows 静态验证",
        "WSL 运行验证",
        "不代表 Gazebo/ROS 运行成功",
        "最终以本地源码为准",
    ]
    for item in required:
        assert item in text


def test_readme_locks_symlinks_observability_and_reference_urls():
    """若软链层级、诊断命令或资料映射退化，操作员将无法复现或定位问题。"""
    text = README.read_text(encoding="utf-8")
    required = [
        'ln -s "/mnt/d/slam/自研/fast-lio2" "fast_lio2"',
        'ln -s "../fast_lio2/slam/src/x30/sensing/FAST_LIO" "src/FAST_LIO"',
        "ros2 topic hz /velodyne_points",
        "ros2 topic info -v /velodyne_points",
        "ros2 topic echo --once /velodyne_points",
        "ros2 run tf2_ros tf2_echo map base_link",
        "ros2 lifecycle get /planner_server",
        "ros2 lifecycle get /smoother_server",
        "ros2 param get /smoother_server use_sim_time",
        "lifecycle_manager_navigation",
        "x/y/z/intensity/time/ring",
        "https://github.com/MIT-SPARK/spark-fast-lio",
        "https://github.com/rohrschacht/FAST_LIO_SLAM_ros2",
        "https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html",
        "https://sdformat.org/spec?ver=1.6&elem=geometry",
        "https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-a-URDF-in-Gazebo.html",
        "https://github.com/ros-perception/pointcloud_to_laserscan",
        "https://github.com/chvmp/champ",
    ]
    for item in required:
        assert item in text


def test_readme_limits_colcon_discovery_and_handles_the_external_livox_key():
    """根级源码软链不得让主构建误扫全仓，外部 Livox 包也不得让 rosdep 假失败。"""
    text = README.read_text(encoding="utf-8")

    assert "--skip-keys livox_ros_driver2" in text
    assert "--base-paths src" in text


def test_every_goal_command_enables_gazebo_simulation_time():
    """可复制的目标命令若漏传 use_sim_time，goal stamp 会与 Gazebo TF 时间域冲突。"""
    text = README.read_text(encoding="utf-8")
    commands = [
        line.strip()
        for line in text.splitlines()
        if line.strip().startswith("ros2 run sw01_navigation send_nav_goal.py")
    ]

    assert len(commands) == 3
    assert all(
        command.endswith("--ros-args -p use_sim_time:=true")
        for command in commands
    )
    assert "目标发送节点" in text
    assert "Gazebo `/clock`" in text


def test_readme_distinguishes_created_fast_lio_endpoints_from_enabled_outputs():
    """README 不得把本地源码创建但默认门控关闭的点云端点写成活跃输出。"""
    text = README.read_text(encoding="utf-8")
    cloud_effected_line = next(
        line for line in text.splitlines() if line.startswith("| `/cloud_effected`")
    )
    laser_map_line = next(
        line for line in text.splitlines() if line.startswith("| `/Laser_map`")
    )
    path_line = next(
        line for line in text.splitlines() if line.startswith("| `/path`")
    )

    assert "源码创建/固定名称端点" in text
    assert "本配置实际启用输出" in text
    assert "`/cloud_registered`、`/cloud_registered_body`、`/Odometry`、`/path`" in text
    assert "默认禁用" in cloud_effected_line and "0 Hz" in cloud_effected_line
    assert "默认禁用" in laser_map_line and "0 Hz" in laser_map_line
    assert "≈ 1 Hz" in path_line and "每 10 帧" in path_line
    assert "publish.effect_map_en" in text
    assert "publish.map_en" in text
    assert "laserMapping.cpp" in text


def _exec_dependencies(package_name):
    root = parse_xml(SRC / package_name / "package.xml")
    return {node.text for node in root.findall("exec_depend")}


def test_package_manifests_declare_direct_runtime_dependencies():
    """删除 launch import、被 include 包、插件或脚本 import 的直接依赖时必须失败。"""
    expected = {
        "sw01_description": {
            "xacro",
            "robot_state_publisher",
            "gazebo_ros",
            "gazebo_plugins",
            "velodyne_gazebo_plugins",
        },
        "sw01_gazebo": {
            "ament_index_python",
            "launch",
            "launch_ros",
            "gazebo_ros",
            "gazebo_plugins",
            "velodyne_gazebo_plugins",
            "xacro",
            "robot_state_publisher",
            "tf2_ros",
            "sw01_description",
            "sw01_slam",
            "sw01_navigation",
        },
        "sw01_slam": {
            "ament_index_python",
            "launch",
            "launch_ros",
            "fast_lio",
            "pointcloud_to_laserscan",
            "slam_toolbox",
            "rviz2",
        },
        "sw01_navigation": {
            "ament_index_python",
            "launch",
            "launch_ros",
            "nav2_bringup",
            "nav2_msgs",
            "nav_msgs",
            "action_msgs",
            "rclpy",
            "python3-numpy",
            "python3-matplotlib",
        },
    }
    for package_name, dependencies in expected.items():
        assert dependencies <= _exec_dependencies(package_name)
