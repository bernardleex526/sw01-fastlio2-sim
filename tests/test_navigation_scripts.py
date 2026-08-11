import ast
import re


from conftest import ROOT


GOAL_SCRIPT = ROOT / "src" / "sw01_navigation" / "scripts" / "send_nav_goal.py"
EVAL_SCRIPT = ROOT / "src" / "sw01_navigation" / "scripts" / "evaluate_slam.py"
NAVIGATION_CMAKE = ROOT / "src" / "sw01_navigation" / "CMakeLists.txt"


def test_goal_sender_uses_nav2_action_and_maze_default():
    """缺少 Nav2 action、默认迷宫坐标或中断取消时必须失败。"""
    source = GOAL_SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)

    assert "NavigateToPose" in source
    assert "ActionClient" in source
    assert "navigate_to_pose" in source
    assert "--x" in source and "12.0" in source
    assert "--y" in source and "12.0" in source
    assert "distance_remaining" in source
    assert "cancel_goal_async" in source
    assert any(isinstance(node, ast.If) for node in ast.walk(tree))


def test_goal_sender_is_installed_as_a_ros_executable():
    """若改回 share 资源安装，ros2 run 将找不到目标发送器。"""
    cmake = NAVIGATION_CMAKE.read_text(encoding="utf-8")

    assert re.search(
        r"install\(\s*PROGRAMS\s+scripts/send_nav_goal\.py\s+"
        r"scripts/evaluate_slam\.py\s+"
        r"DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)",
        cmake,
        flags=re.DOTALL,
    )
    assert not re.search(
        r"install\(\s*DIRECTORY\b[^)]*\bscripts\b", cmake, flags=re.DOTALL
    )


def test_evaluator_uses_required_topics_metrics_and_outputs():
    """评估器缺少数据源、指标、产物或运行时参数时必须失败。"""
    source = EVAL_SCRIPT.read_text(encoding="utf-8")
    ast.parse(source)

    assert source.startswith("#!/usr/bin/env python3")
    assert '"/Odometry"' in source
    assert '"/ground_truth/odom"' in source
    assert source.count("qos_profile_sensor_data") >= 3
    assert "threading.Lock" in source
    assert "ExternalShutdownException" in source
    assert "synchronize_nearest" in source
    assert "align_se2" in source
    assert "apply_se2_to_poses" in source
    assert "compute_ate" in source and "compute_rte" in source
    assert "matplotlib" in source and "csv" in source
    assert "trajectory_samples.csv" in source
    assert "trajectory_comparison.png" in source
    assert "--duration" in source and "--output-dir" in source
    assert "0.05" in source


def test_evaluator_and_metrics_are_installed_together_for_ros2_run():
    """评估器与本地 metrics 未同目录安装时，ros2 run 后导入会失败。"""
    cmake = NAVIGATION_CMAKE.read_text(encoding="utf-8")

    assert re.search(
        r"install\(\s*PROGRAMS\s+scripts/send_nav_goal\.py\s+"
        r"scripts/evaluate_slam\.py\s+DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)",
        cmake,
        flags=re.DOTALL,
    )
    assert re.search(
        r"install\(\s*FILES\s+scripts/trajectory_metrics\.py\s+"
        r"DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)",
        cmake,
        flags=re.DOTALL,
    )
