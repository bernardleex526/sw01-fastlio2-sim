import ast
import re


from conftest import ROOT


GOAL_SCRIPT = ROOT / "src" / "sw01_navigation" / "scripts" / "send_nav_goal.py"
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
        r"DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)",
        cmake,
        flags=re.DOTALL,
    )
    assert not re.search(
        r"install\(\s*DIRECTORY\b[^)]*\bscripts\b", cmake, flags=re.DOTALL
    )
