import ast


from conftest import ROOT


GOAL_SCRIPT = ROOT / "src" / "sw01_navigation" / "scripts" / "send_nav_goal.py"


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
