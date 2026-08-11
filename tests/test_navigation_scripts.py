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


def test_goal_sender_separates_ros_arguments_before_argparse_and_reuses_ros_argv():
    """ROS 参数若进入 argparse，或 init 未收到同一完整 argv，仿真时钟目标会失败。"""
    tree = ast.parse(GOAL_SCRIPT.read_text(encoding="utf-8"))
    main = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "main"
    )

    calls = [node for node in ast.walk(main) if isinstance(node, ast.Call)]
    remove_call = next(
        node
        for node in calls
        if isinstance(node.func, ast.Name) and node.func.id == "remove_ros_args"
    )
    parse_call = next(
        node
        for node in calls
        if isinstance(node.func, ast.Name) and node.func.id == "parse_arguments"
    )
    init_call = next(
        node
        for node in calls
        if isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "rclpy"
        and node.func.attr == "init"
    )

    assert remove_call.lineno < parse_call.lineno < init_call.lineno
    assert len(remove_call.keywords) == 1 and remove_call.keywords[0].arg == "args"
    assert isinstance(remove_call.keywords[0].value, ast.Name)
    ros_argv_name = remove_call.keywords[0].value.id
    assert ros_argv_name == "ros_argv"
    assert len(parse_call.args) == 1
    assert isinstance(parse_call.args[0], ast.Name)
    assert parse_call.args[0].id == "non_ros_argv"
    init_args = next(keyword.value for keyword in init_call.keywords if keyword.arg == "args")
    assert isinstance(init_args, ast.Name) and init_args.id == ros_argv_name


def test_goal_sender_keeps_parse_arguments_independently_callable():
    """自定义参数解析必须继续接受不含程序名的 argv，供 main 和单测复用。"""
    tree = ast.parse(GOAL_SCRIPT.read_text(encoding="utf-8"))
    parser = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "parse_arguments"
    )

    assert [argument.arg for argument in parser.args.args] == ["argv"]
    parse_args_call = next(
        node
        for node in ast.walk(parser)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "parse_args"
    )
    assert len(parse_args_call.args) == 1
    assert isinstance(parse_args_call.args[0], ast.Name)
    assert parse_args_call.args[0].id == "argv"


def test_goal_sender_uses_humble_logger_single_message_api():
    """Humble RcutilsLogger 不接受 printf 风格额外位置参数，入口不得提前崩溃。"""
    tree = ast.parse(GOAL_SCRIPT.read_text(encoding="utf-8"))
    logging_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"debug", "info", "warning", "error", "fatal"}
    ]

    assert logging_calls
    assert all(len(call.args) == 1 for call in logging_calls)


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
