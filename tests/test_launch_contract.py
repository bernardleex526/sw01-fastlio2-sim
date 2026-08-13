import ast

from conftest import SRC


LAUNCHES = {
    "simulation": SRC / "sw01_gazebo/launch/simulation.launch.py",
    "slam": SRC / "sw01_slam/launch/slam.launch.py",
    "navigation": SRC / "sw01_navigation/launch/navigation.launch.py",
    "full": SRC / "sw01_gazebo/launch/full_demo.launch.py",
}
RVIZ = SRC / "sw01_slam/rviz/sw01_sim.rviz"


def test_launch_files_are_valid_python_and_export_generate_launch_description():
    """Catches a missing or syntactically invalid public launch entry point."""
    for path in LAUNCHES.values():
        source = path.read_text(encoding="utf-8")
        ast.parse(source)
        assert "def generate_launch_description" in source


def test_simulation_quotes_the_xacro_path_as_one_shell_argument():
    """Catches shlex splitting an installed Xacro path that contains spaces."""
    tree = ast.parse(LAUNCHES["simulation"].read_text(encoding="utf-8"))
    assignment = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "robot_description"
            for target in node.targets
        )
    )
    command = assignment.value

    assert isinstance(command, ast.Call)
    assert isinstance(command.func, ast.Name) and command.func.id == "Command"
    assert len(command.args) == 1 and isinstance(command.args[0], ast.List)
    parts = command.args[0].elts
    assert len(parts) == 4
    executable = parts[0]
    assert isinstance(executable, ast.Call)
    assert (
        isinstance(executable.func, ast.Name)
        and executable.func.id == "FindExecutable"
    )
    assert executable.args == []
    assert len(executable.keywords) == 1
    assert executable.keywords[0].arg == "name"
    assert (
        isinstance(executable.keywords[0].value, ast.Constant)
        and executable.keywords[0].value.value == "xacro"
    )
    assert isinstance(parts[1], ast.Constant) and parts[1].value == ' "'
    assert isinstance(parts[2], ast.Name) and parts[2].id == "xacro_file"
    assert isinstance(parts[3], ast.Constant) and parts[3].value == '"'


def test_fast_lio_node_name_is_explicitly_pinned_to_local_source():
    """本地源码为 Node("laser_mapping")；launch 必须显式固定，避免随 executable 名漂移。"""
    slam = LAUNCHES["slam"].read_text(encoding="utf-8")

    assert 'name="laser_mapping"' in slam
    assert "fastlio_mapping" in slam


def test_runtime_ownership_is_unambiguous():
    """Catches duplicate owners or a disconnected simulation/SLAM/Nav2 graph."""
    simulation = LAUNCHES["simulation"].read_text(encoding="utf-8")
    slam = LAUNCHES["slam"].read_text(encoding="utf-8")
    navigation = LAUNCHES["navigation"].read_text(encoding="utf-8")
    full = LAUNCHES["full"].read_text(encoding="utf-8")

    assert "spawn_entity.py" in simulation
    assert "body" in simulation and "base_link" in simulation
    assert "fastlio_mapping" in slam and "/cloud_registered" in slam and "/scan" in slam
    assert "async_slam_toolbox_node" in slam
    assert "navigation_launch.py" in navigation
    assert "amcl" not in navigation.lower() and "map_server" not in navigation.lower()
    assert "lifecycle_manager" not in navigation.lower()
    assert "TimerAction" in full and "5.0" in full and "12.0" in full


def test_navigation_can_apply_its_optional_namespace():
    """Catches a namespace argument that rewrites params but never scopes Nav2 nodes."""
    navigation = LAUNCHES["navigation"].read_text(encoding="utf-8")

    assert "PushRosNamespace" in navigation
    assert "IfCondition(use_namespace)" in navigation


def test_slam_projects_registered_cloud_with_the_approved_scan_geometry():
    """Catches scan projection limits that disagree with SLAM and Nav2 consumers."""
    slam = LAUNCHES["slam"].read_text(encoding="utf-8")

    for contract_value in (
        "target_frame",
        "body",
        "min_height",
        "-0.05",
        "max_height",
        "0.45",
        "angle_min",
        "-3.141592653589793",
        "angle_max",
        "3.141592653589793",
        "angle_increment",
        "0.008726646",
        "range_min",
        "0.30",
        "range_max",
        "30.0",
        "transform_tolerance",
        "0.10",
    ):
        assert contract_value in slam


def test_rviz_exposes_mapping_localization_and_navigation_outputs():
    """Catches an RViz setup that hides a required runtime output."""
    source = RVIZ.read_text(encoding="utf-8")

    for display_contract in (
        "Fixed Frame: map",
        "/map",
        "/cloud_registered",
        "/path",
        "RobotModel",
        "TF",
    ):
        assert display_contract in source


def test_rviz_documents_runtime_contract_sources_in_chinese():
    """Catches undocumented RViz topics, frames, or display values."""
    source = RVIZ.read_text(encoding="utf-8")

    assert source.count("# 来源：") >= 10
    for contract_line in (
        "Fixed Frame: map",
        "Value: /map",
        "Value: /cloud_registered",
        "Value: /path",
        "Value: /Odometry",
        "Value: /plan",
        "Value: /local_plan",
    ):
        line = next(line for line in source.splitlines() if contract_line in line)
        assert "# 来源：" in line
