from conftest import ROOT


SETUP = ROOT / "setup.sh"


def _setup_text():
    return SETUP.read_text(encoding="utf-8")


def test_setup_computes_workspace_dir_from_its_own_location():
    """工作目录必须由脚本自身位置推导，与 clone 位置无关。"""
    text = _setup_text()
    assert "set -e" in text
    assert 'WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"' in text


def test_apt_install_block_has_no_comments_inside_line_continuation():
    """反斜杠续行块内的 # 注释会吞掉后续包名，并让下一行变成独立命令。"""
    lines = _setup_text().splitlines()
    inside_continuation = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("sudo apt-get install"):
            inside_continuation = True
            continue
        if inside_continuation:
            assert not stripped.startswith("#"), (
                f"comment inside apt continuation: {stripped!r}"
            )
            if stripped.endswith("\\"):
                continue
            inside_continuation = False
    assert not inside_continuation

    text = _setup_text()
    for package in (
        "ros-humble-gazebo-ros",
        "ros-humble-gazebo-ros-pkgs",
        "ros-humble-gazebo-plugins",
        "ros-humble-velodyne-simulator",
        "ros-humble-nav2-bringup",
        "ros-humble-slam-toolbox",
        "ros-humble-robot-state-publisher",
        "ros-humble-xacro",
        "ros-humble-tf2-ros",
        "ros-humble-rviz2",
        "libpcl-dev",
        "libeigen3-dev",
        "python3-colcon-common-extensions",
        "python3-rosdep",
    ):
        assert package in text


def test_setup_rosdep_skips_the_external_livox_key():
    """本地 fast_lio 无条件依赖 livox_ros_driver2，而它没有 Humble rosdep 键，必须显式跳过。"""
    assert (
        "rosdep install --from-paths src --ignore-src -r -y --skip-keys livox_ros_driver2"
        in _setup_text()
    )


def test_setup_colcon_limits_discovery_to_src():
    """根级 fast_lio2 软链不得让 colcon 递归发现并构建其他非本任务包。"""
    assert (
        "colcon build --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release"
        in _setup_text()
    )


def test_setup_rosdep_init_is_idempotent():
    text = _setup_text()
    assert "/etc/ros/rosdep/sources.list.d/20-default.list" in text
    assert "sudo rosdep init" in text
    assert "rosdep update" in text


def test_setup_apt_packages_match_readme_dependency_docs():
    """setup.sh 的 apt 清单必须与 README 依赖说明一致（同一 deb 包名）。"""
    text = _setup_text()
    apt_packages = {
        line.strip().rstrip(" \\")
        for line in text.splitlines()
        if line.strip().startswith(("ros-humble-", "lib", "python3-"))
    }
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for package in apt_packages:
        assert package in readme, f"apt 包 {package} 未在 README 依赖说明中出现"
