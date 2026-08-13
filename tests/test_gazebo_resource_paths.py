import re

from conftest import SRC, parse_xml


URDF = SRC / "sw01_description" / "urdf" / "sw01_sim.urdf.xacro"


def gazebo_ros_exports(package_name):
    root = parse_xml(SRC / package_name / "package.xml")
    return [node.attrib for node in root.findall("export/gazebo_ros")]


def test_description_exports_gazebo_model_and_media_paths():
    """安装后 Humble gzserver.launch.py 的 GazeboRosPaths 读取这些导出并自动加入
    GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH，用户无需手工设置环境变量。
    ${prefix} 会被替换为包的 share 目录，${prefix}/../ 使
    model://sw01_description/meshes/... 能经 GAZEBO_MODEL_PATH 解析。"""
    exports = gazebo_ros_exports("sw01_description")
    assert {"gazebo_model_path": "${prefix}/../"} in exports
    assert {"gazebo_media_path": "${prefix}"} in exports


def test_gazebo_package_exports_gazebo_model_and_media_paths():
    exports = gazebo_ros_exports("sw01_gazebo")
    assert {"gazebo_model_path": "${prefix}/../"} in exports
    assert {"gazebo_media_path": "${prefix}"} in exports


def test_simulation_spawn_converts_package_mesh_uris_to_model_uris():
    """Humble 无 package:// 插件；spawn_entity.py 必须带 -package_to_model，
    才能把 mesh 的 package:// URI 转为 model:// 并命中导出的 GAZEBO_MODEL_PATH。"""
    launch = (SRC / "sw01_gazebo" / "launch" / "simulation.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"-package_to_model"' in launch
    assert "spawn_entity.py" in launch


def test_all_17_mesh_references_are_package_uris_and_resolve_to_installed_files():
    """17 个 STL 必须全部以 package://sw01_description/meshes/ 引用且真实存在于包内，
    确保 gazebo_ros 的 package:// 解析与安装位置无关。"""
    xml = URDF.read_text(encoding="utf-8")
    references = re.findall(r'<mesh filename="([^"]+)"', xml)
    # 17 个 link 各有一份 visual 和一份 collision mesh 引用。
    assert len(references) == 34
    unique_references = set(references)
    assert len(unique_references) == 17
    for reference in unique_references:
        assert reference.startswith("package://sw01_description/meshes/")
        filename = reference[len("package://sw01_description/meshes/") :]
        assert (SRC / "sw01_description" / "meshes" / filename).is_file()
