import hashlib

from conftest import SRC, parse_xml


DESCRIPTION = SRC / "sw01_description"
URDF = DESCRIPTION / "urdf" / "sw01_sim.urdf.xacro"
MESHES = DESCRIPTION / "meshes"

EXPECTED_LINKS = {
    "base_link",
    "FL_HIP",
    "FL_THIGH",
    "FL_CALF",
    "FL_WHEEL",
    "FR_HIP",
    "FR_THIGH",
    "FR_CALF",
    "FR_WHEEL",
    "RL_HIP",
    "RL_THIGH",
    "RL_CALF",
    "RL_WHEEL",
    "RR_HIP",
    "RR_THIGH",
    "RR_CALF",
    "RR_WHEEL",
    "imu_link",
    "lidar_link",
}
EXPECTED_JOINTS = {
    "FL_HIP_JOINT",
    "FL_THIGH_JOINT",
    "FL_CALF_JOINT",
    "FL_WHEEL_JOINT",
    "FR_HIP_JOINT",
    "FR_THIGH_JOINT",
    "FR_CALF_JOINT",
    "FR_WHEEL_JOINT",
    "RL_HIP_JOINT",
    "RL_THIGH_JOINT",
    "RL_CALF_JOINT",
    "RL_WHEEL_JOINT",
    "RR_HIP_JOINT",
    "RR_THIGH_JOINT",
    "RR_CALF_JOINT",
    "RR_WHEEL_JOINT",
    "base_to_imu",
    "base_to_lidar",
}
EXPECTED_MESHES = {
    "base_link.STL": (
        15597784,
        "5b02d365f7fcdb80aacfcef780a21470377adb32cc3e8901727a0f16581956bc",
    ),
    "FL_CALF.STL": (
        1541084,
        "cc8b4557d6e04a6741653b68e0bccea15dc9d32fc4830313a8053ce24348a243",
    ),
    "FL_HIP.STL": (
        533084,
        "1ec5b1da803cf0e87ae0e433b557e28eb7c08fe96aaccfeedaa913e607e655c1",
    ),
    "FL_THIGH.STL": (
        2983384,
        "3cce7a6859c7a75874eaf46071d78466d6bc6d45b3b1c5511141d88ec17b2444",
    ),
    "FL_WHEEL.STL": (
        1243284,
        "9b7c2d2c98c25793d9e0fc23c5621f3f04f442f7191cdd3b5d2cfce62383609d",
    ),
    "FR_CALF.STL": (
        1543184,
        "417962b4809af98636f42eaf2dfd64f71ef3d57740fbd5c6d3dae15a48bbb609",
    ),
    "FR_HIP.STL": (
        533784,
        "f75d80720d197201374e60b0ddcf6690d9adaa8a015323a5bdfc3ff0584500c5",
    ),
    "FR_THIGH.STL": (
        2966484,
        "f283a911befe7169e7bc8a44e4c0c1c1d71d612c704eea8ccd2fabd89d767ad5",
    ),
    "FR_WHEEL.STL": (
        1229984,
        "b06f0fc124a5169d4b70d51806656658c39d688099300d37fe9431057dc727d9",
    ),
    "RL_CALF.STL": (
        1541084,
        "f617c28d43c4c80521490b0fd146c924f4d135a01422ae1f512ce823554eb02a",
    ),
    "RL_HIP.STL": (
        533084,
        "990f8380129bc1537a48c596e4f16d4ae60b0bbeecba7370577ee62b860dd3c3",
    ),
    "RL_THIGH.STL": (
        2983284,
        "e8a633cb3f50f015011d852a17796cd47c94a7a519cd31f100fef1c8a3e28b67",
    ),
    "RL_WHEEL.STL": (
        1243284,
        "8a9103327d89b576048e4b8ca59eff2fcdc6255d665d407ee62fd46fe3553e5a",
    ),
    "RR_CALF.STL": (
        1543184,
        "5693bcb2e0fc92b49b4d95d4027c346e6537bc73ab2501ea7a0366d9676cfec4",
    ),
    "RR_HIP.STL": (
        533784,
        "50d230a91eeb835a5ffd2ba0587d298ba5329d1b4863bf658cb4c62ca0e95dc9",
    ),
    "RR_THIGH.STL": (
        2966084,
        "0b66568599f045c4ea0ae55fb7a6861da2ca18b7800ba8e921c5933fee1977e4",
    ),
    "RR_WHEEL.STL": (
        1229984,
        "bdd34b41a727fafb1e506ee08a3a50362432fc0ff86acdd5247eb8d4bd95b9ba",
    ),
}


def test_description_preserves_wheel_leg_tree_without_body_link():
    root = parse_xml(URDF)
    links = {node.attrib["name"] for node in root.findall("link")}
    joints = {node.attrib["name"]: node.attrib["type"] for node in root.findall("joint")}
    assert links == EXPECTED_LINKS
    assert "body" not in links
    assert set(joints) == EXPECTED_JOINTS
    assert all(joints[name] == "fixed" for name in EXPECTED_JOINTS)


def test_gazebo_plugins_match_topic_and_rate_contract():
    root = parse_xml(URDF)
    xml = URDF.read_text(encoding="utf-8")
    for library in (
        "libgazebo_ros_velodyne_laser.so",
        "libgazebo_ros_imu_sensor.so",
        "libgazebo_ros_p3d.so",
        "libgazebo_ros_planar_move.so",
    ):
        assert root.find(f".//plugin[@filename='{library}']") is not None
    assert "/velodyne_points" in xml and "/imu/data" in xml
    assert "/ground_truth/odom" in xml and "/cmd_vel" in xml
    assert "<publish_odom>false</publish_odom>" in xml
    assert "<publish_odom_tf>false</publish_odom_tf>" in xml


def test_velodyne_sensor_and_plugin_use_35_meter_maximum_range():
    root = parse_xml(URDF)
    ray_sensor = root.find(".//sensor[@type='ray']")
    velodyne_plugin = root.find(
        ".//plugin[@filename='libgazebo_ros_velodyne_laser.so']"
    )
    assert ray_sensor.findtext("ray/range/max") == "35.0"
    assert velodyne_plugin.findtext("max_range") == "35.0"


def test_velodyne_discards_invalid_rays_before_fast_lio_time_synthesis():
    """Catches NaN placeholders reaching FAST-LIO2's per-ring first-point logic."""
    xml = URDF.read_text(encoding="utf-8")

    assert "<organize_cloud>false</organize_cloud>" in xml


def test_meshes_match_audited_size_and_sha256():
    assert {path.name for path in MESHES.glob("*.STL")} == set(EXPECTED_MESHES)
    for filename, (expected_size, expected_sha256) in EXPECTED_MESHES.items():
        path = MESHES / filename
        assert path.stat().st_size == expected_size
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected_sha256
