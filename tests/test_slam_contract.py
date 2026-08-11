from pathlib import Path

import pytest

from conftest import SRC, load_yaml


FAST_LIO = SRC / "sw01_slam/config/sw01_sim.yaml"
SLAM_TOOLBOX = SRC / "sw01_slam/config/slam_toolbox_params.yaml"


def ros_params(path: Path, node: str = "/**"):
    return load_yaml(path)[node]["ros__parameters"]


def test_fast_lio_matches_simulated_velodyne_and_imu():
    """Catches a FAST-LIO2 config that cannot consume SW01's sensor contract."""
    p = ros_params(FAST_LIO)

    assert p["feature_extract_enable"] is False
    assert p["point_filter_num"] == 4
    assert p["max_iteration"] == 3
    assert p["filter_size_surf"] == pytest.approx(0.10)
    assert p["filter_size_map"] == pytest.approx(0.20)
    assert p["cube_side_length"] == pytest.approx(1000.0)
    assert p["runtime_pos_log_enable"] is False
    assert p["map_file_path"] == "./test.pcd"
    assert p["common"] == {
        "lid_topic": "/velodyne_points",
        "imu_topic": "/imu/data",
        "time_sync_en": False,
        "time_offset_lidar_to_imu": 0.0,
    }
    assert p["preprocess"] == {
        "lidar_type": 2,
        "scan_line": 16,
        "scan_rate": 10,
        "timestamp_unit": 2,
        "blind": pytest.approx(0.30),
    }
    assert p["mapping"]["det_range"] == pytest.approx(35.0)
    assert p["mapping"]["extrinsic_est_en"] is False
    assert p["mapping"]["extrinsic_T"] == pytest.approx(
        [-0.0865429535, 0.0050846333, 0.463075705]
    )
    assert p["mapping"]["extrinsic_R"] == pytest.approx(
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    assert p["publish"] == {
        "path_en": True,
        "scan_publish_en": True,
        "dense_publish_en": True,
        "scan_bodyframe_pub_en": True,
    }
    assert p["pcd_save"] == {"pcd_save_en": False, "interval": -1}


def test_slam_toolbox_owns_map_to_fast_lio_odom_frame():
    """Catches a mapper that publishes a transform outside the SW01 TF contract."""
    p = load_yaml(SLAM_TOOLBOX)["slam_toolbox"]["ros__parameters"]

    assert p["use_sim_time"] is True
    assert p["mode"] == "mapping"
    assert p["map_frame"] == "map"
    assert p["odom_frame"] == "camera_init"
    assert p["base_frame"] == "body"
    assert p["scan_topic"] == "/scan"
    assert p["resolution"] == pytest.approx(0.05)
    assert p["max_laser_range"] == pytest.approx(30.0)
    assert p["minimum_time_interval"] == pytest.approx(0.10)
    assert p["transform_publish_period"] == pytest.approx(0.02)
    assert p["map_update_interval"] == pytest.approx(1.0)
    assert p["use_scan_matching"] is True
    assert p["do_loop_closing"] is True
