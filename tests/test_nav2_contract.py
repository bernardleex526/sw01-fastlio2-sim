import yaml

from conftest import SRC, load_yaml


NAV2 = SRC / "sw01_navigation/config/nav2_params.yaml"
FOOTPRINT = [
    [0.46, 0.0],
    [0.325, 0.19],
    [0.0, 0.27],
    [-0.325, 0.19],
    [-0.46, 0.0],
    [-0.325, -0.19],
    [0.0, -0.27],
    [0.325, -0.19],
]


def test_nav2_planner_controller_and_frames():
    """Catches planners or motion limits that violate the SW01 Nav2 contract."""
    data = load_yaml(NAV2)
    planner = data["planner_server"]["ros__parameters"]
    controller = data["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    bt = data["bt_navigator"]["ros__parameters"]

    assert planner["GridBased"]["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert planner["GridBased"]["tolerance"] == 0.25
    assert planner["GridBased"]["allow_unknown"] is True
    assert planner["GridBased"]["downsample_costmap"] is False
    assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
    assert follow_path["min_vel_x"] == -0.25
    assert follow_path["max_vel_x"] == 0.80
    assert follow_path["min_vel_y"] == -0.40
    assert follow_path["max_vel_y"] == 0.40
    assert follow_path["max_vel_theta"] == 1.0
    assert follow_path["max_speed_xy"] == 0.80
    assert follow_path["acc_lim_x"] == 0.8
    assert follow_path["acc_lim_y"] == 0.8
    assert follow_path["acc_lim_theta"] == 1.5
    assert follow_path["sim_time"] == 1.7
    assert follow_path["vx_samples"] == 20
    assert follow_path["vy_samples"] == 15
    assert follow_path["vtheta_samples"] == 20
    assert controller["general_goal_checker"]["xy_goal_tolerance"] == 0.25
    assert controller["general_goal_checker"]["yaw_goal_tolerance"] == 0.25
    assert bt["global_frame"] == "map"
    assert bt["robot_base_frame"] == "base_link"
    assert bt["odom_topic"] == "/Odometry"


def test_costmaps_use_map_and_projected_scan_with_actual_footprint():
    """Catches costmaps that cannot consume the projected scan in the agreed TF tree."""
    data = load_yaml(NAV2)
    for outer in ("local_costmap", "global_costmap"):
        params = data[outer][outer]["ros__parameters"]
        assert yaml.safe_load(params["footprint"]) == FOOTPRINT
        assert params["robot_base_frame"] == "base_link"

    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = data["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"] == "camera_init"
    assert local["width"] == 6
    assert local["height"] == 6
    assert local["resolution"] == 0.05
    assert local["update_frequency"] == 10.0
    assert local["publish_frequency"] == 5.0
    assert global_["global_frame"] == "map"
    assert local["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert global_["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert "static_layer" in global_["plugins"]
    for params in (local, global_):
        scan = params["obstacle_layer"]["scan"]
        assert scan["data_type"] == "LaserScan"
        assert scan["marking"] is True
        assert scan["clearing"] is True
        assert params["inflation_layer"]["inflation_radius"] == 0.45
        assert params["inflation_layer"]["cost_scaling_factor"] == 3.0


def test_all_nav2_nodes_use_sim_time_and_smoother_uses_fast_lio_odometry():
    """Catches mixed clocks or a velocity smoother disconnected from FAST-LIO2 odometry."""
    data = load_yaml(NAV2)
    node_params = {
        "planner_server": data["planner_server"]["ros__parameters"],
        "controller_server": data["controller_server"]["ros__parameters"],
        "bt_navigator": data["bt_navigator"]["ros__parameters"],
        "behavior_server": data["behavior_server"]["ros__parameters"],
        "waypoint_follower": data["waypoint_follower"]["ros__parameters"],
        "velocity_smoother": data["velocity_smoother"]["ros__parameters"],
        "local_costmap": data["local_costmap"]["local_costmap"]["ros__parameters"],
        "global_costmap": data["global_costmap"]["global_costmap"]["ros__parameters"],
    }
    assert all(params["use_sim_time"] is True for params in node_params.values())

    smoother = node_params["velocity_smoother"]
    assert smoother["smoothing_frequency"] == 40.0
    assert smoother["feedback"] == "OPEN_LOOP"
    assert smoother["odom_topic"] == "/Odometry"
