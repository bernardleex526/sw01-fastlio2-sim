# SLAM 后端：slam_toolbox（当前激活）。备用 FAST-LIO2 参数见 config/sw01_sim.yaml。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory("sw01_slam")
    fast_lio_params = os.path.join(slam_share, "config", "sw01_sim.yaml")
    slam_toolbox_params = os.path.join(slam_share, "config", "slam_toolbox_params.yaml")
    rviz_config = os.path.join(slam_share, "rviz", "sw01_sim.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam_toolbox = LaunchConfiguration("use_slam_toolbox")
    rviz = LaunchConfiguration("rviz")

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        output="screen",
        # 来源：sw01_sim.yaml；消费 /velodyne_points 与 /imu/data，发布 camera_init->body、/cloud_registered、/path、/Odometry。
        parameters=[fast_lio_params, {"use_sim_time": use_sim_time}],
    )

    cloud_projection = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        output="screen",
        # 来源：Task 6 点云投影契约；高度与距离单位 m、角度单位 rad、容差单位 s，目标坐标系为 FAST-LIO2 body。
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "body",
                "min_height": -0.05,
                "max_height": 0.45,
                "angle_min": -3.141592653589793,
                "angle_max": 3.141592653589793,
                "angle_increment": 0.008726646,
                "range_min": 0.30,
                "range_max": 30.0,
                "transform_tolerance": 0.10,
            }
        ],
        # 来源：FAST-LIO2 注册点云输出；投影后的 /scan 同时供 slam_toolbox 与 Nav2 使用。
        remappings=[("cloud_in", "/cloud_registered"), ("scan", "/scan")],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(use_slam_toolbox),
        # 来源：slam_toolbox_params.yaml；以 map、camera_init、body 和 /scan 构成二维建图契约。
        parameters=[slam_toolbox_params, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        # 来源：sw01_slam 可安装的 RViz 配置；固定坐标系为 map。
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            # 来源：Gazebo /clock；SLAM 图中的全部节点默认使用仿真时间。
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # 来源：任务开关；false 时仅运行 FAST-LIO2 与点云投影。
            DeclareLaunchArgument("use_slam_toolbox", default_value="true"),
            # 来源：任务开关；false 时不启动 RViz，适用于无图形运行。
            DeclareLaunchArgument("rviz", default_value="true"),
            fast_lio,
            cloud_projection,
            slam_toolbox,
            rviz_node,
        ]
    )
