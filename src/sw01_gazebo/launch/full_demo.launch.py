import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory("sw01_gazebo")
    slam_share = get_package_share_directory("sw01_slam")
    navigation_share = get_package_share_directory("sw01_navigation")

    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "simulation.launch.py")
        ),
        # 来源：full-demo 透传接口；出生坐标单位 m、yaw 单位 rad，并共享 Gazebo 图形与仿真时钟开关。
        launch_arguments={
            "world": world,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "gui": gui,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "slam.launch.py")
        ),
        # 来源：full-demo 透传接口；RViz 与 SLAM 共用 map 固定坐标系及 /clock。
        launch_arguments={"rviz": rviz, "use_sim_time": use_sim_time}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "navigation.launch.py")
        ),
        # 来源：full-demo 透传接口；Nav2 消费 SLAM 已建立的 map/TF 与 /scan。
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    default_world = os.path.join(gazebo_share, "worlds", "sw01_maze.world")
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            # 来源：仿真迷宫安全出生点；x、y、z 单位 m，yaw 单位 rad。
            DeclareLaunchArgument("x", default_value="-12.0"),
            DeclareLaunchArgument("y", default_value="-12.0"),
            DeclareLaunchArgument("z", default_value="0.62"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            # 来源：任务开关；gui、rviz 默认开启，所有阶段默认使用 Gazebo /clock。
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # 来源：任务规定启动顺序；0 s 启动 Gazebo/机器人，5 s 启动 SLAM，12 s 启动 Nav2。
            simulation,
            TimerAction(period=5.0, actions=[slam]),
            TimerAction(period=12.0, actions=[navigation]),
        ]
    )
