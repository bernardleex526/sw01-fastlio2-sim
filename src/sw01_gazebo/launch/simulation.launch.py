import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo_package_share = get_package_share_directory("sw01_gazebo")
    description_share = get_package_share_directory("sw01_description")

    # 来源：sw01_gazebo 自带迷宫世界；world 参数允许替换为其他 Gazebo 世界文件。
    default_world = os.path.join(gazebo_package_share, "worlds", "sw01_maze.world")
    # 来源：sw01_description 的仿真 Xacro；robot_description 是生成实体与状态发布器的共同输入。
    xacro_file = os.path.join(description_share, "urdf", "sw01_sim.urdf.xacro")
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])

    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "gui": gui}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        # 来源：Gazebo /clock；所有机器人 TF 与仿真传感器时间戳保持一致。
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        # 来源：任务约定；实体 sw01 从 robot_description 生成，默认起点为 (-12, -12, 0.62)，偏航 0 rad。
        arguments=[
            "-entity",
            "sw01",
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-Y",
            yaw,
        ],
    )

    body_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        # 来源：FAST-LIO2 body 到 URDF base_link 的标定；平移单位 m，旋转 roll/pitch/yaw 均为 0 rad。
        arguments=[
            "-0.0065429535",
            "0.0050846333",
            "-0.016924295",
            "0",
            "0",
            "0",
            "body",
            "base_link",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            # 来源：迷宫世界的安全出生点；x、y、z 单位 m，yaw 单位 rad。
            DeclareLaunchArgument("x", default_value="-12.0"),
            DeclareLaunchArgument("y", default_value="-12.0"),
            DeclareLaunchArgument("z", default_value="0.62"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            # 来源：Gazebo 启动接口；默认显示图形界面并使用 /clock 仿真时间。
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            body_to_base_link,
        ]
    )
