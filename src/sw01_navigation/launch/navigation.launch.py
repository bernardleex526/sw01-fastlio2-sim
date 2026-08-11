import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    nav2_share = get_package_share_directory("nav2_bringup")
    navigation_share = get_package_share_directory("sw01_navigation")

    # 来源：sw01_navigation 的 Humble Nav2 参数；消费 map、camera_init、base_link、/scan 与 /Odometry。
    default_params = os.path.join(navigation_share, "config", "nav2_params.yaml")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        # 来源：Nav2 Humble navigation_launch.py 公共参数；由被包含入口统一管理导航服务器状态。
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "namespace": namespace,
        }.items(),
    )

    namespaced_navigation = GroupAction(
        actions=[
            # 来源：Nav2 可选命名空间接口；仅在 use_namespace=true 时限定节点及相对话题作用域。
            PushRosNamespace(namespace, condition=IfCondition(use_namespace)),
            navigation,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            # 来源：Gazebo /clock；所有导航节点默认使用仿真时间并自动启动。
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            # 来源：Nav2 可选命名空间接口；空字符串与 false 保持默认根命名空间话题。
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_namespace", default_value="false"),
            namespaced_navigation,
        ]
    )
