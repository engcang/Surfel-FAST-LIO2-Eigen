from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("surfel_fast_lio2_eigen"))

    config_file = LaunchConfiguration("config_file")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(package_share / "config" / "avia.yaml"),
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            Node(
                package="surfel_fast_lio2_eigen",
                executable="surfel_fast_lio2_eigen_mapping",
                name="laser_mapping",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", str(package_share / "rviz.rviz")],
                condition=IfCondition(rviz),
            ),
        ]
    )
