from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    config_path = PathJoinSubstitution([FindPackageShare("surfel_fast_lio2_eigen"), "config", config_file])
    rviz = LaunchConfiguration("rviz")
    rviz_path = PathJoinSubstitution([FindPackageShare("surfel_fast_lio2_eigen"), "rviz.rviz"])

    return LaunchDescription([DeclareLaunchArgument("config_file", default_value="mid360.yaml"),
                              DeclareLaunchArgument("rviz", default_value="false"),
                              Node(package="surfel_fast_lio2_eigen",
                                   executable="surfel_fast_lio2_eigen_mapping",
                                   name="laser_mapping",
                                   output="screen",
                                   parameters=[config_path]),
                              Node(package="rviz2",
                                   executable="rviz2",
                                   name="rviz",
                                   arguments=["-d", rviz_path],
                                   condition=IfCondition(rviz)),
    ])
