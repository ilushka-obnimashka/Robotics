import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ex02_share = get_package_share_directory("ex02")
    rviz_config_file = os.path.join(pkg_ex02_share, "config/", "carrot.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "radius",
                default_value="1.0",
                description="Carrot frame rotation radius",
            ),
            DeclareLaunchArgument(
                "direction",
                default_value="1.0",
                description="Direction of rotation of the carrot (-1 or 1)",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="turtle1",
                description="The frame around which the carrot revolves",
            ),
            DeclareLaunchArgument(
                "child_frame_id", default_value="carrot1", description="Carrot's name"
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="carrot1",
                description="The name of the frame to follow",
            ),
            DeclareLaunchArgument(
                "follower_frame",
                default_value="turtle2",
                description="The name of the frame that will follow someone",
            ),
            Node(package="turtlesim", executable="turtlesim_node", name="sim"),
            Node(
                package="ex2",
                executable="carrot_broadcaster",
                name="carrot_broadcaster",
                parameters=[
                    {"radius": LaunchConfiguration("radius")},
                    {"direction": LaunchConfiguration("direction")},
                    {"frame_id": LaunchConfiguration("frame_id")},
                    {"child_frame_id": LaunchConfiguration("child_frame_id")},
                ],
            ),
            Node(
                package="ex2",
                executable="carrot_follower",
                name="carrot_follower",
                parameters=[
                    {"target_frame": LaunchConfiguration("target_frame")},
                    {"follower_frame": LaunchConfiguration("follower_frame")},
                ],
                remappings=[("/cmd_vel", "/turtle2/cmd_vel")],
            ),
            Node(
                package="turtlesim",
                executable="turtle_teleop_key",
                name="teleop",
                remappings=[("/cmd_vel", "/turtle1/cmd_vel")],
                prefix="gnome-terminal --",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
            ),
            Node(
                package="ex2",
                executable="turtle_tf2_broadcaster",
                name="broadcaster1",
                parameters=[{"turtlename": "turtle1"}],
            ),
            Node(
                package="ex2",
                executable="turtle_tf2_broadcaster",
                name="broadcaster1",
                parameters=[{"turtlename": "turtle2"}],
            ),
        ]
    )
