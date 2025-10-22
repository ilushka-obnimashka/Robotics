from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = "ex03"

    delay_arg = DeclareLaunchArgument("delay", default_value="5.0")

    parent_frame_arg = DeclareLaunchArgument("parent_frame", default_value="turtle1")
    child_frame_arg = DeclareLaunchArgument("child_frame", default_value="turtle2")

    broadcaster1 = Node(
        package=pkg_name,
        executable="turtle_tf2_broadcaster",
        name="broadcaster1",
        parameters=[{"turtlename": "turtle1"}],
    )

    broadcaster2 = Node(
        package=pkg_name,
        executable="turtle_tf2_broadcaster",
        name="broadcaster2",
        parameters=[{"turtlename": "turtle2"}],
    )

    teleop_turtle1 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        name="teleop_turtle1",
        prefix="x-terminal-emulator -e",
    )

    delay_listener = Node(
        package=pkg_name,
        executable="delay_tf2_listener",
        name="delay_listener",
        parameters=[
            {"delay": LaunchConfiguration("delay")},
            {"parent_frame": LaunchConfiguration("parent_frame")},
            {"child_frame": LaunchConfiguration("child_frame")},
        ],
    )

    turtlesim_node = Node(package="turtlesim", executable="turtlesim_node", name="sim")

    return LaunchDescription(
        [
            delay_arg,
            child_frame_arg,
            parent_frame_arg,
            broadcaster1,
            broadcaster2,
            teleop_turtle1,
            delay_listener,
            turtlesim_node,
        ]
    )
