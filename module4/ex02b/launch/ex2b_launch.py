import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = "ex02b"
    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name), "config/", "conf.rviz"
    )

    static_target_arg = DeclareLaunchArgument(
        "static_target", default_value="['8.0', '2.0']"
    )
    radius_arg = DeclareLaunchArgument("radius", default_value="1.0")
    direction_arg = DeclareLaunchArgument("direction", default_value="1.0")
    frames_id_arg = DeclareLaunchArgument(
        "frames_id", default_value="['turtle1', 'turtle3']"
    )
    child_frames_id_arg = DeclareLaunchArgument(
        "child_frames_id", default_value="['carrot1', 'carrot2']"
    )
    switch_threshold_arg = DeclareLaunchArgument(
        "switch_threshold", default_value="1.0"
    )
    turtle_name_arg = DeclareLaunchArgument("turtle_name", default_value="turtle2")

    turtlesim_node = Node(package="turtlesim", executable="turtlesim_node", name="sim")

    spawn_turtle2 = ExecuteProcess(
        cmd=[
            "sleep 1; ros2 service call /spawn turtlesim/srv/Spawn \"{x: 1.0, y: 1.0, name: 'turtle2'}\""
        ],
        shell=True,
    )

    spawn_turtle3 = ExecuteProcess(
        cmd=[
            "sleep 1; ros2 service call /spawn turtlesim/srv/Spawn \"{x: 8.0, y: 8.0, name: 'turtle3'}\""
        ],
        shell=True,
    )

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
    broadcaster3 = Node(
        package=pkg_name,
        executable="turtle_tf2_broadcaster",
        name="broadcaster3",
        parameters=[{"turtlename": "turtle3"}],
    )

    target_switcher_node = Node(
        package=pkg_name,
        executable="target_switcher",
        name="target_switcher",
        parameters=[
            {"static_target": LaunchConfiguration("static_target")},
            {"radius": LaunchConfiguration("radius")},
            {"direction": LaunchConfiguration("direction")},
            {"frames_id": LaunchConfiguration("frames_id")},
            {"child_frames_id": LaunchConfiguration("child_frames_id")},
        ],
    )

    turtle_controller_node = Node(
        package=pkg_name,
        executable="turtle_controller",
        name="turtle_controller",
        output="screen",
        prefix="x-terminal-emulator -e",
        parameters=[
            {"switch_threshold": LaunchConfiguration("switch_threshold")},
            {"turtle_name": LaunchConfiguration("turtle_name")},
        ],
    )

    teleop_turtle1 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        name="teleop_turtle1",
        prefix="x-terminal-emulator -e",
    )
    teleop_turtle3 = Node(
        package="turtlesim",
        executable="turtle_teleop_key",
        name="teleop_turtle3",
        remappings=[("/turtle1/cmd_vel", "/turtle3/cmd_vel")],
        prefix="x-terminal-emulator -e",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            static_target_arg,
            radius_arg,
            direction_arg,
            frames_id_arg,
            child_frames_id_arg,
            switch_threshold_arg,
            turtle_name_arg,
            turtlesim_node,
            spawn_turtle2,
            spawn_turtle3,
            broadcaster1,
            broadcaster2,
            broadcaster3,
            target_switcher_node,
            turtle_controller_node,
            teleop_turtle1,
            teleop_turtle3,
            rviz,
        ]
    )
