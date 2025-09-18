from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            namespace="turtlesim1",
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            namespace="turtlesim2",
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            namespace="turtlesim3",
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_t2_follows_t1',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel')
            ]
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_t3_follows_t2',
            remappings=[
                ('/input/pose', '/turtlesim2/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim3/turtle1/cmd_vel')
            ]
        ),
    ])
