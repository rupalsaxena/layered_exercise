from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim",
            output="screen",
        ),
        Node(
            package="layered_exercise",
            executable="kill_turtle_node",
            name="kill_turtle_node",
        ),
        Node(
            package="layered_exercise",
            executable="spawn_turtle_node",
            name="spawn_turtle_node"
        ),
        Node(
            package="layered_exercise",
            executable="layered_exercise_node",
            name="layered_exercise_node",
            output="screen"
        )
    ])
