from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node",
            output="screen",
        ),

        TimerAction(
            period=1.0,
            actions= [
                Node(
                    package="rosp_layered_exercise",
                    executable="kill_turtle_node",
                    name="kill_turtle_node",
                )
            ]
        ),

        TimerAction(
            period=2.0,
            actions= [
                Node(
                    package="rosp_layered_exercise",
                    executable="spawn_turtle_node",
                    name="spawn_turtle_node"
                )
            ]
        ),

        TimerAction(
            period=2.5,
            actions= [
                Node(
                    package="rosp_layered_exercise",
                    executable="layered_exercise_node",
                    name="layered_exercise_node",
                    output="screen"
                )
            ]
        )
    ])
