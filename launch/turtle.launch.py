# import launch
# import launch_ros.actions

# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch_ros.actions.Node(
#             package='turtlesim',
#             executable='turtlesim_node',
#             name='turtlesim',
#             output='screen'
#         ),

#         launch_ros.actions.Node(
#             package='ros2service',
#             executable='ros2service',
#             name='kill_turtle',
#             arguments=['call', '/kill', "{name: 'turtle1'}"]
#         ),

#         launch_ros.actions.Node(
#             package='ros2service',
#             executable='ros2service',
#             name='spawn_turtle',
#             arguments=['call', '/spawn', "{x: 1.0, y: 1.0, name: 'turtle'}"]
#         ),

#         launch_ros.actions.Node(
#             package='layered_exercise',
#             executable='layered_exercise_node',
#             name='my_node',
#             output='screen'
#         ),
#     ])



# import launch
# import launch_ros.actions
# from launch.actions import RegisterEventHandler, LogInfo
# from launch.event_handlers import OnProcessExit

# def generate_launch_description():
#     # Launch turtlesim
#     turtlesim_node = launch_ros.actions.Node(
#         package='turtlesim',
#         executable='turtlesim_node',
#         name='turtlesim',
#         output='screen'
#     )

#     # Kill turtle1
#     kill_turtle_node = launch_ros.actions.Node(
#         package='layered_exercise',
#         executable='kill_turtle_node',
#         name='kill_turtle',
#         output='screen'
#     )

#     # Spawn new turtle (Only runs after kill_turtle exits)
#     spawn_turtle_node = launch_ros.actions.Node(
#         package='layered_exercise',
#         executable='spawn_turtle_node',
#         name='spawn_turtle',
#         output='screen'
#     )

#     # Ensure spawn_turtle runs only after kill_turtle has finished
#     spawn_after_kill = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=kill_turtle_node,
#             on_exit=[LogInfo(msg="Kill finished, now spawning new turtle..."), spawn_turtle_node],
#         )
#     )

#     # Main control node
#     main_node = launch_ros.actions.Node(
#         package='layered_exercise',
#         executable='layered_exercise_node',
#         name='my_node',
#         output='screen'
#     )

#     return launch.LaunchDescription([
#         turtlesim_node,
#         kill_turtle_node,
#         spawn_after_kill,  # This ensures spawn_turtle starts only after kill_turtle finishes
#         main_node
#     ])



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
            name="layered_exercise_node"
        )
    ])
