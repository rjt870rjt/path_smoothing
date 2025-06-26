from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to parameters.yaml
    params_file = os.path.join(
        get_package_share_directory('path_smoothing_control'),
        'config',
        'parameters.yaml'
    )

    # TurtleBot3 simulation
    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'launch', 'turtlebot3_world.launch.py')
        ])
    )

    # Nodes for path smoothing control
    waypoint_publisher = Node(
        package='path_smoothing_control',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        parameters=[params_file],
        output='screen'
    )

    path_smoother = Node(
        package='path_smoothing_control',
        executable='path_smoother',
        name='path_smoother',
        parameters=[params_file],
        output='screen'
    )

    trajectory_generator = Node(
        package='path_smoothing_control',
        executable='trajectory_generator',
        name='trajectory_generator',
        parameters=[params_file],
        output='screen'
    )

    obstacle_avoidance = Node(
        package='path_smoothing_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[params_file],
        output='screen'
    )

    trajectory_follower = Node(
        package='path_smoothing_control',
        executable='trajectory_follower',
        name='trajectory_follower',
        parameters=[params_file],
        output='screen'
    )

    metrics_node = Node(
        package='path_smoothing_control',
        executable='metrics_node',
        name='metrics_node',
        parameters=[params_file],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('path_smoothing_control'),
            'rviz', 'path_tracking_config.rviz')],
        output='screen'
    )

    return LaunchDescription([
        tb3_gazebo,
        waypoint_publisher,
        path_smoother,
        trajectory_generator,
        obstacle_avoidance,
        trajectory_follower,
        metrics_node,
        rviz_node
    ])
