from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    tb4_viz_share = get_package_share_directory('turtlebot4_viz')

    nav2_params = '/home/eva/nav2_custom.yaml'
    map_file = '/home/eva/ros2_ws/src/PacManBot_ROS2/maps/map_01.yaml'  # change if needed

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'namespace': '/robot_15',
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'namespace': '/robot_15',
            'params_file': nav2_params,
            'tf_topic': '/robot_15/tf',
            'tf_static_topic': '/robot_15/tf_static',
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_viz_share, 'launch', 'view_navigation.launch.py')
        ),
        launch_arguments={
            'namespace': '/robot_15',
        }.items()
    )

    pellet_manager = Node(
        package='pacmanbot_package',
        executable='pellet_manager',
        name='pellet_manager',
        output='screen'
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    delayed_rviz = TimerAction(
        period=7.0,
        actions=[rviz_launch]
    )

    return LaunchDescription([
        localization_launch,
        delayed_nav2,
        delayed_rviz,
        pellet_manager
    ])