import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    pacman_share = get_package_share_directory('pacmanbot_package')
    default_params_file = os.path.join(
        pacman_share,
        'launch',
        'nav2_custom.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot_15'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument('tf_topic', default_value='/robot_15/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/robot_15/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb4_nav_share, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'tf_topic': LaunchConfiguration('tf_topic'),
                'tf_static_topic': LaunchConfiguration('tf_static_topic'),
            }.items()
        ),
    ])
