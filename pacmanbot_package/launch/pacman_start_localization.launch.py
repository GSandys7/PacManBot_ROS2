import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    pacman_share = get_package_share_directory('pacmanbot_package')
    map_file = os.path.join(pacman_share, 'maps', 'map_01.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb4_nav_share, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'namespace': '/robot_15',
            }.items()
        ),
    ])
