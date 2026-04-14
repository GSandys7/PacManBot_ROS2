from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pacmanbot_package')
    map_file = os.path.join(pkg_share, 'maps', 'map_01.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'namespace': '/robot_15',          
            'use_sim_time': 'false',
            'map': map_file,
            'tf_topic': '/robot_15/tf',
            'tf_static_topic': '/robot_15/tf_static',
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        remappings=[
            ('/tf', '/robot_15/tf'),
            ('/tf_static', '/robot_15/tf_static'),
            ('/initialpose', '/robot_15/initialpose'),
        ]
    )

    pellet_manager = Node(
        package='pacmanbot_package',
        executable='pellet_manager',
        name='pellet_manager',
        output='screen'
    )

    return LaunchDescription([
        nav2_launch,
        rviz,
        pellet_manager
    ])