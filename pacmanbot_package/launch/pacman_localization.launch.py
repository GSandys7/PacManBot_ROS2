from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os


def prepend_env_path(name, path):
    current = os.environ.get(name, '')
    if not current:
        return path
    paths = current.split(os.pathsep)
    if path in paths:
        return current
    return os.pathsep.join([path, current])


def generate_launch_description():
    pacman_share = get_package_share_directory('pacmanbot_package')
    pacman_prefix = get_package_prefix('pacmanbot_package')

    if os.path.exists(os.path.join(pacman_prefix, 'share', 'pacmanbot_rviz_plugins')):
        rviz_plugin_prefix = pacman_prefix
    else:
        rviz_plugin_prefix = os.path.join(
            os.path.dirname(pacman_prefix),
            'pacmanbot_rviz_plugins'
        )
    rviz_plugin_lib = os.path.join(rviz_plugin_prefix, 'lib')

    rviz_config = os.path.join(pacman_share, 'rviz2', 'PacManBot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='/robot_15',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'AMENT_PREFIX_PATH',
            prepend_env_path('AMENT_PREFIX_PATH', rviz_plugin_prefix)
        ),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            prepend_env_path('LD_LIBRARY_PATH', rviz_plugin_lib)
        ),
        rviz_node,
    ])
