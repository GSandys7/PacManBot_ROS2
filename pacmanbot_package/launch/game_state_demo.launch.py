from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pacmanbot_package',
            executable='audio_node',
            name='audio_node',
            output='screen'
        ),
        Node(
            package='pacmanbot_package',
            executable='game_light_node',
            name='game_light_node',
            output='screen'
        ),
        Node(
            package='pacmanbot_package',
            executable='game_event_mapper',
            name='game_event_mapper',
            output='screen'
        ),
        Node(
            package='pacmanbot_package',
            executable='game_state_demo_gui',
            name='game_state_demo_gui',
            output='screen'
        ),
    ])
