from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pellet_manager = Node(
        package='pacmanbot_package',
        executable='pellet_manager',
        name='pellet_manager',
        output='screen'
    )

    audio_node = Node(
        package='pacmanbot_package',
        executable='audio_node',
        name='audio_node',
        output='screen'
    )

    game_light_node = Node(
        package='pacmanbot_package',
        executable='game_light_node',
        name='game_light_node',
        output='screen'
    )

    clyde_ghost_node = Node(
        package='pacmanbot_package',
        executable='clyde_ghost_node',
        name='clyde_ghost_node',
        output='screen'
    )

    game_event_mapper = Node(
        package='pacmanbot_package',
        executable='game_event_mapper',
        name='game_event_mapper',
        output='screen'
    )

    game_controller = Node(
        package='pacmanbot_package',
        executable='game_controller',
        name='game_controller',
        output='screen'
    )

    wait_for_amcl_pose = Node(
        package='pacmanbot_package',
        executable='wait_for_amcl_pose',
        name='wait_for_amcl_pose',
        output='screen'
    )

    planner_stub = Node(
        package='pacmanbot_package',
        executable='planner_stub',
        name='planner_stub',
        output='screen'
    )

    return LaunchDescription([
        pellet_manager,
        audio_node,
        game_light_node,
        clyde_ghost_node,
        game_event_mapper,
        game_controller,
        wait_for_amcl_pose,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_amcl_pose,
                on_exit=[
                    planner_stub,
                ],
            )
        ),
    ])
