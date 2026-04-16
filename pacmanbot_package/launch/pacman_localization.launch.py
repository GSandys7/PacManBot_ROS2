from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    DELAY = 5.0 

    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    tb4_viz_share = get_package_share_directory('turtlebot4_viz')
    pacman_share = get_package_share_directory('pacmanbot_package')

    nav2_params = '/home/eva/nav2_custom.yaml'
    map_file = os.path.join(pacman_share, 'maps', 'map_01.yaml')
    print("MAP FILE:", map_file)
    print("PACMAN SHARE:", pacman_share)

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

    game_event_mapper = Node(
        package='pacmanbot_package',
        executable='game_event_mapper',
        name='game_event_mapper',
        output='screen'
    )

    planner_stub = Node(
        package='pacmanbot_package',
        executable='planner_stub',
        name='planner_stub',
        output='screen'
    )

    return LaunchDescription([
        localization_launch,

        TimerAction(
            period=DELAY,
            actions=[
                nav2_launch,

                TimerAction(
                    period=DELAY,
                    actions=[
                        pellet_manager,
                        audio_node,
                        game_light_node,

                        TimerAction(
                            period=DELAY,
                            actions=[
                                game_event_mapper,

                                TimerAction(
                                    period=DELAY,
                                    actions=[
                                        planner_stub,

                                        TimerAction(
                                            period=DELAY,
                                            actions=[
                                                rviz_launch
                                            ]
                                        )
                                    ]
                                )
                            ]
                        )
                    ]
                )
            ]
        )
    ])