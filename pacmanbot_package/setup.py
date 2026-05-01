from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pacmanbot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),
        (os.path.join('share', package_name, 'rviz2'),
         glob('rviz2/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The Cool Team',
    maintainer_email='eva@todo.todo',
    description='Pacman TurtleBot Package',
    license='TODO',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'audio_node = pacmanbot_package.audio_node:main',
            'planner_stub = pacmanbot_package.planner_stub:main',
            'game_light_node = pacmanbot_package.game_light_node:main',
            'pellet_manager = pacmanbot_package.pellet_manager:main',
            'game_event_mapper = pacmanbot_package.game_event_mapper:main',
            'game_controller = pacmanbot_package.game_controller:main',
            'clyde_ghost_node = pacmanbot_package.clyde_ghost_node:main',
            'wait_for_amcl_pose = pacmanbot_package.wait_for_amcl_pose:main',
            'game_state_demo_gui = pacmanbot_package.game_state_demo_gui:main',
        ],
    },
)
