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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The Cool Team',
    maintainer_email='eva@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_stub = pacmanbot_package.planner_stub:main',
            'game_light = pacmanbot_package.game_light:main',
        ],
    },
)
