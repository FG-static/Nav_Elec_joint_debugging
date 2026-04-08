import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_serial_driver = get_package_share_directory('rm_serial_driver')
    pkg_my_nav2_robot = get_package_share_directory('my_nav2_robot')

    serial_driver_launch = os.path.join(
        pkg_serial_driver, 'launch', 'serial_driver.launch.py')

    navigation_launch = os.path.join(
        pkg_my_nav2_robot, 'launch', 'full_navigation.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(serial_driver_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={'use_sim_time': 'false'}.items()
        )
    ])
