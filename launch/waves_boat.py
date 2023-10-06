from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

import os

def gz_simulation(world_name, headless=False, paused=False, extra_gz_args=''):
    gz_args = ['-v 4']
    if not paused:
        gz_args.append('-r')

    if headless:
        gz_args.append('-s')

    gz_args.append(extra_gz_args)
    gz_args.append(f'{world_name}.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())

    return gz_sim


def generate_launch_description():
    return LaunchDescription([
        gz_simulation("../worlds/waves", paused=True)
    ])