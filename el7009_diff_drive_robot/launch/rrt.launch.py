#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name='amcl_launch'
    map_yaml = os.path.join(get_package_share_directory(package_name),'map','mapa.yaml')
    return LaunchDescription([
        Node(
            package='el7009_diff_drive_robot',
            executable='dwa_node.py',
            name='dwa_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='el7009_diff_drive_robot',
            executable='gttf.py',
            name='ground_truth_transform',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='el7009_diff_drive_robot',
            executable='rrt_star_node.py',
            name='rrt_node',
            output='screen'
        )
    ])
