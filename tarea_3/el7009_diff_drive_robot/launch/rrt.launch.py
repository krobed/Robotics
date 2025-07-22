#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    map_path = '/home/krobed/EL7009/FProject/amcl_launch/map/mapa.yaml'
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_path,
                'topic_name': 'map',
                'frame_id': 'map',
                'autostart': True }]
        ),
        Node(
            package='el7009_diff_drive_robot',
            executable='dwa_node.py',
            name='dwa_node',
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
