#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='el7009_diff_drive_robot',
            executable='dwa_node.py',
            name='dwa_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
