#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_yaml = os.path.join(get_package_share_directory('amcl_launch'),'map','mapa.yaml')
    local_yaml = os.path.join(get_package_share_directory('amcl_launch'),'map','localization_map.yaml')
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': True,
                'always_send_full_map': True
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'always_send_full_map': True,
                'node_names': ['map_server']
            }]
        )
    ])
