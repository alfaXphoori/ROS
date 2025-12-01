#!/usr/bin/env python3
"""Exercise 1: Simple Webots simulation launch file"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare
from pathlib import Path

def generate_launch_description():
    pkg_share = FindPackageShare('ce_robot').find('ce_robot')
    world_path = Path(pkg_share) / 'worlds' / 'simple_robot_world.wbt'
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=str(world_path),
        description='Path to Webots world file'
    )
    
    webots_process = ExecuteProcess(
        cmd=[
            'webots',
            LaunchConfiguration('world_file'),
            '--batch'
        ],
        output='screen',
        name='webots_simulator'
    )
    
    return LaunchDescription([
        world_file_arg,
        webots_process,
    ])
