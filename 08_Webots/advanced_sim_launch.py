#!/usr/bin/env python3
"""Exercise 3: Advanced multi-sensor Webots simulation launch file"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    pkg_share = FindPackageShare('ce_robot').find('ce_robot')
    world_path = Path(pkg_share) / 'worlds' / 'advanced_sensor_world.wbt'
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=str(world_path),
        description='Path to Webots world file'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
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
    
    # ROS 2 node for data analysis and logging
    data_logger_node = Node(
        package='ce_robot',
        executable='sensor_data_logger',
        name='sensor_data_logger',
        output='screen',
        parameters=[
            {'log_directory': '/tmp/webots_logs'},
            {'log_enabled': True}
        ]
    )
    
    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(Path(pkg_share) / 'rviz' / 'advanced_config.rviz')],
        output='screen',
        on_exit=None
    )
    
    return LaunchDescription([
        world_file_arg,
        enable_rviz_arg,
        webots_process,
        data_logger_node,
        rviz_node,
    ])
