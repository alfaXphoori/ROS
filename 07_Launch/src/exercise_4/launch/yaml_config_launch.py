#!/usr/bin/env python3
"""
Exercise 4: YAML Configuration and Composition
Loads parameters from YAML files and composes other launch files
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    """Generate launch description with YAML config"""
    
    # Declare arguments
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='small',
        description='Robot configuration to load (small, large, simulation, or hardware)',
        choices=['small', 'large', 'simulation', 'hardware']
    )

    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='0',
        description='ROS_DOMAIN_ID to use. Must match across terminals to discover nodes.'
    )
    
    include_simple_launch_arg = DeclareLaunchArgument(
        'include_simple_launch',
        default_value='false',
        description='Include the simple_launch.py as well'
    )
    
    # Get configurations
    robot_config = LaunchConfiguration('robot_config')
    include_simple = LaunchConfiguration('include_simple_launch')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    
    # Build YAML file path (works both in src workspace and after install)
    # Installed config files live under: <pkg_share>/launch/config/
    config_file = PathJoinSubstitution([
        FindPackageShare('ce_robot_launch'),
        'launch',
        'config',
        TextSubstitution(text='robot_'),
        robot_config,
        TextSubstitution(text='.yaml'),
    ])
    
    # Set environment variable (optional, but useful for debugging)
    set_env = SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id)
    
    # Load nodes with YAML parameters
    # Using nodes from 07_Launch exercises
    battery_monitor_node = Node(
        package='ce_robot',
        executable='07_battery_monitor',
        name='battery_monitor',
        output='screen',
        parameters=[config_file]
    )
    
    navigation_node = Node(
        package='ce_robot',
        executable='07_navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[config_file]
    )
    
    task_processor_node = Node(
        package='ce_robot',
        executable='07_task_processor',
        name='task_processor',
        output='screen',
        parameters=[config_file]
    )
    
    fleet_monitor_node = Node(
        package='ce_robot',
        executable='07_fleet_monitor',
        name='fleet_monitor',
        output='screen',
        parameters=[config_file]
    )
    
    # Try to include another launch file conditionally
    # Note: This requires simple_launch.py to exist in the same directory
    try:
        pkg_share = FindPackageShare('ce_robot_launch').find('ce_robot_launch')
        simple_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'simple_launch.py'
                ])
            ]),
            condition=IfCondition(include_simple)
        )
        has_simple_launch = True
    except:
        # If simple_launch.py doesn't exist, create a dummy action
        simple_launch_include = LogInfo(
            msg='Note: simple_launch.py not found - skipping composition',
            condition=IfCondition(include_simple)
        )
        has_simple_launch = False
    
    return LaunchDescription([  # type: ignore
        # Environment
        set_env,
        
        # Arguments
        robot_config_arg,
        include_simple_launch_arg,
        ros_domain_id_arg,
        
        # Startup messages
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='📋 YAML Configuration Launch - Exercise 4'),
        LogInfo(msg=['🔧 Loading config: ', config_file]),
        LogInfo(msg=['🌐 ROS_DOMAIN_ID: ', ros_domain_id]),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        
        # Nodes with YAML config
        battery_monitor_node,
        navigation_node,
        task_processor_node,
        fleet_monitor_node,
        
        # Composed launch file
        simple_launch_include,
    ])