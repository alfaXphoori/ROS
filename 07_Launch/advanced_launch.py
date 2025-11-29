#!/usr/bin/env python3
"""
Exercise 3: Advanced Launch File with Arguments and Logic
Demonstrates arguments, substitutions, and conditional node launching
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate advanced launch description"""
    
    # Declare launch arguments with defaults
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_advanced',
        description='Name of the robot'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='300',
        description='Robot ID number'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )
    
    enable_monitoring_arg = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='Enable monitoring subscriber'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging'
    )
    
    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    robot_id = LaunchConfiguration('robot_id')
    publish_rate = LaunchConfiguration('publish_rate')
    enable_monitoring = LaunchConfiguration('enable_monitoring')
    debug_mode = LaunchConfiguration('debug_mode')
    
    # Publisher node with dynamic parameters
    publisher_node = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='hw_publisher_ex3',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'robot_number': robot_id},
            {'publish_rate': publish_rate},
            {'debug_mode': debug_mode},
        ],
        remappings=[
            ('/hardware_status', '/robot_hw_status'),
        ]
    )
    
    # Conditional subscriber (only launched if enabled)
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='hw_monitor_ex3',
        output='screen',
        parameters=[
            {'robot_name': 'system_monitor'},
            {'debug_mode': True},
        ],
        remappings=[
            ('/hardware_status', '/robot_hw_status'),
        ],
        condition=IfCondition(enable_monitoring)
    )
    
    # Service server
    service_server = Node(
        package='ce_robot',
        executable='cal_rect_server',
        name='geometry_server_ex3',
        output='screen'
    )
    
    # Return all components
    return LaunchDescription([
        # Arguments
        robot_name_arg,
        robot_id_arg,
        publish_rate_arg,
        enable_monitoring_arg,
        debug_mode_arg,
        
        # Nodes
        publisher_node,
        subscriber_node,
        service_server,
    ])
