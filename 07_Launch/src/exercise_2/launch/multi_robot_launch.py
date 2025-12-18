#!/usr/bin/env python3
"""
Exercise 2: Multi-Robot Launch with Namespaces
Launches 3 robot instances with isolated namespaces using custom Exercise 2 nodes
Real-world scenario: Multi-zone warehouse with independent robot operations
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for multi-robot system with namespace isolation"""
    
    # Declare arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of robots to launch (1-3)'
    )
    
    fleet_id_arg = DeclareLaunchArgument(
        'fleet_id',
        default_value='WAREHOUSE-FLEET-A',
        description='Fleet identifier'
    )
    
    # Fleet Monitor - Central monitoring node (no namespace, monitors all robots)
    fleet_monitor_node = Node(
        package='ce_robot',
        executable='07_fleet_monitor',
        name='fleet_monitor',
        output='screen',
        parameters=[
            {'fleet_id': LaunchConfiguration('fleet_id')},
            {'num_robots': LaunchConfiguration('num_robots')},
            {'monitor_rate_hz': 0.5},
        ],
    )
    
    # Robot 1: Heavy transport robot in loading dock
    # Real-world: Handles pallets and heavy cargo, high priority operations
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='ce_robot',
            executable='07_robot_status',
            name='robot_status_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-TRANSPORT-HEAVY-001'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-A-LOADING-DOCK'},
                {'fleet_number': 1},
                {'max_payload_kg': 1000.0},  # Heavy-duty transport
                {'current_location': 'DOCK-A-STATION-5'},
                {'assigned_task': 'PALLET-TRANSPORT-TO-STORAGE'},
                {'battery_level': 78.0},
                {'status_rate_hz': 2.0},
            ],
        ),
        Node(
            package='ce_robot',
            executable='07_zone_coordinator',
            name='zone_coordinator',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-TRANSPORT-HEAVY-001'},
                {'zone_id': 'WAREHOUSE-A-LOADING-DOCK'},
                {'robot_type': 'transport'},
                {'max_capacity': 1000.0},
            ],
        ),
    ])
    
    # Robot 2: Picker robot with low battery - returning to charging
    # Real-world: Light picker for order fulfillment, needs charging
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='ce_robot',
            executable='07_robot_status',
            name='robot_status_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-PICKER-LIGHT-002'},
                {'robot_type': 'picker'},
                {'zone_id': 'WAREHOUSE-B-PICKING-AREA'},
                {'fleet_number': 2},
                {'max_payload_kg': 50.0},  # Light picker
                {'current_location': 'AISLE-B-12-SHELF-3'},
                {'assigned_task': 'RETURN-TO-CHARGING-STATION'},
                {'battery_level': 18.5},  # Low battery - needs charging!
                {'status_rate_hz': 1.5},
            ],
        ),
        Node(
            package='ce_robot',
            executable='07_zone_coordinator',
            name='zone_coordinator',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-PICKER-LIGHT-002'},
                {'zone_id': 'WAREHOUSE-B-PICKING-AREA'},
                {'robot_type': 'picker'},
                {'max_capacity': 50.0},
            ],
        ),
    ])
    
    # Robot 3: Multi-function delivery robot at sorting station
    # Real-world: Medium capacity, active delivery operations
    robot3_group = GroupAction([
        PushRosNamespace('robot3'),
        Node(
            package='ce_robot',
            executable='07_robot_status',
            name='robot_status_publisher',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-DELIVERY-MULTI-003'},
                {'robot_type': 'transport'},
                {'zone_id': 'WAREHOUSE-C-SORTING-STATION'},
                {'fleet_number': 3},
                {'max_payload_kg': 300.0},  # Medium capacity
                {'current_location': 'SORT-C-CONVEYOR-7'},
                {'assigned_task': 'PACKAGE-DELIVERY-ROUTE-12'},
                {'battery_level': 92.0},  # Fully charged, ready for operations
                {'status_rate_hz': 3.0},  # Fast updates for active operations
            ],
        ),
        Node(
            package='ce_robot',
            executable='07_zone_coordinator',
            name='zone_coordinator',
            output='screen',
            parameters=[
                {'robot_id': 'AMR-DELIVERY-MULTI-003'},
                {'zone_id': 'WAREHOUSE-C-SORTING-STATION'},
                {'robot_type': 'transport'},
                {'max_capacity': 300.0},
            ],
        ),
    ])
    
    return LaunchDescription([
        num_robots_arg,
        fleet_id_arg,
        fleet_monitor_node,
        robot1_group,
        robot2_group,
        robot3_group,
    ])
