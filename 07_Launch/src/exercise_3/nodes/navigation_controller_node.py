#!/usr/bin/env python3
"""
Exercise 3: Navigation Controller Node
CRITICAL SYSTEM - Real-world robot navigation with safety monitoring
Handles path planning, obstacle avoidance, and collision prevention
Failure of this node requires immediate system shutdown for safety
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from std_msgs.msg import String
import json
import random


class NavigationControllerNode(Node):
    """
    Safety-critical navigation controller for warehouse robots
    Monitors: position, obstacles, path planning, collision avoidance
    """
    
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'AMR-NAV-001')
        self.declare_parameter('robot_type', 'transport')
        self.declare_parameter('zone_id', 'WAREHOUSE-MAIN-FLOOR')
        self.declare_parameter('max_speed_ms', 2.5)  # meters per second
        self.declare_parameter('safety_radius_m', 0.75)  # Safety bubble around robot
        self.declare_parameter('lidar_range_m', 20.0)
        self.declare_parameter('status_rate_hz', 1.0)
        self.declare_parameter('simulate_failure', False)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.max_speed = self.get_parameter('max_speed_ms').value
        self.safety_radius = self.get_parameter('safety_radius_m').value
        self.lidar_range = self.get_parameter('lidar_range_m').value
        self.rate = self.get_parameter('status_rate_hz').value
        self.simulate_failure = self.get_parameter('simulate_failure').value
        
        # Navigation state
        self.position_x = 5.0
        self.position_y = 3.0
        self.heading = 90.0  # degrees
        self.current_speed = 0.0
        self.target_x = 15.0
        self.target_y = 8.0
        self.distance_to_target = 0.0
        self.obstacles_detected = 0
        self.path_clear = True
        self.navigation_active = False
        self.emergency_stop = False
        
        # Service for navigation commands
        self.service = self.create_service(
            SetBool,
            'navigation_command',
            self.navigation_command_callback
        )
        
        # Publisher for navigation status
        self.publisher = self.create_publisher(
            String,
            'navigation_status',
            10
        )
        
        # Timer for status updates
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.publish_navigation_status
        )
        
        self.iteration = 0
        
        self.get_logger().info(f'🗺️  Navigation Controller Started')
        self.get_logger().info(f'   Robot ID: {self.robot_id}')
        self.get_logger().info(f'   Max Speed: {self.max_speed} m/s')
        self.get_logger().info(f'   Safety Radius: {self.safety_radius} m')
        self.get_logger().info(f'   LIDAR Range: {self.lidar_range} m')
        
    def navigation_command_callback(self, request, response):
        """Handle navigation commands (start/stop)"""
        
        if request.data:
            self.navigation_active = True
            self.current_speed = self.max_speed * 0.7  # Start at 70% speed
            response.success = True
            response.message = f'Navigation activated for {self.robot_id}'
            self.get_logger().info('🚀 Navigation STARTED')
        else:
            self.navigation_active = False
            self.current_speed = 0.0
            response.success = True
            response.message = f'Navigation stopped for {self.robot_id}'
            self.get_logger().info('🛑 Navigation STOPPED')
        
        return response
    
    def publish_navigation_status(self):
        """Publish real-time navigation status"""
        
        self.iteration += 1
        
        # Simulate robot movement if navigation is active
        if self.navigation_active:
            # Update position (simplified motion)
            dx = self.target_x - self.position_x
            dy = self.target_y - self.position_y
            distance = (dx**2 + dy**2)**0.5
            self.distance_to_target = distance
            
            if distance > 0.1:  # Still moving to target
                # Normalize direction and move
                move_x = (dx / distance) * self.current_speed * (1.0 / self.rate)
                move_y = (dy / distance) * self.current_speed * (1.0 / self.rate)
                self.position_x += move_x
                self.position_y += move_y
                
                # Update heading
                import math
                self.heading = math.degrees(math.atan2(dy, dx))
            else:
                # Reached target
                self.navigation_active = False
                self.current_speed = 0.0
                self.get_logger().info('✅ Target reached!')
        
        # Simulate obstacle detection
        self.obstacles_detected = random.randint(0, 3)
        self.path_clear = self.obstacles_detected == 0
        
        # Safety check: Emergency stop if obstacle too close
        if self.obstacles_detected > 2 and self.navigation_active:
            self.emergency_stop = True
            self.current_speed = 0.0
            self.navigation_active = False
            self.get_logger().warn('🚨 EMERGENCY STOP: Obstacles detected!')
        
        # Create navigation status message
        nav_data = {
            'robot_id': self.robot_id,
            'robot_type': self.robot_type,
            'zone_id': self.zone_id,
            'timestamp': self.get_clock().now().to_msg().sec,
            'navigation': {
                'active': self.navigation_active,
                'position': {
                    'x': round(self.position_x, 2),
                    'y': round(self.position_y, 2)
                },
                'target': {
                    'x': self.target_x,
                    'y': self.target_y
                },
                'heading_deg': round(self.heading, 1),
                'current_speed_ms': round(self.current_speed, 2),
                'distance_to_target_m': round(self.distance_to_target, 2),
                'obstacles_detected': self.obstacles_detected,
                'path_clear': self.path_clear,
                'emergency_stop': self.emergency_stop
            },
            'safety': {
                'safety_radius_m': self.safety_radius,
                'lidar_range_m': self.lidar_range,
                'collision_risk': 'high' if self.obstacles_detected > 2 else 'low'
            }
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(nav_data)
        self.publisher.publish(msg)
        
        # Log status periodically
        if self.iteration % int(self.rate * 5) == 0:  # Every 5 seconds
            status = 'ACTIVE' if self.navigation_active else 'IDLE'
            self.get_logger().info(
                f'🗺️  Nav [{status}]: Pos({self.position_x:.1f}, {self.position_y:.1f}) | '
                f'Speed: {self.current_speed:.1f} m/s | '
                f'Distance: {self.distance_to_target:.1f}m | '
                f'Obstacles: {self.obstacles_detected}'
            )
        
        # Simulate failure for testing event handlers
        if self.simulate_failure and self.iteration > 15:
            self.get_logger().fatal('💥 CRITICAL: Navigation controller crashed!')
            raise RuntimeError('Simulated navigation failure - SAFETY CRITICAL')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Navigation controller exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
