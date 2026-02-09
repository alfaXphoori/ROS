#!/usr/bin/env python3
"""
Navigation Service Node - Path Planning with Obstacle Avoidance
Real-world use: Calculate safe navigation paths around obstacles in warehouse
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import NavigationPath
import math


class NavigationService(Node):
    def __init__(self):
        super().__init__('rect_server')
        
        # Declare robot parameters
        self.declare_parameter('robot_id', 'AMR-WH-A-001')
        self.declare_parameter('robot_type', 'transport')
        self.declare_parameter('zone_id', 'WAREHOUSE-A-DOCK-3')
        self.declare_parameter('max_payload_kg', 500.0)
        self.declare_parameter('safety_margin_m', 0.5)
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.zone_id = self.get_parameter('zone_id').value
        self.max_payload = self.get_parameter('max_payload_kg').value
        self.safety_margin = self.get_parameter('safety_margin_m').value
        
        # Create service for navigation path calculation
        self.srv = self.create_service(
            NavigationPath,
            '/navigation_path',
            self.calculate_navigation_path_callback
        )
        
        self.get_logger().info('🟢 Navigation Service READY')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'Robot Type: {self.robot_type}')
        self.get_logger().info(f'Operating Zone: {self.zone_id}')
        self.get_logger().info(f'Max Payload: {self.max_payload} kg')
        self.get_logger().info(f'Safety Margin: {self.safety_margin} m')
        self.get_logger().info('Service: /navigation_path')
        self.get_logger().info('Purpose: Calculate safe navigation paths for obstacle avoidance')
        
    def calculate_navigation_path_callback(self, request, response):
        """
        Calculate safe navigation path with obstacle avoidance
        Real-world: Determine if robot can safely navigate around detected obstacle
        
        Args:
            obstacle_length: Obstacle length in meters
            obstacle_width: Obstacle width in meters
            robot_x, robot_y: Current robot position
            safety_margin: Required safety clearance
            zone_id: Warehouse zone identifier
        
        Returns:
            safe_area: Safe navigation area (m²)
            path_perimeter: Boundary perimeter (m)
            clearance_left/right: Side clearances (m)
            can_navigate: Whether path is safe
            recommended_action: Navigation recommendation
            estimated_time: Time to navigate (seconds)
        """
        # Use request safety margin or default
        safety = request.safety_margin if request.safety_margin > 0 else self.safety_margin
        
        # Add safety margins to obstacle dimensions
        safe_length = request.obstacle_length + (2 * safety)
        safe_width = request.obstacle_width + (2 * safety)
        
        # Calculate safe area and perimeter
        response.safe_area = safe_length * safe_width
        response.path_perimeter = 2 * (safe_length + safe_width)
        
        # Calculate clearances (simulate corridor width)
        corridor_width = 4.0  # meters (typical warehouse aisle)
        response.clearance_left = (corridor_width - safe_width) / 2
        response.clearance_right = response.clearance_left
        
        # Robot width (depends on type)
        robot_widths = {'transport': 1.2, 'picker': 0.6, 'sorter': 0.8}
        robot_width = robot_widths.get(self.robot_type, 0.8)
        
        # Determine if robot can navigate
        min_clearance_needed = robot_width + 0.3  # robot width + safety buffer
        response.can_navigate = min(response.clearance_left, response.clearance_right) >= min_clearance_needed
        
        # Recommended action based on clearance
        if not response.can_navigate:
            response.recommended_action = "STOP"
        elif response.clearance_left < (min_clearance_needed + 0.2):
            response.recommended_action = "REDUCE_SPEED"
        elif response.clearance_left < (min_clearance_needed + 0.5):
            response.recommended_action = "PROCEED"
        else:
            response.recommended_action = "PROCEED"
        
        # Estimate navigation time (distance / speed)
        distance = math.sqrt((request.robot_x ** 2) + (request.robot_y ** 2))
        speed = 1.5 if response.recommended_action == "PROCEED" else 0.5  # m/s
        response.estimated_time = distance / speed if distance > 0 else 0.0
        
        # Log detailed information
        self.get_logger().info(
            f'📐 Navigation Path Calculation [{self.robot_id}]:\n'
            f'   Zone: {request.zone_id if request.zone_id else self.zone_id}\n'
            f'   Robot Position: ({request.robot_x:.2f}, {request.robot_y:.2f})m\n'
            f'   Detected Obstacle: {request.obstacle_length:.2f}m x {request.obstacle_width:.2f}m\n'
            f'   Safety Buffer: +{safety:.2f}m on all sides\n'
            f'   Safe Zone Required: {safe_length:.2f}m x {safe_width:.2f}m\n'
            f'   Safe Zone Area: {response.safe_area:.2f} m²\n'
            f'   Path Perimeter: {response.path_perimeter:.2f} m\n'
            f'   Left Clearance: {response.clearance_left:.2f}m | Right Clearance: {response.clearance_right:.2f}m\n'
            f'   Robot Width: {robot_width:.2f}m ({self.robot_type})\n'
            f'   Can Navigate: {"✅ YES" if response.can_navigate else "❌ NO - REROUTE REQUIRED"}\n'
            f'   Recommended Action: {response.recommended_action}\n'
            f'   Estimated Time: {response.estimated_time:.1f}s'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    navigation_service = NavigationService()
    
    try:
        rclpy.spin(navigation_service)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_service.get_logger().info('🔴 Navigation Service shutting down')
        navigation_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
