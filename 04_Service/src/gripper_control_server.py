#!/usr/bin/env python3
"""
Exercise 2: Gripper Control Service Server
Controls robot gripper for pick and place operations
"""

import rclpy
import time
import random
from rclpy.node import Node
from ce_robot_interfaces.srv import GripperCommand


class GripperControlServer(Node):
    def __init__(self):
        super().__init__('gripper_control_server')
        
        self.srv = self.create_service(
            GripperCommand,
            'gripper_command',
            self.gripper_callback
        )
        
        # Gripper state
        self.current_position = 1.0  # 1.0 = fully open
        self.current_force = 0.0
        self.object_in_gripper = False
        
        # Gripper specifications
        self.min_position = 0.0   # Fully closed
        self.max_position = 1.0   # Fully open
        self.max_force = 1.0      # Maximum grip force
        
        self.command_count = 0
        
        self.get_logger().info('🤖 Gripper Control Server started')
        self.get_logger().info(f'   Current state: Position={self.current_position:.2f}, Force={self.current_force:.2f}')

    def simulate_object_detection(self, position):
        """Simulate object detection based on gripper position"""
        # Object detected if gripper is somewhat closed (0.2-0.6 range)
        if 0.2 <= position <= 0.6:
            return random.choice([True, True, False])  # 66% detection rate
        return False

    def gripper_callback(self, request, response):
        """Handle gripper command"""
        self.command_count += 1
        
        command = request.command
        position = request.position
        force = request.force
        check_object = request.check_object
        
        self.get_logger().info(
            f'\n🤖 Gripper Command #{self.command_count}:\n'
            f'   Command: {command} (1=Open, 2=Close, 3=SetPosition)\n'
            f'   Position: {position:.2f} | Force: {force:.2f}\n'
            f'   Check Object: {check_object}'
        )
        
        # Validate command
        if command not in [1, 2, 3]:
            response.success = False
            response.object_detected = False
            response.actual_position = self.current_position
            response.actual_force = self.current_force
            response.message = '❌ Invalid command! Use 1=Open, 2=Close, 3=SetPosition'
            self.get_logger().error(response.message)
            return response
        
        # Process command
        if command == 1:  # Open
            target_position = 1.0
            self.get_logger().info('📂 Opening gripper...')
            
        elif command == 2:  # Close
            target_position = 0.0
            self.get_logger().info('📁 Closing gripper...')
            
        elif command == 3:  # Set Position
            # Validate position
            if not (0.0 <= position <= 1.0):
                response.success = False
                response.object_detected = False
                response.actual_position = self.current_position
                response.actual_force = self.current_force
                response.message = f'❌ Invalid position: {position}. Must be 0.0-1.0'
                self.get_logger().error(response.message)
                return response
            
            target_position = position
            self.get_logger().info(f'🎯 Setting gripper to position {target_position:.2f}...')
        
        # Validate force
        if not (0.0 <= force <= 1.0):
            force = min(max(force, 0.0), 1.0)
            self.get_logger().warn(f'⚠️  Force clamped to {force:.2f}')
        
        # Simulate movement time
        movement_distance = abs(target_position - self.current_position)
        movement_time = movement_distance * 2.0  # 2 seconds per full range
        time.sleep(min(movement_time / 5, 1.0))  # Scaled simulation
        
        # Update gripper state
        self.current_position = target_position
        self.current_force = force if target_position < 0.5 else 0.0  # Force only when closing
        
        # Check for object if requested
        if check_object:
            self.object_in_gripper = self.simulate_object_detection(self.current_position)
        else:
            self.object_in_gripper = False
        
        # Prepare response
        response.success = True
        response.object_detected = self.object_in_gripper
        response.actual_position = float(self.current_position)
        response.actual_force = float(self.current_force)
        
        if self.object_in_gripper:
            response.message = f'✅ Gripper command complete! Object detected at position {self.current_position:.2f}'
        else:
            response.message = f'✅ Gripper at position {self.current_position:.2f}'
        
        self.get_logger().info(
            f'✅ Complete: Position={self.current_position:.2f}, '
            f'Force={self.current_force:.2f}, Object={self.object_in_gripper}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🤖 Gripper Control Server shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
