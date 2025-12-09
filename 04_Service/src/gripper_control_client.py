#!/usr/bin/env python3
"""
Exercise 2: Gripper Control Service Client
Sends gripper commands
Usage: ros2 run ce_robot gripper_client <command> [position] [force] [check_object]
Commands: 1=Open, 2=Close, 3=SetPosition
"""

import sys
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import GripperCommand


class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')

    def send_command(self, command, position=0.5, force=0.5, check_object=True):
        """Send gripper command"""
        
        client = self.create_client(GripperCommand, 'gripper_command')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for gripper service...')
        
        request = GripperCommand.Request()
        request.command = command
        request.position = position
        request.force = force
        request.check_object = check_object
        
        cmd_names = {1: "OPEN", 2: "CLOSE", 3: "SET_POSITION"}
        self.get_logger().info(
            f'🤖 Sending gripper command: {cmd_names.get(command, "UNKNOWN")}'
        )
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print('Usage: gripper_client <command> [position] [force] [check_object]')
        print('Commands:')
        print('  1 = Open gripper')
        print('  2 = Close gripper')
        print('  3 = Set position (requires position 0.0-1.0)')
        print('Examples:')
        print('  gripper_client 1              # Open')
        print('  gripper_client 2              # Close')
        print('  gripper_client 3 0.5 0.8 true # Set to 50%, force 80%, check object')
        sys.exit(1)
    
    try:
        command = int(sys.argv[1])
        position = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
        force = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
        check_object = sys.argv[4].lower() == 'true' if len(sys.argv) > 4 else True
    except ValueError:
        print('Error: Invalid input values')
        sys.exit(1)
    
    node = GripperClient()
    response = node.send_command(command, position, force, check_object)
    
    if response.success:
        node.get_logger().info(
            f'\n✅ Gripper Command Success!\n'
            f'   Position: {response.actual_position:.2f}\n'
            f'   Force: {response.actual_force:.2f}\n'
            f'   Object Detected: {response.object_detected}\n'
            f'   Message: {response.message}'
        )
    else:
        node.get_logger().error(
            f'\n❌ Gripper Command Failed!\n'
            f'   Message: {response.message}'
        )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
