#!/usr/bin/env python3
"""
Tutorial 00: Keyboard Teleoperation Controller

Basic keyboard control for differential drive robot.
Learn manual control before adding sensors.

Controls:
    Q   W   E       Speed Control:
    A   S   D       +  Increase speed
        X           -  Decrease speed
                    ESC  Quit

Movement:
    W - Forward          Q - Forward-Left
    S - Backward         E - Forward-Right
    A - Spin Left        X - Stop
    D - Spin Right
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios


class KeyboardTeleopNode(Node):
    """Keyboard teleoperation node for Tutorial 00"""
    
    def __init__(self):
        super().__init__('keyboard_teleop_00')
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Default velocities
        self.linear_vel = 0.5   # m/s
        self.angular_vel = 0.5  # rad/s
        
        # Velocity increments
        self.linear_step = 0.1
        self.angular_step = 0.1
        
        # Key mapping: key -> (linear, angular)
        self.key_map = {
            'w': (1.0, 0.0),    # Forward
            's': (-1.0, 0.0),   # Backward
            'a': (0.0, 1.0),    # Spin left
            'd': (0.0, -1.0),   # Spin right
            'q': (1.0, 1.0),    # Forward-left
            'e': (1.0, -1.0),   # Forward-right
            'x': (0.0, 0.0),    # Stop
        }
        
        self.get_logger().info('Keyboard Teleop Controller Started')
        self.print_instructions()
    
    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("Keyboard Teleop Controller")
        print("="*60)
        print("Controls:")
        print("    Q   W   E          Speed Control")
        print("    A   S   D          +  Increase")
        print("        X              -  Decrease")
        print()
        print("Legend:")
        print("W : Forward          Q : Forward-Left")
        print("S : Backward         E : Forward-Right  ")
        print("A : Turn Left        X : Stop")
        print("D : Turn Right       ESC : Quit")
        print("="*60)
        print(f"Current: v={self.linear_vel:.2f} m/s, ω={self.angular_vel:.2f} rad/s")
        print()
    
    def get_key(self):
        """Read a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def publish_velocity(self, linear_factor, angular_factor):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear_factor * self.linear_vel
        twist.angular.z = angular_factor * self.angular_vel
        self.publisher.publish(twist)
        
        # Show current command
        if linear_factor != 0.0 or angular_factor != 0.0:
            self.get_logger().info(
                f'v={twist.linear.x:.2f} m/s, ω={twist.angular.z:.2f} rad/s'
            )
    
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # ESC to quit
                if key == '\x1b':  # ESC
                    self.get_logger().info('Exiting...')
                    self.publish_velocity(0.0, 0.0)  # Stop robot
                    break
                
                # Movement keys
                elif key in self.key_map:
                    linear_factor, angular_factor = self.key_map[key]
                    self.publish_velocity(linear_factor, angular_factor)
                
                # Increase speed
                elif key == '+' or key == '=':
                    self.linear_vel = min(self.linear_vel + self.linear_step, 2.0)
                    self.angular_vel = min(self.angular_vel + self.angular_step, 2.0)
                    self.get_logger().info(
                        f'Speed increased: v={self.linear_vel:.2f} m/s, '
                        f'ω={self.angular_vel:.2f} rad/s'
                    )
                
                # Decrease speed
                elif key == '-' or key == '_':
                    self.linear_vel = max(self.linear_vel - self.linear_step, 0.1)
                    self.angular_vel = max(self.angular_vel - self.angular_step, 0.1)
                    self.get_logger().info(
                        f'Speed decreased: v={self.linear_vel:.2f} m/s, '
                        f'ω={self.angular_vel:.2f} rad/s'
                    )
                
                # Help
                elif key == 'h' or key == 'H':
                    self.print_instructions()
                
                # Unknown key
                else:
                    # Ignore unknown keys silently
                    pass
                
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
            self.publish_velocity(0.0, 0.0)  # Stop robot
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.publish_velocity(0.0, 0.0)  # Stop robot


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = KeyboardTeleopNode()
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
