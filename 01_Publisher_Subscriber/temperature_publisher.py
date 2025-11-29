#!/usr/bin/env python3
"""
Exercise 5: Temperature Sensor Simulation
Publishes simulated temperature data with random variation
Temperature range: 20-30°C
Publish rate: Every 2 seconds
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        # Create publisher with Float32 message type
        self.pub = self.create_publisher(Float32, 'temperature', 10)
        
        # Create timer to publish every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Base temperature (center point)
        self.base_temp = 25.0
        
        # Counter for display
        self.count = 0
        
        self.get_logger().info('Temperature Publisher started')
        self.get_logger().info('Publishing temperature data every 2 seconds...')

    def timer_callback(self):
        # Generate temperature with random variation
        # Range: base_temp ± 1°C = 24-26°C around base
        # Overall range: 20-30°C
        temperature = self.base_temp + random.uniform(-1, 1)
        
        # Clamp temperature to valid range
        temperature = max(20.0, min(30.0, temperature))
        
        # Create Float32 message
        msg = Float32()
        msg.data = temperature
        
        # Publish message
        self.pub.publish(msg)
        
        # Log with formatted output
        self.count += 1
        self.get_logger().info(
            f'[#{self.count}] Temperature: {temperature:.2f}°C'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
