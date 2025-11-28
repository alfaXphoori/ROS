#!/usr/bin/env python3
"""
Combined Sensor Publisher Node
Publishes combined sensor data (Temperature + Humidity + Pressure)
Uses a custom message type
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time


class CombinedSensorPublisher(Node):
    def __init__(self):
        super().__init__('combined_sensor_publisher')
        
        # Subscribe to individual sensor topics
        self.temp_subscription = self.create_subscription(
            Float64, 'sensor/temperature', self.temp_callback, 10)
        self.humidity_subscription = self.create_subscription(
            Float64, 'sensor/humidity', self.humidity_callback, 10)
        self.pressure_subscription = self.create_subscription(
            Float64, 'sensor/pressure', self.pressure_callback, 10)
        
        # Publisher for combined data
        self.combined_publisher = self.create_publisher(Float64, 'sensor/combined', 10)
        
        # Store latest sensor values
        self.temperature = 0.0
        self.humidity = 0.0
        self.pressure = 0.0
        
        self.get_logger().info('Combined Sensor Publisher started')
        self.get_logger().info('Waiting for sensor data...')

    def temp_callback(self, msg):
        self.temperature = msg.data
        self.publish_combined()

    def humidity_callback(self, msg):
        self.humidity = msg.data
        self.publish_combined()

    def pressure_callback(self, msg):
        self.pressure = msg.data
        self.publish_combined()

    def publish_combined(self):
        # Publish a combined message with all sensor values
        # Format: "temp,humidity,pressure"
        combined_msg = Float64()
        # For demonstration, sending average of all values
        combined_msg.data = (self.temperature + self.humidity + self.pressure) / 3
        self.combined_publisher.publish(combined_msg)
        self.get_logger().info(
            f'Combined: Temp={self.temperature:.2f}°C, '
            f'Humidity={self.humidity:.2f}%, '
            f'Pressure={self.pressure:.2f} hPa')


def main(args=None):
    rclpy.init(args=args)
    node = CombinedSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
