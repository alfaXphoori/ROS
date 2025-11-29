#!/usr/bin/env python3
"""
Exercise 1: Temperature Conversion Client
Converts temperature using the server
Usage: ros2 run ce_robot temperature_converter_client <temp> <unit>
Units: 1=Celsius, 2=Fahrenheit, 3=Kelvin
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class TempConverterClient(Node):
    def __init__(self):
        super().__init__('temp_converter_client')

    def convert(self, temp, unit):
        """Request temperature conversion"""
        
        client = self.create_client(AddTwoInts, 'convert_temperature')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = AddTwoInts.Request()
        request.a = temp
        request.b = unit
        
        self.get_logger().info(f'Converting {temp} (unit={unit})')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print('Usage: temperature_converter_client <temperature> <unit>')
        print('Units: 1=Celsius, 2=Fahrenheit, 3=Kelvin')
        sys.exit(1)
    
    temp = int(sys.argv[1])
    unit = int(sys.argv[2])
    
    node = TempConverterClient()
    response = node.convert(temp, unit)
    
    units = {1: '°C', 2: '°F', 3: 'K'}
    
    if response.sum == -999:
        node.get_logger().error('Conversion failed')
    else:
        node.get_logger().info(
            f'Converted: {temp}{units[unit]} → {response.sum}°C'
        )
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
