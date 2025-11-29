#!/usr/bin/env python3
"""
Exercise 1: Temperature Conversion Server
Converts temperature between Celsius, Fahrenheit, and Kelvin
Service type: AddTwoInts (from example_interfaces)
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class TempConverterServer(Node):
    def __init__(self):
        super().__init__('temp_converter_server')
        
        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'convert_temperature',
            self.convert_callback
        )
        
        self.get_logger().info('Temperature Converter Server started')
        self.get_logger().info('Service: /convert_temperature')
        self.get_logger().info('Units: 1=Celsius, 2=Fahrenheit, 3=Kelvin')

    def convert_callback(self, request, response):
        """
        Convert temperature
        request.a = temperature value
        request.b = unit (1=C, 2=F, 3=K)
        response.sum = result
        """
        temp = request.a
        unit = request.b
        
        # Validate temperature
        if unit == 1:  # Celsius input
            if temp < -273:
                self.get_logger().warn('Temperature below absolute zero!')
                response.sum = -999
                return response
            
            # Convert to Fahrenheit and Kelvin for demo
            fahrenheit = (temp * 9/5) + 32
            kelvin = temp + 273
            
            self.get_logger().info(
                f'Converting {temp}°C → F: {fahrenheit:.2f}°F, K: {kelvin:.2f}K'
            )
            response.sum = int(fahrenheit)
        
        elif unit == 2:  # Fahrenheit input
            celsius = (temp - 32) * 5/9
            kelvin = celsius + 273
            
            self.get_logger().info(
                f'Converting {temp}°F → C: {celsius:.2f}°C, K: {kelvin:.2f}K'
            )
            response.sum = int(celsius)
        
        elif unit == 3:  # Kelvin input
            celsius = temp - 273
            fahrenheit = (celsius * 9/5) + 32
            
            self.get_logger().info(
                f'Converting {temp}K → C: {celsius:.2f}°C, F: {fahrenheit:.2f}°F'
            )
            response.sum = int(celsius)
        
        else:
            self.get_logger().error('Invalid unit. Use 1=C, 2=F, 3=K')
            response.sum = -999
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TempConverterServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
