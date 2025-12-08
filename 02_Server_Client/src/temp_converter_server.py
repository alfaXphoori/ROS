#!/usr/bin/env python3
"""
Temperature Unit Conversion Server
Converts temperatures between Celsius, Fahrenheit, and Kelvin
- Input °C → Output °F
- Input °F → Output K (Kelvin)
- Input K → Output °C
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class TemperatureConverterServer(Node):
    def __init__(self):
        super().__init__('temp_converter_server')
        self.srv = self.create_service(
            AddTwoInts,
            'convert_temperature',
            self.convert_temperature_callback
        )
        self.get_logger().info('Temperature Converter Server started')
        self.get_logger().info('Service: convert_temperature')
        self.get_logger().info('Logic:')
        self.get_logger().info('  - Send (value=temp, b=1) for °C→°F conversion')
        self.get_logger().info('  - Send (value=temp, b=2) for °F→K conversion')
        self.get_logger().info('  - Send (value=temp, b=3) for K→°C conversion')

    def convert_temperature_callback(self, request, response):
        """
        Convert temperature based on unit flag
        request.a: temperature value
        request.b: unit flag (1=C→F, 2=F→K, 3=K→C)
        response.sum: converted temperature (as integer)
        """
        temp_value = float(request.a)
        unit_flag = request.b

        try:
            if unit_flag == 1:  # Celsius to Fahrenheit
                # Formula: F = (C × 9/5) + 32
                result = (temp_value * 9 / 5) + 32
                self.get_logger().info(
                    f'Convert {temp_value}°C → {result:.2f}°F'
                )

            elif unit_flag == 2:  # Fahrenheit to Kelvin
                # Formula: K = (F - 32) × 5/9 + 273.15
                result = (temp_value - 32) * 5 / 9 + 273.15
                self.get_logger().info(
                    f'Convert {temp_value}°F → {result:.2f}K'
                )

            elif unit_flag == 3:  # Kelvin to Celsius
                # Formula: C = K - 273.15
                result = temp_value - 273.15
                self.get_logger().info(
                    f'Convert {temp_value}K → {result:.2f}°C'
                )

            else:
                self.get_logger().warn(
                    f'Unknown unit flag: {unit_flag}. '
                    'Use 1 (C→F), 2 (F→K), or 3 (K→C)'
                )
                result = 0

            response.sum = int(result)
            return response

        except Exception as e:
            self.get_logger().error(f'Conversion error: {str(e)}')
            response.sum = 0
            return response


def main(args=None):
    rclpy.init(args=args)
    server = TemperatureConverterServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
