#!/usr/bin/env python3
"""
Temperature Unit Conversion Client
Sends temperature values to the server for conversion
- Send (value=temp, flag=1) for °C→°F
- Send (value=temp, flag=2) for °F→°C
- Send (value=temp, flag=3) for K→°C
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class TemperatureConverterClient(Node):
    def __init__(self):
        super().__init__('temp_converter_client')
        self.cli = self.create_client(
            AddTwoInts,
            'convert_temperature'
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, temperature, unit_flag):
        """
        Send temperature conversion request
        temperature: float value to convert
        unit_flag: 1 (C→F), 2 (F→K), 3 (K→C)
        """
        request = AddTwoInts.Request()
        request.a = int(temperature)
        request.b = unit_flag

        unit_map = {
            1: '°C → °F',
            2: '°F → °C',
            3: 'K → °C'
        }

        self.get_logger().info(
            f'Sending request: {temperature} ({unit_map.get(unit_flag, "Unknown")})'
        )

        future = self.cli.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    client = TemperatureConverterClient()

    # Example conversions
    print("\n=== Temperature Conversion Examples ===\n")

    # Example 1: Convert 25°C to °F
    print("Example 1: Convert 25°C to Fahrenheit")
    future1 = client.send_request(25, 1)
    rclpy.spin_until_future_complete(client, future1)
    result1 = future1.result()
    print(f"Result: 25°C = {result1.sum}°F\n")

    # Example 2: Convert 77°F to °C
    print("Example 2: Convert 77°F to Celsius")
    future2 = client.send_request(77, 2)
    rclpy.spin_until_future_complete(client, future2)
    result2 = future2.result()
    print(f"Result: 77°F = {result2.sum}°C\n")

    # Example 3: Convert 298K to °C
    print("Example 3: Convert 298K to Celsius")
    future3 = client.send_request(298, 3)
    rclpy.spin_until_future_complete(client, future3)
    result3 = future3.result()
    print(f"Result: 298K = {result3.sum}°C\n")

    # Interactive mode
    print("=== Interactive Mode ===\n")
    while True:
        try:
            print("\nConversion options:")
            print("1. Celsius → Fahrenheit (°C → °F)")
            print("2. Fahrenheit → Celsius (°F → °C)")
            print("3. Kelvin → Celsius (K → °C)")
            print("0. Exit")

            choice = input("\nSelect conversion (0-3): ").strip()

            if choice == '0':
                break
            elif choice in ['1', '2', '3']:
                temp = float(input("Enter temperature value: "))
                future = client.send_request(temp, int(choice))
                rclpy.spin_until_future_complete(client, future)
                result = future.result()
                print(f"✓ Converted result: {result.sum}")
            else:
                print("Invalid choice. Please enter 0-3.")

        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except ValueError:
            print("Invalid input. Please enter a valid number.")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
