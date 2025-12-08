#!/usr/bin/env python3
"""
Temperature Unit Conversion Client
Usage: ros2 run ce_robot temp_converter_client <temperature> <flag>

Examples:
  ros2 run ce_robot temp_converter_client 25 1    # Convert 25°C to °F
  ros2 run ce_robot temp_converter_client 77 2    # Convert 77°F to °C
  ros2 run ce_robot temp_converter_client 298 3   # Convert 298K to °C

Flags:
  1: Celsius → Fahrenheit (°C → °F)
  2: Fahrenheit → Celsius (°F → °C)
  3: Kelvin → Celsius (K → °C)
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
        unit_flag: 1 (C→F), 2 (F→C), 3 (K→C)
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

    # Parse command-line arguments
    if len(sys.argv) < 3:
        print("\n=== Temperature Unit Conversion Client ===\n")
        print("Usage: ros2 run ce_robot temp_converter_client <temperature> <flag>\n")
        print("Examples:")
        print("  ros2 run ce_robot temp_converter_client 25 1    # Convert 25°C to °F")
        print("  ros2 run ce_robot temp_converter_client 77 2    # Convert 77°F to °C")
        print("  ros2 run ce_robot temp_converter_client 298 3   # Convert 298K to °C\n")
        print("Conversion Flags:")
        print("  1: Celsius → Fahrenheit (°C → °F)")
        print("  2: Fahrenheit → Celsius (°F → °C)")
        print("  3: Kelvin → Celsius (K → °C)\n")
        client.destroy_node()
        rclpy.shutdown()
        return

    try:
        temperature = float(sys.argv[1])
        unit_flag = int(sys.argv[2])

        # Validate flag
        if unit_flag not in [1, 2, 3]:
            print(f"\n❌ Error: Invalid flag '{unit_flag}'")
            print("Use: 1 (C→F), 2 (F→C), or 3 (K→C)\n")
            client.destroy_node()
            rclpy.shutdown()
            return

        # Send conversion request
        print(f"\n🔄 Converting {temperature}...")
        future = client.send_request(temperature, unit_flag)
        rclpy.spin_until_future_complete(client, future)
        result = future.result()

        # Display result
        unit_names = {
            1: ('°C', '°F'),
            2: ('°F', '°C'),
            3: ('K', '°C')
        }
        from_unit, to_unit = unit_names[unit_flag]
        print(f"✓ Result: {temperature}{from_unit} = {result.sum}{to_unit}\n")

    except ValueError as e:
        print(f"\n❌ Error: Invalid input. Please enter valid numbers.")
        print(f"Usage: ros2 run ce_robot temp_converter_client <temperature> <flag>\n")
    except Exception as e:
        print(f"\n❌ Error: {str(e)}\n")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

