#!/usr/bin/env python3

"""
Simple Map Saver - Save occupancy grid map to PGM and YAML files
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
import sys


class MapSaver(Node):
    def __init__(self, filename):
        super().__init__('map_saver')
        self.filename = filename
        self.map_received = False
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        print(f"Waiting for map on /map topic...")
    
    def map_callback(self, msg):
        if self.map_received:
            return
        
        self.map_received = True
        print(f"Map received! Saving to {self.filename}...")
        
        # Extract map data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin
        
        # Convert map data to 2D array
        map_data = np.array(msg.data).reshape((height, width))
        
        # Convert to PGM format (0-255)
        # ROS: -1=unknown, 0=free, 100=occupied
        # PGM: 255=free, 0=occupied, 205=unknown
        pgm_data = np.zeros((height, width), dtype=np.uint8)
        
        for y in range(height):
            for x in range(width):
                cell = map_data[y, x]
                if cell == -1:  # Unknown
                    pgm_data[y, x] = 205
                elif cell == 0:  # Free
                    pgm_data[y, x] = 254
                else:  # Occupied (cell > 0)
                    pgm_data[y, x] = 0
        
        # Flip vertically (PGM origin is top-left, ROS is bottom-left)
        pgm_data = np.flipud(pgm_data)
        
        # Save PGM file
        pgm_filename = f"{self.filename}.pgm"
        with open(pgm_filename, 'wb') as f:
            # PGM header
            f.write(f"P5\n".encode())
            f.write(f"# CREATOR: map_saver.py\n".encode())
            f.write(f"{width} {height}\n".encode())
            f.write(f"255\n".encode())
            # Write binary data
            f.write(pgm_data.tobytes())
        
        print(f"✓ Saved map image: {pgm_filename}")
        
        # Save YAML metadata
        yaml_filename = f"{self.filename}.yaml"
        yaml_data = {
            'image': pgm_filename,
            'resolution': float(resolution),
            'origin': [
                float(origin.position.x),
                float(origin.position.y),
                float(origin.position.z)
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(yaml_filename, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        print(f"✓ Saved map metadata: {yaml_filename}")
        print(f"\n✅ Map saved successfully!")
        print(f"   Resolution: {resolution}m/pixel")
        print(f"   Size: {width}x{height} pixels")
        print(f"   Origin: ({origin.position.x:.2f}, {origin.position.y:.2f})")
        
        # Shutdown
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run ce_webots save_map <filename>")
        print("Example: ros2 run ce_webots save_map my_map")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    rclpy.init(args=args)
    saver = MapSaver(filename)
    
    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
