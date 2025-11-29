#!/usr/bin/env python3
"""
Exercise 2: Database Client
Queries the student database
Usage: ros2 run ce_robot database_client <query_type> [id]
query_type: 0=all, 1=by_id
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class DatabaseClient(Node):
    def __init__(self):
        super().__init__('database_client')

    def query(self, query_type, query_id=0):
        """Query the database"""
        
        client = self.create_client(AddTwoInts, 'query_database')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        request = AddTwoInts.Request()
        request.a = query_type
        request.b = query_id
        
        self.get_logger().info(f'Querying database...')
        future = client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print('Usage: database_client <query_type> [id]')
        print('query_type: 0=all, 1=by_id')
        sys.exit(1)
    
    query_type = int(sys.argv[1])
    query_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    
    node = DatabaseClient()
    response = node.query(query_type, query_id)
    
    if response.sum >= 0:
        node.get_logger().info(f'Query successful: {response.sum}')
    else:
        node.get_logger().warn('Query returned no results')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
