#!/usr/bin/env python3
"""
Exercise 2: Database Query Server
Maintains a student database and handles queries
Service type: AddTwoInts (from example_interfaces)
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class DatabaseServer(Node):
    def __init__(self):
        super().__init__('database_server')
        
        # Initialize student database
        self.students = {
            1: {'name': 'Alice', 'gpa': 3.8, 'major': 'CS'},
            2: {'name': 'Bob', 'gpa': 3.5, 'major': 'EE'},
            3: {'name': 'Charlie', 'gpa': 3.9, 'major': 'CS'},
            4: {'name': 'Diana', 'gpa': 3.6, 'major': 'ME'},
            5: {'name': 'Eve', 'gpa': 3.7, 'major': 'CS'},
        }
        
        self.srv = self.create_service(
            AddTwoInts,
            'query_database',
            self.query_callback
        )
        
        self.get_logger().info('Database Server started')
        self.get_logger().info(f'Database loaded: {len(self.students)} records')

    def query_callback(self, request, response):
        """Handle database queries"""
        
        query_type = request.a  # 0=all, 1=by_id
        query_id = request.b
        
        if query_type == 0:  # Get all
            count = len(self.students)
            self.get_logger().info(f'Query: All records ({count} total)')
            response.sum = count
        
        elif query_type == 1:  # Get by ID
            if query_id in self.students:
                student = self.students[query_id]
                self.get_logger().info(
                    f'Query: ID {query_id} → {student["name"]} '
                    f'(GPA: {student["gpa"]}, Major: {student["major"]})'
                )
                response.sum = query_id  # Return success
            else:
                self.get_logger().warn(f'Query: ID {query_id} not found')
                response.sum = -1  # Not found
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DatabaseServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
