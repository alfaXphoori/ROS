#!/usr/bin/env python3
"""
Exercise 2: Database Client
Queries the student database with formatted output
Usage: ros2 run ce_robot database_client <query_type> [id]
query_type: 0=all, 1=by_id

Examples:
  ros2 run ce_robot database_client 0      # Get all records
  ros2 run ce_robot database_client 1 1    # Get student with ID 1
  ros2 run ce_robot database_client 1 3    # Get student with ID 3
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class DatabaseClient(Node):
    def __init__(self):
        super().__init__('database_client')
        
        # Student database for display
        self.students = {
            1: {'name': 'Alice', 'gpa': 3.8, 'major': 'CS'},
            2: {'name': 'Bob', 'gpa': 3.5, 'major': 'EE'},
            3: {'name': 'Charlie', 'gpa': 3.9, 'major': 'CS'},
            4: {'name': 'Diana', 'gpa': 3.6, 'major': 'ME'},
            5: {'name': 'Eve', 'gpa': 3.7, 'major': 'CS'},
        }

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
        print('\n=== Database Query Client ===\n')
        print('Usage: database_client <query_type> [id]\n')
        print('Query Types:')
        print('  0 = Get all records')
        print('  1 = Get record by ID\n')
        print('Examples:')
        print('  ros2 run ce_robot database_client 0      # Get all records')
        print('  ros2 run ce_robot database_client 1 1    # Get student with ID 1')
        print('  ros2 run ce_robot database_client 1 3    # Get student with ID 3\n')
        sys.exit(1)
    
    try:
        query_type = int(sys.argv[1])
        query_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
        
        node = DatabaseClient()
        response = node.query(query_type, query_id)
        
        print('\n' + '='*60)
        print('📊 DATABASE QUERY RESULT')
        print('='*60 + '\n')
        
        if query_type == 0:  # Get all records
            if response.sum > 0:
                print(f'✓ Total Records: {response.sum}\n')
                print(f'{"ID":<5} {"Name":<15} {"GPA":<8} {"Major":<10}')
                print('-'*48)
                for sid, student in node.students.items():
                    print(f'{sid:<5} {student["name"]:<15} {student["gpa"]:<8} {student["major"]:<10}')
                print()
            else:
                print('❌ No records found\n')
        
        elif query_type == 1:  # Get by ID
            if response.sum >= 0 and response.sum in node.students:
                student = node.students[response.sum]
                print(f'✓ Query Successful\n')
                print(f'ID:    {response.sum}')
                print(f'Name:  {student["name"]}')
                print(f'GPA:   {student["gpa"]}')
                print(f'Major: {student["major"]}\n')
            else:
                print(f'❌ Student with ID {query_id} not found\n')
        
        else:
            print(f'❌ Invalid query type: {query_type}\n')
        
        print('='*60 + '\n')
        
        node.destroy_node()
        rclpy.shutdown()
        
    except ValueError:
        print(f'\n❌ Error: Invalid input. Please enter valid numbers.\n')
        sys.exit(1)
    except Exception as e:
        print(f'\n❌ Error: {str(e)}\n')
        sys.exit(1)


if __name__ == '__main__':
    main()
