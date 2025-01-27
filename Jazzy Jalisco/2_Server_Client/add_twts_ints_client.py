#!/usr/bin/env python3
import sys
import rclpy
from functools import partial
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoInts_Client(Node):
    def __init__(self):
        super().__init__("Add_Two_Ints_Client")
        #self.callback_add_two_ints_server(1, 2)

    def callback_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("W8 for service server...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_add_two_ints, a=a, b=b)
        )
    def callback_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " +
                                   str(b) + " = " + 
                                   str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoInts_Client()
    reponse = node.callback_add_two_ints_server(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
