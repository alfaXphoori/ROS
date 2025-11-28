#!/usr/bin/env python3
import sys
import rclpy
from functools import partial
from rclpy.node import Node
from ce_robot_interfaces.srv import CalCircle

class Cal_CirCle_Client(Node):
    def __init__(self):
        super().__init__("Cal_Circle_Client")
        #self.callback_add_two_ints_server(1.1, 2.2)

    def callback_add_two_ints_server(self, r):
        client = self.create_client(CalCircle, "cal_circle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("W8 for service server...")

        request = CalCircle.Request()
        request.r = r

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_add_value, a=r)
        )
    def callback_add_value(self, future, a):
        try:
            response = future.result()
            self.get_logger().info("Phoori Chantima -> R: "+str(a) + " = " + 
                                   "Area: "+str(response.area))
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))

def main(args=None):
    rclpy.init(args=args)
    node = Cal_CirCle_Client()
    reponse = node.callback_add_two_ints_server(float(sys.argv[1]))
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()