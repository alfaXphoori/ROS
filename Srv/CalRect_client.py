#!/usr/bin/env python3
import sys
import rclpy
from functools import partial
from rclpy.node import Node
from ce_robot_interfaces.srv import CalRectangle

class AddTwoInts_Client(Node):
    def __init__(self):
        super().__init__("Cal_Rect_Client")
        #self.callback_add_two_ints_server(1.1, 2.2)

    def callback_add_two_ints_server(self, length, width):
        client = self.create_client(CalRectangle, "cal_rect")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("W8 for service server...")

        request = CalRectangle.Request()
        request.length = length
        request.width = width

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_add_two_ints, a=length, b=width)
        )
    def callback_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info("L: "+str(a) + " * " +
                                   "W: "+str(b) + " = " + 
                                   "Area: "+str(response.area_rectangle))
        except Exception as e:
            self.get_logger().error("Service call failed %r" %(e,))

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoInts_Client()
    reponse = node.callback_add_two_ints_server(float(sys.argv[1]), float(sys.argv[2]))
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
