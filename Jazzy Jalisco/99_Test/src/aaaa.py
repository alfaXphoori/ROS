#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import CalCircle

class Cal_Circle(Node):
    def __init__(self):
        super().__init__("calcircle_server")
        self.server_ = self.create_service(
            CalCircle, "cal_circle", self.callback_cal_rectangle)
        self.get_logger().info("Cal circle Server Start!")
        
    def callback_cal_rectangle(self, request, response):
        response.area = (request.r * request.r) * 3.14159265359
        self.get_logger().info("r:"+str(request.r) + " * " + 
                               "pi"+
                               "area:"+str(response.area))
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = Cal_Circle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main() 