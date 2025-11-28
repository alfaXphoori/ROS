#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.srv import CalRectangle

class Cal_Rectangle(Node):
    def __init__(self):
        super().__init__("calrect_server")
        self.server_ = self.create_service(
            CalRectangle, "cal_rect", self.callback_cal_rectangle)
        self.get_logger().info("Cal Rectangle Server Start!")
        
    def callback_cal_rectangle(self, request, response):
        response.area_rectangle = request.length * request.width
        self.get_logger().info("length:"+str(request.length) + " * " + 
                               "width:"+str(request.width) + " = " +
                               "area:"+str(response.area_rectangle))
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = Cal_Rectangle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
