#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus

class HwStatusNode_Para(Node):
    def __init__(self):
        super().__init__("hw_para_publish")
        self.declare_parameter("rb_name","name")
        self.declare_parameter("rb_no",9999)
        
        self.robot_name_ = self.get_parameter("rb_name").value
        self.robot_number_ = self.get_parameter("rb_no").value
        
        self.hw_status_publish_= self.create_publisher(HardwareStatus,"hardware_status_para", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Hw_Status_Publish_Node Start Now!")

    def publish_hw_status(self):
        msg = HardwareStatus()

        msg.name_robot = self.robot_name_
        msg.number_robot = self.robot_number_

        self.hw_status_publish_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HwStatusNode_Para()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
