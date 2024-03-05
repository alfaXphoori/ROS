#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ce_robot_interface.msg import HardwareStatus

class HwStatusNode(Node):
    def __init__(self):
        super().__init__("hardwarestatus_publish")
        self.robot_name_ = "CE-RO"
        self.robot_number_ = 9981
        self.hw_status_publish_= self.create_publisher(HardwareStatus,"hardware_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Hw_Status_Publish_Node Start Now!")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.name_robot = self.robot_name_
        msg.number_robot = self.robot_number_
        msg.temperature = 50
        msg.debug_message = "Motor 1"
        self.hw_status_publish_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HwStatusNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
