#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import HardwareStatus

class HwStatusNode(Node):
    def __init__(self):
        super().__init__("phoori_publish")
        self.hw_status_publish_= self.create_publisher(HardwareStatus,"phoori_std", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Phoori Node Start Now!")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.std_name = "Phoori"
        msg.std_surname = "Chantima"
        msg.std_class = "CE6721"
        msg.std_number = 99
        self.hw_status_publish_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HwStatusNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()