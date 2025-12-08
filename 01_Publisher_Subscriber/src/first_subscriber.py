#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class FirstSubscribeNode(Node):
    def __init__(self):
        super().__init__("first_subscribe_node")
        self.subscriber_ = self.create_subscription(
            String, "ce_robot_pub", self.callback_robot_data, 10)
        self.get_logger().info("My-First-Subscribe-Node Start!.")

    def callback_robot_data(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = FirstSubscribeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
