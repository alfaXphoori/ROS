first_publisher.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class FirstPublishNode(Node):
    def __init__(self):
        super().__init__("first_publish_node")
        self.nodename_ = "Ce-Robot"
        self.counter_ = 0
        self.publishers_ = self.create_publisher(String, "ce_robot_pub",10)
        self.timer_ = self.create_timer(1, self.publishers_data)
        self.get_logger().info("Ce-Robot-Publisher-Node Start!.")

    def publishers_data(self):
        msg = String()
        msg.data = "Hi my "+ str(self.nodename_) + "Num : " + \
            str(self.counter_)
        self.counter_ +=1
        self.get_logger().info("Num : " + str(self.counter_))
        self.publishers_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FirstPublishNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
