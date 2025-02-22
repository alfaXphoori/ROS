#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from ce_robot_interfaces.action import MoveRobot  # Ensure correct import


class MoveRobotClientNode(Node):
    def __init__(self):
        super().__init__("move_robot_client")
        self.move_robot_client_ = ActionClient(self, MoveRobot, "move_robot")
    
    def send_goal(self, distance, speed):
        self.move_robot_client_.wait_for_server()
        goal = MoveRobot.Goal()
        goal.distance = distance
        goal.speed = speed
        self.move_robot_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def cancel_goal(self):
        self.get_logger().info("Send a cancel goal request")
        self.goal_handle_.cancel_goal_async()
    
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected")
    
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.success))
    
    def goal_feedback_callback(self, feedback_msg):
        current_distance = feedback_msg.feedback.current_distance
        self.get_logger().info("Got feedback: " + str(current_distance))
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode()
    node.send_goal(5.0, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
