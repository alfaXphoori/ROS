#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from ce_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MoveRobotServer(Node):
    def __init__(self):
        super().__init__("move_robot_server")
        self.move_robot_server_ = ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("MoveRobot Action Server has been started.")

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Received a goal")
        if goal_request.distance <= 0:
            self.get_logger().warn("Rejecting the goal, distance must be positive")
            return GoalResponse.REJECT
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        distance = goal_handle.request.distance
        speed = goal_handle.request.speed
        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()  # Ensure feedback message is created properly
        current_distance = 0.0

        self.get_logger().info("Executing the goal")

        while current_distance < distance:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling goal")
                goal_handle.canceled()
                result.success = False
                return result

            current_distance += speed
            self.get_logger().info(f"Current Distance: {current_distance}")

            # Assign value to the feedback message
            feedback.current_distance = float(current_distance)
            goal_handle.publish_feedback(feedback)

            time.sleep(1)

        goal_handle.succeed()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()