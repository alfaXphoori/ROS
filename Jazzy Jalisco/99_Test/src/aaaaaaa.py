#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from ce_robot_interfaces.action import RobotRun
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class RobotRun_Server(Node):
    def __init__(self):
        super().__init__("RobotRun_server")
        self.declare_parameter("topic_name","robot_run")
        self.topic_name_ = self.get_parameter("topic_name").get_parameter_value().string_value

        self.robot_run_server_ = ActionServer(
            self,
            RobotRun,
            self.topic_name_,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started.")

    def goal_callback(self, goal_request: RobotRun.Goal):
        self.get_logger().info("Received a goal")
        if goal_request.target_distance <= 0:
            self.get_logger().warn("Rejecting the goal, target number must be positive")
            return GoalResponse.REJECT
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_distance = goal_handle.request.target_distance
        speed = goal_handle.request.speed
        result = RobotRun.Result()
        feedback = RobotRun.Feedback()
        counter = 0

        self.get_logger().info("Executing the goal")
        for i in range (target_distance):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling goal")
                goal_handle.canceled()
                result.robot_time = counter
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_distance = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(speed)
        
        goal_handle.succeed()
        result.robot_time = float(target_distance*speed)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotRun_Server()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()