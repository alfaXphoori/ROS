#### **`move_robot_server.py` Implementation**

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ce_robot_actions.action import MoveRobot
import time

class MoveRobotServer(Node):
    def __init__(self):
        super().__init__('move_robot_server')
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_callback)
        self.get_logger().info('MoveRobot Action Server Started')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: Move {goal_handle.request.distance}m at {goal_handle.request.speed}m/s')
        feedback_msg = MoveRobot.Feedback()
        current_distance = 0.0

        while current_distance < goal_handle.request.distance:
            time.sleep(1)
            current_distance += goal_handle.request.speed
            feedback_msg.current_distance = current_distance
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Current Distance: {current_distance}m')

        goal_handle.succeed()
        result = MoveRobot.Result()
        result.success = True
        return result


def main():
    rclpy.init()
    move_robot_server = MoveRobotServer()
    rclpy.spin(move_robot_server)
    move_robot_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()