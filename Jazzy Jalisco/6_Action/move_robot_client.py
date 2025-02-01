#### **`move_robot_client.py` Implementation**
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ce_robot_actions.action import MoveRobot
import sys

class MoveRobotClient(Node):
    def __init__(self):
        super().__init__('move_robot_client')
        self._action_client = ActionClient(self, MoveRobot, 'move_robot')

    def send_goal(self, distance, speed):
        goal_msg = MoveRobot.Goal()
        goal_msg.distance = distance
        goal_msg.speed = speed
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: Current Distance = {feedback_msg.feedback.current_distance}m')
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action completed: Success = {result.success}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init()
    action_client = MoveRobotClient()
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    action_client.send_goal(distance, speed)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()