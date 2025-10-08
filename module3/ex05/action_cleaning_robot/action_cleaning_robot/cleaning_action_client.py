#!/usr/bin/env python3
import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from action_cleaning_robot.action import CleaningTask


class CleaningActionClient(Node):

    def __init__(self):
        super().__init__('cleaning_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'CleaningTask')
        self.goal_done = False

    def send_goal(self, task_type, size=0.0, x=0.0, y=0.0):
        self.goal_done = False

        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = size
        goal_msg.target_x = x
        goal_msg.target_y = y

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal request: {task_type}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        self.get_logger().info('Goal accepted by server, waiting for result...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info('=' * 30)
        self.get_logger().info(f'RESULT RECEIVED:')
        self.get_logger().info(f'  Success: {result.success}')
        self.get_logger().info(f'  Total distance: {result.total_distance:.2f} meters')
        self.get_logger().info(f'  Cleaned area: {result.cleaned_points} m^2')
        self.get_logger().info('=' * 30)

        self.goal_done = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: Progress: {feedback.progress_percent}% | '
            f'Cleaned so far: {feedback.current_cleaned_points} m^2 | '
            f'At: ({feedback.current_x:.2f}, {feedback.current_y:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    action_client = CleaningActionClient()

    try:
        action_client.send_goal(task_type='clean_square', size=3.0)

        while rclpy.ok() and not action_client.goal_done:
            rclpy.spin_once(action_client, timeout_sec=0.1)

        action_client.get_logger().info('Square cleaning finished. Returning home in 3 seconds...')
        time.sleep(3)

        if rclpy.ok():
            action_client.send_goal(task_type='return_home', x=5.5, y=5.5)
            while rclpy.ok() and not action_client.goal_done:
                rclpy.spin_once(action_client, timeout_sec=0.1)

        if rclpy.ok():
             action_client.get_logger().info('All tasks completed.')

    except KeyboardInterrupt:

        action_client.get_logger().info('Program interrupted by user (Ctrl+C). Shutting down.')

if __name__ == '__main__':
    main()