#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from action_cleaning_robot.action import CleaningTask


class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        self._current_pose = None
        self.callback_group = ReentrantCallbackGroup()

        self._linear_k = 1.0
        self._angular_k = 1.0
        self._max_linear_vel = 1.5
        self._max_angular_vel = 1.0
        self._distance_tolerance = 0.05
        self._theta_tolerance = 0.05

        self._action_server = ActionServer(
            self, CleaningTask, 'CleaningTask',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10,
            callback_group=self.callback_group
        )
        self.velocity_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        self.get_logger().info("Cleaning Action Server has been started.")

    def pose_callback(self, msg):
        self._current_pose = msg

    def execute_callback(self, goal_handle):
        task_type = goal_handle.request.task_type.lower().strip()
        self.get_logger().info(f'Executing goal: {task_type}')

        while self._current_pose is None:
            self.get_logger().info('Waiting for initial pose...')
            time.sleep(1)

        result = CleaningTask.Result()

        if task_type == "clean_square":
            side_length = goal_handle.request.area_size
            total_distance, cleaned_area = self.fill_square(goal_handle, side_length)
        elif task_type == "return_home":
            target_x = goal_handle.request.target_x
            target_y = goal_handle.request.target_y
            total_distance, cleaned_area = self.return_home(goal_handle, target_x, target_y)
        else:
            self.get_logger().error(f"Unknown task type: {task_type}")
            goal_handle.abort()
            result.success = False
            return result

        if goal_handle.is_active:
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
            result.success = True

        result.total_distance = total_distance
        result.cleaned_points = int(cleaned_area)
        return result

    def _euclidean_distance(self, goal_pose):
        return math.sqrt(pow((goal_pose.x - self._current_pose.x), 2) +
                         pow((goal_pose.y - self._current_pose.y), 2))

    def _linear_vel(self, goal_pose):
        distance = self._euclidean_distance(goal_pose)
        linear_vel = self._linear_k * distance
        return min(linear_vel, self._max_linear_vel)

    def _steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self._current_pose.y, goal_pose.x - self._current_pose.x)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _angular_vel(self, goal_pose):
        angle_to_goal = self._steering_angle(goal_pose)
        angle_error = self._normalize_angle(angle_to_goal - self._current_pose.theta)
        angular_vel = self._angular_k * angle_error
        return max(min(angular_vel, self._max_angular_vel), -self._max_angular_vel)

    def return_home(self, goal_handle, target_x, target_y):
        goal_pose = Pose()
        goal_pose.x = target_x
        goal_pose.y = target_y
        goal_pose.theta = 0.0
        initial_distance = self._euclidean_distance(goal_pose)
        if initial_distance < 0.01: initial_distance = 1.0
        rate = self.create_rate(50)
        vel_msg = Twist()
        is_aligning = False

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.velocity_publisher.publish(Twist())
                return initial_distance - self._euclidean_distance(goal_pose), 0.0

            distance = self._euclidean_distance(goal_pose)

            if distance > self._distance_tolerance and not is_aligning:
                vel_msg.linear.x = self._linear_vel(goal_pose)
                vel_msg.angular.z = self._angular_vel(goal_pose)
            else:
                is_aligning = True
                vel_msg.linear.x = 0.0
                error_theta = self._normalize_angle(goal_pose.theta - self._current_pose.theta)
                if abs(error_theta) > self._theta_tolerance:
                    angular_vel_align = self._angular_k * error_theta
                    vel_msg.angular.z = max(min(angular_vel_align, self._max_angular_vel), -self._max_angular_vel)
                else:
                    self.velocity_publisher.publish(Twist())
                    break

            self.velocity_publisher.publish(vel_msg)
            distance_traveled = initial_distance - distance
            feedback_msg = CleaningTask.Feedback()
            progress = int((distance_traveled / initial_distance) * 100) if not is_aligning else 99
            feedback_msg.progress_percent = min(progress, 100)
            feedback_msg.current_x = self._current_pose.x
            feedback_msg.current_y = self._current_pose.y
            feedback_msg.current_cleaned_points = 0
            goal_handle.publish_feedback(feedback_msg)
            rate.sleep()
        return initial_distance, 0.0

    def get_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def _move_and_wait(self, goal_handle, total_estimated_distance, distance_covered, total_area,
                       target_distance=0.0, target_angle=0.0, is_turn=False):

        start_pose = self._current_pose
        rate = self.create_rate(50)
        target_theta = self._normalize_angle(start_pose.theta + target_angle)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                break

            cmd = Twist()

            if is_turn:
                error_theta = self._normalize_angle(target_theta - self._current_pose.theta)
                if abs(error_theta) < self._theta_tolerance:
                    break

                angular_speed = self._angular_k * error_theta
                cmd.angular.z = max(min(angular_speed, self._max_angular_vel), -self._max_angular_vel)

            else:
                distance_traveled = self.get_distance(start_pose, self._current_pose)
                remaining_distance = target_distance - distance_traveled

                if remaining_distance < self._distance_tolerance:
                    break

                linear_speed = self._linear_k * remaining_distance
                cmd.linear.x = min(linear_speed, self._max_linear_vel)

                current_total_distance = distance_covered + distance_traveled
                feedback_msg = CleaningTask.Feedback()

                if total_estimated_distance > 0:
                    progress = int((current_total_distance / total_estimated_distance) * 100)
                    feedback_msg.progress_percent = min(progress, 100)

                    current_area = (current_total_distance / total_estimated_distance) * total_area
                    feedback_msg.current_cleaned_points = int(current_area)

                feedback_msg.current_x = self._current_pose.x
                feedback_msg.current_y = self._current_pose.y
                goal_handle.publish_feedback(feedback_msg)

            self.velocity_publisher.publish(cmd)
            rate.sleep()

        self.velocity_publisher.publish(Twist())
        return self.get_distance(start_pose, self._current_pose) if not is_turn else 0.0

    def fill_square(self, goal_handle, side):
        brush_width = 0.4
        num_passes = math.ceil(side / brush_width)
        total_estimated_distance = num_passes * side + (num_passes - 1) * brush_width
        if total_estimated_distance == 0: total_estimated_distance = 1
        distance_covered = 0.0

        total_area = side * side

        for i in range(num_passes):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled!")
                current_area = (distance_covered / total_estimated_distance) * total_area
                return distance_covered, current_area

            distance_covered += self._move_and_wait(goal_handle, total_estimated_distance, distance_covered, total_area,
                                                    target_distance=side)
            if i < num_passes - 1:
                turn_direction = 1 if i % 2 == 0 else -1
                self._move_and_wait(goal_handle, total_estimated_distance, distance_covered, total_area, is_turn=True,
                                    target_angle=turn_direction * math.pi / 2)
                distance_covered += self._move_and_wait(goal_handle, total_estimated_distance, distance_covered,
                                                        total_area,
                                                        target_distance=brush_width)
                self._move_and_wait(goal_handle, total_estimated_distance, distance_covered, total_area, is_turn=True,
                                    target_angle=turn_direction * math.pi / 2)

        feedback_msg = CleaningTask.Feedback()
        feedback_msg.progress_percent = 100
        feedback_msg.current_cleaned_points = int(total_area)
        feedback_msg.current_x = self._current_pose.x
        feedback_msg.current_y = self._current_pose.y
        goal_handle.publish_feedback(feedback_msg)

        return distance_covered, total_area


def main(args=None):
    rclpy.init(args=args)
    action_server = CleaningActionServer()
    executor = MultiThreadedExecutor()

    try:
        action_server.get_logger().info("Action server is running. Press Ctrl+C to shut down.")
        rclpy.spin(action_server, executor=executor)

    except KeyboardInterrupt:
        action_server.get_logger().info("Shutdown requested by user (Ctrl+C).")

    finally:
        action_server.get_logger().info("Destroying node and shutting down ROS client library.")
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()