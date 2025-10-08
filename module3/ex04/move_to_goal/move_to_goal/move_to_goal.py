import sys
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Move2Goal(Node):

    def __init__(
            self,
            topic_for_pub: str = "/turtle1/cmd_vel",
            topic_for_sub: str = "/turtle1/pose",
            goal_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
            distance_tolerance: float = 0.1,
            theta_tolerance: float = 0.1,
    ):
        super().__init__("Move2Goal")
        self._publisher = self.create_publisher(Twist, topic_for_pub, 10)

        self._subscription = self.create_subscription(
            Pose, topic_for_sub, self._control_loop_callback, 10
        )

        self._pose = Pose()
        self._goal = Pose()
        self._goal.x = goal_pose[0]
        self._goal.y = goal_pose[1]
        self._goal.theta = goal_pose[2]

        self._distance_tolerance = distance_tolerance
        self._theta_tolerance = theta_tolerance

        self._is_goal_reached = False
        self._is_aligning = False

        self._linear_k = 0.5
        self._angular_k = 2.0
        self._max_linear_vel = 2.0
        self._max_angular_vel = 1.8

        self.get_logger().info(
            f"Node initialized. Target: ({self._goal.x}, {self._goal.y}, {self._goal.theta})"
        )
        self.get_logger().info(
            f"Control K: Lin={self._linear_k}, Ang={self._angular_k}. Max Vel: Lin={self._max_linear_vel}, Ang={self._max_angular_vel}"
        )

    @property
    def is_goal_reached(self):
        return self._is_goal_reached

    def _euclidean_distance(self, goal_pose: Pose):
        return math.sqrt(pow((goal_pose.x - self._pose.x), 2) +
                         pow((goal_pose.y - self._pose.y), 2))

    def _linear_vel(self, goal_pose):
        distance = self._euclidean_distance(goal_pose)
        linear_vel = self._linear_k * distance
        return min(linear_vel, self._max_linear_vel)

    def _steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self._pose.y, goal_pose.x - self._pose.x)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _angular_vel(self, goal_pose):
        angle_to_goal = self._steering_angle(goal_pose)
        angle_error = self._normalize_angle(angle_to_goal - self._pose.theta)
        angular_vel = self._angular_k * angle_error
        return max(min(angular_vel, self._max_angular_vel), -self._max_angular_vel)

    def _stop(self, vel_msg: Twist):
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0


    def _control_loop_callback(self, data):
        self._pose = data
        self._pose.x = round(self._pose.x, 4)
        self._pose.y = round(self._pose.y, 4)
        self._pose.theta = round(self._pose.theta, 4)

        if self._is_goal_reached:
            return

        vel_msg = Twist()
        distance = self._euclidean_distance(self._goal)

        if distance < self._distance_tolerance or self._is_aligning:
            self._is_aligning = True
            self._rotate_to_final_orientation(vel_msg)

        else:
            self._control_p_controller(vel_msg)

        self._publisher.publish(vel_msg)
        self.get_logger().info(
            f"Published: Lin={vel_msg.linear.x:.2f}, Ang={vel_msg.angular.z:.2f}. Dist={distance:.3f}")

    def _control_p_controller(self, vel_msg: Twist):
        vel_msg.linear.x = self._linear_vel(self._goal)
        vel_msg.angular.z = self._angular_vel(self._goal)


    def _rotate_to_final_orientation(self, vel_msg: Twist):
        vel_msg.linear.x = 0.0
        error_theta = self._normalize_angle(self._goal.theta - self._pose.theta)

        if abs(error_theta) > self._theta_tolerance:
            angular_vel = self._angular_k * error_theta
            vel_msg.angular.z = max(min(angular_vel, self._max_angular_vel), -self._max_angular_vel)

        else:
            self._stop(vel_msg)
            self._is_goal_reached = True
            self.get_logger().info("Goal (X, Y, Theta) has been fully reached! Stopping node.")


def main(args=None):
    if len(sys.argv) < 4:
        print(
            "Usage: ros2 run <package_name> <node_name> <x> <y> <theta>")
        print("Example: ros2 run my_pkg move_to_goal 8.0 2.0 1.2")
        sys.exit(1)

    rclpy.init(args=args)

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_theta = float(sys.argv[3])

    mover = Move2Goal(
        goal_pose=(goal_x, goal_y, goal_theta),
    )

    try:
        rclpy.spin(mover)
    except KeyboardInterrupt:
        mover._stop(Twist())
        mover._publisher.publish(Twist())
        mover.get_logger().info("Node stopped by user (Ctrl+C)")
    finally:
        print(mover.is_goal_reached)
        mover.destroy_node()
        sys.exit()


if __name__ == "__main__":
    main()