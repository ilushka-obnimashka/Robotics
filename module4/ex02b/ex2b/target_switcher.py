from enum import EnumType
import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class TargetSwitcher(Node):
    def __init__(self):
        super().__init__("target_switcher")
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)

        static_target_str = (
            self.declare_parameter("static_target", ["8.0", "2.0"])
            .get_parameter_value()
            .string_array_value
        )

        self._static_target = [float(static_target_str[0]), float(static_target_str[1])]

        self.get_logger().info(
            f"Static_target set to: ({self._static_target[0], self._static_target[1]})"
        )
        # Публикует статическое преобразование для static_target только один раз
        self._public_static_transforms(self._static_target)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(0.1, self._broadcast_timer_callback)

        self._radius = (
            self.declare_parameter("radius", 1.0).get_parameter_value().double_value
        )

        self.get_logger().info(f"Radius set to: {self._radius}")

        direction = (
            self.declare_parameter("direction", 1.0).get_parameter_value().double_value
        )

        if direction not in [-1, 1]:
            self.get_logger().warn(
                f"Rotation must be -1 or 1",
                f"The default value is used",
                f"Rotation set to 1",
            )
            direction = 1.0

        self._direction = direction
        self.get_logger().info(f"Direction set to: {self._direction}")

        # используем списки, чтобы узел мог создавать любое_количество вращающихся целей.
        frames_id = (
            self.declare_parameter("frames_id", ["turtle1", "turtle3"])
            .get_parameter_value()
            .string_array_value
        )
        child_frames_id = (
            self.declare_parameter("child_frames_id", ["carrot1", "carrot2"])
            .get_parameter_value()
            .string_array_value
        )

        if len(frames_id) != len(child_frames_id):
            self.get_logger().warn(
                f"Number of frames_id and child_frames_id must be equal",
                f"The default value is used",
                f"Number of frames_id and child_frames_id set to 2",
            )
            frames_id = ["turtle1", "turtle3"]
            child_frames_id = ["carrot1", "carrot2"]

        self._frames_id = frames_id
        self._child_frames_id = child_frames_id

    def _public_static_transforms(self, target_pos: list[float]):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "static_target"

        t.transform.translation.x = float(target_pos[0])
        t.transform.translation.y = float(target_pos[1])
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._tf_static_broadcaster.sendTransform(t)

    def _broadcast_timer_callback(self):
        now_msg = self.get_clock().now().to_msg()

        seconds_float = now_msg.sec + now_msg.nanosec / 1_000_000_000.0
        angle = seconds_float * math.pi * self._direction

        transforms_list = []

        for frame_id, child_frame_id in zip(self._frames_id, self._child_frames_id):
            t = TransformStamped()

            t.header.stamp = now_msg
            t.header.frame_id = frame_id
            t.child_frame_id = child_frame_id

            t.transform.translation.x = self._radius * math.sin(angle)
            t.transform.translation.y = self._radius * math.cos(angle)
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0

            transforms_list.append(t)

        self._tf_broadcaster.sendTransform(transforms_list)


def main(args=None):
    rclpy.init(args=args)

    node = TargetSwitcher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
