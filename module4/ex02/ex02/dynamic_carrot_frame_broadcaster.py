# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class DynamicCarrotFrameBroadcaster(Node):
    def __init__(self):
        super().__init__("dynamic_carrot_frame_broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.radius = (
            self.declare_parameter("radius", 1.0).get_parameter_value().double_value
        )

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

        self.direction = direction

        self.frame_id = (
            self.declare_parameter("frame_id", "turtle1")
            .get_parameter_value()
            .string_value
        )
        self.child_frame_id = (
            self.declare_parameter("child_frame_id", "carrot1")
            .get_parameter_value()
            .string_value
        )

    def broadcast_timer_callback(self):
        now_msg = self.get_clock().now().to_msg()

        seconds_float = now_msg.sec + now_msg.nanosec / 1_000_000_000.0

        angle = seconds_float * math.pi * self.direction

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.radius * math.sin(angle)
        t.transform.translation.y = self.radius * math.cos(angle)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DynamicCarrotFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
