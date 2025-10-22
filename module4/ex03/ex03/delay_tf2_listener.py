# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may
#     obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):
    def __init__(self):
        super().__init__("turtle_tf2_frame_listener")

        # Declare and acquire `target_frame` parameter
        self._parent_frame = (
            self.declare_parameter("parent_frame", "turtle1")
            .get_parameter_value()
            .string_value
        )

        self._child_frame = (
            self.declare_parameter("child_frame", "turtle2")
            .get_parameter_value()
            .string_value
        )

        self._delay = (
            self.declare_parameter("delay", 5.0).get_parameter_value().double_value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._spawner = self.create_client(Spawn, "spawn")
        self._turtle_spawning_service_ready = False
        self._turtle_spawned = False

        self._publisher = self.create_publisher(
            Twist, f"{self._child_frame}/cmd_vel", 1
        )

        self.timer = self.create_timer(1.0, self._on_timer)

    def _on_timer(self):
        if self._turtle_spawning_service_ready:
            if self._turtle_spawned:
                try:
                    now = self.get_clock().now()
                    when = now - Duration(seconds=self._delay)

                    t = self._tf_buffer.lookup_transform_full(
                        target_frame=self._child_frame,
                        target_time=rclpy.time.Time().to_msg(),
                        source_frame=self._parent_frame,
                        source_time=when.to_msg(),
                        fixed_frame="world",
                        timeout=Duration(seconds=0.05),
                    )
                except (
                    LookupException,
                    ConnectivityException,
                    ExtrapolationException,
                ) as ex:
                    self.get_logger().info(
                        f"Could not transform {self._child_frame} to {self._parent_frame}: {ex}"
                    )
                    return

                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y, t.transform.translation.x
                )

                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x**2 + t.transform.translation.y**2
                )

                self._publisher.publish(msg)
            else:
                if self._result.done():
                    self.get_logger().info(
                        f"Successfully spawned {self._result.result().name}"
                    )
                    self._turtle_spawned = True
                else:
                    self.get_logger().info("Spawn is not finished")
        else:
            if self._spawner.service_is_ready():
                request = Spawn.Request()
                request.name = self._child_frame
                request.x = 4.0
                request.y = 2.0
                request.theta = 0.0
                self._result = self._spawner.call_async(request)
                self._turtle_spawning_service_ready = True
            else:
                self.get_logger().info("Service is not ready")


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
