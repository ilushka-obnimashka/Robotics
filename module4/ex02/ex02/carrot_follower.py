import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn


class CarrotFollower(Node):
    def __init__(self):
        super().__init__("carrot_follower")

        self.target_frame = (
            self.declare_parameter("target_frame", "carrot")
            .get_parameter_value()
            .string_value
        )

        self.follower_frame = (
            self.declare_parameter("follower_frame", "turtle2")
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spawner = self.create_client(Spawn, "spawn")

        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False

        self.publisher = self.create_publisher(Twist, "cmd_vel", 1)

        self.timer = self.create_timer(0.1, self.__on_timer)

    def __on_timer(self):
        # 'from_frame_rel' - ИМЯ ФРЕЙМА, ИЗ КОТОРОГО мы смотрим (наша цель).
        # Мы берем его из параметра, который задали в '__init__'.
        # (По умолчанию 'turtle1', но для твоего задания ты поменяешь на 'carrot')
        from_frame_rel = self.target_frame
        # 'to_frame_rel' - ИМЯ ФРЕЙМА, В КОТОРЫЙ мы смотрим (наше положение).
        # Это фрейм нашей второй черепахи.
        to_frame_rel = self.follower_frame

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                try:
                    # 'self.tf_buffer.lookup_transform' - ГЛАВНАЯ ФУНКЦИЯ 'tf2'.
                    # Мы просим буфер рассчитать преобразование...
                    # 'to_frame_rel' ('turtle2') - ...В ЭТОТ фрейм (целевая система координат)
                    # 'from_frame_rel' ('carrot') - ...ИЗ ЭТОГО фрейма (то, что мы ищем)
                    # 'rclpy.time.Time()' - ...на "сейчас" (самое последнее доступное время).
                    #
                    # Перевод: "Эй, tf_buffer, скажи, где находится 'carrot'
                    #         (from_frame_rel) с точки зрения 'turtle2' (to_frame_rel)?"
                    #
                    # 't' - это будет объект TransformStamped, содержащий
                    # 't.transform.translation.x' (расстояние по X до цели)
                    # 't.transform.translation.y' (расстояние по Y до цели)
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel, from_frame_rel, rclpy.time.Time()
                    )
                except TransformException as ex:
                    self.get_logger().info(
                        f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
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

                self.publisher.publish(msg)

            else:
                if self.result.done():
                    self.get_logger().info(
                        f"Successfully spawned {self.result.result().name}"
                    )
                    self.turtle_spawned = True
                else:
                    self.get_logger().info("Spawn is not finished")

        else:
            if self.spawner.service_is_ready():
                request = Spawn.Request()
                request.name = self.follower_frame

                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)

                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                self.get_logger().info("Service is not ready")


def main():
    rclpy.init()
    node = CarrotFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
