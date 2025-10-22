import rclpy
import math
import sys
import select
import termios
import tty
import threading

from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from ex2b_interface.msg import TargetInfo


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self._turtle_name = (
            self.declare_parameter("turtle_name", "turtle2")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f"Управление черепахой: {self._turtle_name}")

        self._switch_threshold = (
            self.declare_parameter("switch_threshold", 1.0)
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(f"Switch threshold set to: {self._switch_threshold}")

        self._targets = (
            self.declare_parameter("targets", ["carrot1", "carrot2", "static_target"])
            .get_parameter_value()
            .string_array_value
        )
        self.get_logger().info(f"Targets set to: {self._targets}")
        self._target_index = 0
        self._current_target_frame = self._targets[self._target_index]

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_vel_pub = self.create_publisher(
            Twist, f"/{self._turtle_name}/cmd_vel", 10
        )
        self._target_info_pub = self.create_publisher(TargetInfo, "/current_target", 10)

        self._timer = self.create_timer(0.1, self.on_control_loop)

        self.get_logger().info(
            "Press 'n' for manual target switching, 'Ctrl+C' to exit."
        )
        self._key_thread = threading.Thread(target=self._keyboard_loop)
        self._key_thread.daemon = True
        self._key_thread.start()

    def _keyboard_loop(self):
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == "n":
                        self.switch_target(manual=True)
            except Exception as e:
                self.get_logger().error(f"Error in keyboard thread: {e}")
                break

    def switch_target(self, manual=False):
        self._target_index = (self._target_index + 1) % len(self._targets)
        self._current_target_frame = self._targets[self._target_index]

        log_msg = f"Switch to: {self._current_target_frame}"
        if manual:
            log_msg = "[Manual] " + log_msg
        else:
            log_msg = "[Auto] " + log_msg
        self.get_logger().info(log_msg)

    def on_control_loop(self):
        try:
            trans_to_target = self._tf_buffer.lookup_transform(
                self._turtle_name, self._current_target_frame, rclpy.time.Time()
            )

            trans_world_to_target = self._tf_buffer.lookup_transform(
                "world", self._current_target_frame, rclpy.time.Time()
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"Transform не найден: {e}", throttle_duration_sec=1.0
            )
            return

        distance = math.sqrt(
            trans_to_target.transform.translation.x**2
            + trans_to_target.transform.translation.y**2
        )

        info_msg = TargetInfo()
        info_msg.target_name = self._current_target_frame
        info_msg.target_x = trans_world_to_target.transform.translation.x
        info_msg.target_y = trans_world_to_target.transform.translation.y
        info_msg.distance_to_target = distance
        self._target_info_pub.publish(info_msg)

        if distance < self._switch_threshold:
            self.switch_target(manual=False)
            self._cmd_vel_pub.publish(Twist())
            return

        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5 * distance
        cmd_msg.angular.z = 1.0 * math.atan2(
            trans_to_target.transform.translation.y,
            trans_to_target.transform.translation.x,
        )
        self._cmd_vel_pub.publish(cmd_msg)


def main(args=None):
    # --- ЭТО ОБЯЗАТЕЛЬНО ---
    # Сохраняем исходные настройки терминала
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # --- И ЭТО ОБЯЗATЕЛЬНО ---
        # Восстанавливаем терминал при выходе, иначе он "сломается"
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
