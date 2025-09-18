import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TextToCmdVel(Node):
    def __init__(
        self,
        topic_for_pub: str = "/turtle1/cmd_vel",
        topic_for_sub: str = "/cmd_text",
    ):
        super().__init__("TextToCmdVel")
        self.publisher_ = self.create_publisher(Twist, topic_for_pub, 10)
        self.subscription_ = self.create_subscription(
            String, topic_for_sub, self._cmd_msg_callback, 10
        )
        self.get_logger().info("Node initialized")


    def _cmd_msg_callback(self, msg : String):
        out_twist = Twist()
        command = msg.data.lower()

        match command:
            case "move_forward":
                out_twist.linear.x = 2.0
            case "move_backward":
                out_twist.linear.x = -2.0
            case "turn_left":
                out_twist.angular.z = 2.0
            case "turn_right":
                out_twist.angular.z = -2.0
            case _:
                out_twist.linear.x = 0.0
                out_twist.angular.z = 0.0

        self.publisher_.publish(out_twist)
        self.get_logger().info(
            f"Published: linear.x={out_twist.linear.x}, angular.z={out_twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)

    text_to_twist_pub = TextToCmdVel()

    try:
        rclpy.spin(text_to_twist_pub)
    except KeyboardInterrupt:
        text_to_twist_pub.get_logger().info("Node stopped by user (Ctrl+C)")
    finally:
        text_to_twist_pub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
