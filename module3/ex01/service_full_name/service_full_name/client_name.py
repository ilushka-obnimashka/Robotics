import sys

from rclpy.task import Future
from full_name_interfaces.srv import SummFullName

import rclpy
from rclpy.node import Node


class ClientNameAsync(Node):
    def __init__(self):
        super().__init__("minimal_client_async")
        self.cli = self.create_client(SummFullName, "full_name_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SummFullName.Request()

    def send_request(self, first_name: str, name: str, last_name: str):
        self.req.first_name = first_name
        self.req.name = name
        self.req.last_name = last_name
        return self.cli.call_async(self.req)


def main(args=None):
    if len(sys.argv) != 4:
        print(
            "Usage: ros2 run service_full_name client <first_name> <name> <last_name>"
        )
        sys.exit(1)

    rclpy.init(args=args)

    client_name = ClientNameAsync()

    try:
        future = client_name.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
        rclpy.spin_until_future_complete(client_name, future)
        response = future.result()
        client_name.get_logger().info(f"Full name: {response.full_name}")
    except KeyboardInterrupt:
        client_name.get_logger().info("Node stopped by user (Ctrl+C)")
    finally:
        client_name.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
