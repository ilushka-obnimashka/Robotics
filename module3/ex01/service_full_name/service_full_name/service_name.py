from full_name_interfaces.srv import SummFullName

import rclpy
from rclpy.node import Node


class ServiceName(Node):
    def __init__(self):
        super().__init__("service_name")
        self.srv = self.create_service(
            SummFullName,
            "full_name_service",
            self.concat_full_name_callback,
        )

    def concat_full_name_callback(self, request, response):
        if not request.first_name or not request.name or not request.last_name:
            response.full_name = "ERROR: Missing required name parts."
            self.get_logger().warn("Request received without required fields.")
            return response

        full_name = f"{request.first_name} {request.name} {request.last_name}".strip()
        response.full_name = full_name

        return response


def main(args=None):
    rclpy.init(args=args)

    service_name = ServiceName()

    try:
        rclpy.spin(service_name)
    except KeyboardInterrupt:
        service_name.get_logger().info("Node stopped by user (Ctrl+C)")
    finally:
        service_name.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
