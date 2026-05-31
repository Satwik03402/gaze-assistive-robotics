import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__("robot_status_subscriber")

        self.subscription_ = self.create_subscription(
            String,
            "robot_status",
            self.status_callback,
            10
        )

        self.get_logger().info("Robot Status Subscriber has started.")

    def status_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    node = RobotStatusSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
