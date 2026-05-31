#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__("robot_status_publisher")

        self.publisher_ = self.create_publisher(
            String,
            "robot_status",
            10
        )

        self.timer_ = self.create_timer(
            1.0,
            self.publish_status
        )

        self.counter = 0
        self.get_logger().info("Robot Status Publisher has started.")

    def publish_status(self):
        msg = String()
        msg.data = f"Robot is alive. Status count: {self.counter}"

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        self.counter += 1

    def selected_object_callback(self, msg):
        pass

    def start_task_callback(self, msg):
        pass

    def joint_state_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    node = RobotStatusPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
