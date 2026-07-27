#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from summer_robotics_interfaces.msg import EyeGaze


class FakeGazeNode(Node):

    def __init__(self):
        super().__init__("fake_gaze_node")

        self.gaze_pub = self.create_publisher(
            EyeGaze,
            "/eye_gaze",
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_gaze
        )

        self.get_logger().info("Fake Gaze Node started.")

    def publish_gaze(self):

        msg = EyeGaze()

        msg.x = 0.9
        msg.y = -0.25
        msg.z = 0.48

        self.gaze_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakeGazeNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
