#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker


class GazeboObjectControllerNode(Node):

    def __init__(self):
        super().__init__("gazebo_object_controller_node")

        self.attached_pose_sub = self.create_subscription(
            Marker,
            "/visualization_marker",
            self.marker_callback,
            10
        )

        self.get_logger().info("Gazebo Object Controller Node started.")
        self.get_logger().info("Listening for attached/placed object markers.")
        self.current_attached_pose = None
        self.current_placed_pose = None
        self.is_attached = False

    def marker_callback(self, msg):

        if msg.ns == "attached_object":

            self.current_attached_pose = msg.pose
            self.is_attached = True

            self.get_logger().info(
                f"Attached object updated: "
                f"({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}, "
                f"{msg.pose.position.z:.2f})"
            )

        elif msg.ns == "placed_object":

            self.current_placed_pose = msg.pose
            self.is_attached = False

            self.get_logger().info(
                f"Placed object updated: "
                f"({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}, "
                f"{msg.pose.position.z:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)

    node = GazeboObjectControllerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
