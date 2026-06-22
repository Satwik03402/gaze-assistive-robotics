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

    def marker_callback(self, msg):
        if msg.ns not in ["attached_object", "placed_object"]:
            return

        self.get_logger().info(
            f"Received object marker: ns={msg.ns}, "
            f"frame={msg.header.frame_id}, "
            f"pos=({msg.pose.position.x:.2f}, "
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
