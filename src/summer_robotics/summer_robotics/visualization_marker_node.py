#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class VisualizationMarkerNode(Node):

    def __init__(self):
        super().__init__("visualization_marker_node")

        self.marker_pub = self.create_publisher(
            Marker,
            "/visualization_marker",
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_markers
        )

        self.get_logger().info("Visualization Marker Node started.")

    def create_sphere_marker(self, marker_id, name, x, y, z, r, g, b):
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        return marker

    def publish_markers(self):
        red_object = self.create_sphere_marker(
            1,
            "red_cube_target",
            0.9,
            -0.25,
            0.48,
            1.0,
            0.0,
            0.0
        )

        blue_object = self.create_sphere_marker(
            2,
            "blue_cube_target",
            0.9,
            0.25,
            0.48,
            0.0,
            0.0,
            1.0
        )

        place_zone = self.create_sphere_marker(
            3,
            "place_zone",
            0.5,
            0.0,
            0.48,
            0.6,
            0.0,
            1.0
        )

        for marker in [
            red_object,
            blue_object,
            place_zone
        ]:
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = VisualizationMarkerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()