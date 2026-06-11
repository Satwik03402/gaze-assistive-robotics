#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from summer_robotics_interfaces.msg import DetectedObjectArray


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

        self.detected_objects = []

        self.object_sub = self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self.detected_objects_callback,
            10
        )

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

        for obj in self.detected_objects:

            if obj.label.lower() == "red_cube":
                r, g, b = 1.0, 0.0, 0.0

            elif obj.label.lower() == "blue_cube":
                r, g, b = 0.0, 0.0, 1.0

            else:
                r, g, b = 0.0, 1.0, 0.0

            marker = self.create_sphere_marker(
                obj.id,
                obj.label,
                obj.x,
                obj.y,
                obj.z,
                r,
                g,
                b
            )

            self.marker_pub.publish(marker)

        place_zone = self.create_sphere_marker(
            1000,
            "place_zone",
            0.5,
            0.0,
            0.48,
            0.6,
            0.0,
            1.0
        )

        self.marker_pub.publish(place_zone)

    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

def main(args=None):
    rclpy.init(args=args)

    node = VisualizationMarkerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()