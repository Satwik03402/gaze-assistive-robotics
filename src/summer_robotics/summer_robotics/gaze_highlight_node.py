#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from visualization_msgs.msg import Marker

from summer_robotics_interfaces.msg import DetectedObjectArray


class GazeHighlightNode(Node):

    def __init__(self):
        super().__init__("gaze_highlight_node")

        self.candidate_object_id = -1
        self.detected_objects = []

        self.candidate_sub = self.create_subscription(
            Int32,
            "/gaze_candidate_object_id",
            self.candidate_callback,
            10
        )

        self.detected_objects_sub = self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self.detected_objects_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/gaze_highlight_marker",
            10
        )

        self.get_logger().info("Gaze Highlight Node started.")

    def candidate_callback(self, msg):
        self.candidate_object_id = msg.data

        self.publish_highlight()

    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

        self.publish_highlight()

    def publish_highlight(self):
        if self.candidate_object_id < 0:
            self.clear_highlight()
            return

        selected_object = None

        for obj in self.detected_objects:
            if obj.id == self.candidate_object_id:
                selected_object = obj
                break

        if selected_object is None:
            self.clear_highlight()
            return

        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "gaze_highlight"
        marker.id = 999
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = selected_object.x
        marker.pose.position.y = selected_object.y
        marker.pose.position.z = selected_object.z + 0.08

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.015

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7

        self.marker_pub.publish(marker)

    def clear_highlight(self):
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "gaze_highlight"
        marker.id = 999
        marker.action = Marker.DELETE

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = GazeHighlightNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()