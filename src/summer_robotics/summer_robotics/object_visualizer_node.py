#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray

from summer_robotics_interfaces.srv import GetAvailableObjects


class ObjectVisualizerNode(Node):

    def __init__(self):
        super().__init__("object_visualizer_node")

        self.available_objects_client = self.create_client(
            GetAvailableObjects,
            "/get_available_objects"
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/object_markers",
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_markers
        )

        self.get_logger().info("Object Visualizer Node started.")

    def get_status_color(self, status):
        if status == "ACTIVE":
            return 0.0, 1.0, 0.0
        if status == "TEMP_LOST":
            return 1.0, 1.0, 0.0
        if status == "LOST":
            return 1.0, 0.0, 0.0

        return 1.0, 1.0, 1.0

    def publish_markers(self):
        if not self.available_objects_client.service_is_ready():
            self.get_logger().warn("GetAvailableObjects service not ready.")
            return

        request = GetAvailableObjects.Request()
        future = self.available_objects_client.call_async(request)

        future.add_done_callback(self.handle_available_objects_response)

    def handle_available_objects_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f"Failed to get available objects: {error}")
            return

        marker_array = MarkerArray()

        for index, (object_id, label, status) in enumerate(
            zip(response.ids, response.labels, response.statuses)
        ):
            x = 0.6 + 0.3 * index
            y = 0.0
            z = 0.5

            r, g, b = self.get_status_color(status)

            cube_marker = Marker()
            cube_marker.header.frame_id = "world"
            cube_marker.header.stamp = self.get_clock().now().to_msg()
            cube_marker.ns = "objects"
            cube_marker.id = object_id * 10
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.pose.position.x = x
            cube_marker.pose.position.y = y
            cube_marker.pose.position.z = z
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = 0.12
            cube_marker.scale.y = 0.12
            cube_marker.scale.z = 0.12
            cube_marker.color.r = r
            cube_marker.color.g = g
            cube_marker.color.b = b
            cube_marker.color.a = 0.9

            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "object_labels"
            text_marker.id = object_id * 10 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 0.18
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.08
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"ID {object_id}\n{label}\n{status}"

            marker_array.markers.append(cube_marker)
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = ObjectVisualizerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
