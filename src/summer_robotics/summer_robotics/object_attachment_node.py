#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from summer_robotics_interfaces.msg import RobotStatus
from summer_robotics.robot_commands import PICK, PLACE


class ObjectAttachmentNode(Node):

    def __init__(self):
        super().__init__("object_attachment_node")

        self.attached = False
        self.attached_object_id = 0

        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            "/robot_status",
            self.robot_status_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/visualization_marker",
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_attached_marker
        )

        self.get_logger().info("Object Attachment Node started.")

    def robot_status_callback(self, msg):
        if msg.status != "DONE" or not msg.success:
            return

        if msg.current_command == PICK:
            self.attached = True
            self.attached_object_id = msg.object_id

            self.get_logger().info(
                f"Attached object ID {self.attached_object_id}"
            )

        elif msg.current_command == PLACE:
            self.attached = False

            self.publish_placed_marker(msg.object_id)

            self.get_logger().info(
                f"Detached object ID {msg.object_id}"
            )

    def create_carried_marker(self):
        marker = Marker()

        marker.header.frame_id = "end_effector_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "attached_object"
        marker.id = 5000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def create_placed_marker(self):
        marker = Marker()

        marker.header.frame_id = "end_effector_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "placed_object"
        marker.id = 5001
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def publish_attached_marker(self):
        if not self.attached:
            return

        marker = self.create_carried_marker()
        self.marker_pub.publish(marker)

    def publish_placed_marker(self, object_id):
        marker = self.create_placed_marker()
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = ObjectAttachmentNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()