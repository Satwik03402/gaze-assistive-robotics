#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from summer_robotics_interfaces.msg import RobotStatus
from summer_robotics.robot_commands import PICK, PLACE
from summer_robotics_interfaces.srv import GetObjectById


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
        self.place_zone_id = 3
        self.place_zone_pose = None

        self.object_lookup_client = self.create_client(
            GetObjectById,
            "/get_object_by_id"
        )

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

    def create_placed_marker(self, x, y, z):
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "placed_object"
        marker.id = 5001
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
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
        if not self.object_lookup_client.service_is_ready():
            self.get_logger().warn("Object registry service not ready.")
            return

        request = GetObjectById.Request()
        request.object_id = self.place_zone_id

        future = self.object_lookup_client.call_async(request)
        future.add_done_callback(self.place_zone_lookup_done)

    def place_zone_lookup_done(self, future):
        response = future.result()

        if not response.success:
            self.get_logger().warn("Place zone lookup failed for placed marker.")
            return

        marker = self.create_placed_marker(
            response.pose[0],
            response.pose[1],
            response.pose[2] + 0.07
        )

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"Placed marker published at place zone pose: {list(response.pose)}"
        )

def main(args=None):
    rclpy.init(args=args)

    node = ObjectAttachmentNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()