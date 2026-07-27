#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from summer_robotics_interfaces.msg import DetectedObject, DetectedObjectArray


class FakePerceptionNode(Node):

    def __init__(self):
        super().__init__("fake_perception_node")

        self.publisher_ = self.create_publisher(
            DetectedObjectArray,
            "/detected_objects",
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_fake_detections
        )

        self.get_logger().info("Fake Perception Node started.")

    def create_object(self, object_id, label, x, y, z, bbox):
        obj = DetectedObject()

        obj.id = object_id
        obj.label = label

        obj.x = x
        obj.y = y
        obj.z = z

        obj.bbox = bbox

        obj.pickable = True
        obj.status = "ACTIVE"

        return obj

    def publish_fake_detections(self):
        msg = DetectedObjectArray()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        red_cube = self.create_object(
            1,
            "red_cube",
            0.9,
            -0.25,
            0.48,
            [320.0, 220.0, 80.0, 80.0]
        )

        blue_cube = self.create_object(
            2,
            "blue_cube",
            0.9,
            0.25,
            0.48,
            [460.0, 220.0, 80.0, 80.0]
        )

        green_cube = self.create_object(
            3,
            "green_cube",
            0.8,
            0.0,
            0.48,
            [590.0, 220.0, 80.0, 80.0]
        )

        msg.objects = [
            red_cube,
            blue_cube,
            green_cube
        ]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakePerceptionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()