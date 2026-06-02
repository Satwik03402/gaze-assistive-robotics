#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from summer_robotics_interfaces.srv import GetObjectById
from summer_robotics_interfaces.msg import DetectedObjectArray

class ObjectRegistryNode(Node):

    def __init__(self):
        super().__init__("object_registry_node")

        self.objects = {}

        self.get_object_service = self.create_service(
            GetObjectById,
            "get_object_by_id",
            self.get_object_by_id_callback
        )

        self.detected_objects_sub = self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self.detected_objects_callback,
            10
        )

        self.get_logger().info("Object Registry Node started.")
        self.object_timeout_sec = 3.0
        self.registry_timer = self.create_timer(0.5, self.update_object_statuses)

    def get_object_by_id_callback(self, request, response):
        object_id = request.object_id

        if object_id not in self.objects:
            response.success = False
            response.label = ""
            response.pose = [0.0, 0.0, 0.0]
            response.bbox = [0.0, 0.0, 0.0, 0.0]
            response.pickable = False
            response.status = "NOT_FOUND"
            return response

        obj = self.objects[object_id]

        response.success = True
        response.label = obj["label"]
        response.pose = obj["pose"]
        response.bbox = obj["bbox"]
        response.pickable = obj["pickable"]
        response.status = obj["status"]

        self.get_logger().info(f"Lookup requested for object ID {object_id}: {response.label}")

        return response

    def detected_objects_callback(self, msg):
        now = self.get_clock().now()

        for detected_object in msg.objects:
            object_id = detected_object.id

            self.objects[object_id] = {
                "label": detected_object.label,
                "pose": [
                    detected_object.x,
                    detected_object.y,
                    detected_object.z
                ],
                "bbox": list(detected_object.bbox),
                "pickable": detected_object.pickable,
                "status": detected_object.status,
                "last_seen_time": now
            }

            self.get_logger().info(
                f"Updated object ID {object_id}: {detected_object.label}"
            )

    def update_object_statuses(self):
        now = self.get_clock().now()

        for object_id, obj in self.objects.items():
            if "last_seen_time" not in obj:
                continue

            elapsed_time = (
                now - obj["last_seen_time"]
            ).nanoseconds / 1e9

            if elapsed_time > self.object_timeout_sec:
                obj["status"] = "TEMP_LOST"

def main(args=None):
    rclpy.init(args=args)

    node = ObjectRegistryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
