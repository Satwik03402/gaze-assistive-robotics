#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from summer_robotics_interfaces.srv import GetObjectById


class ObjectRegistryNode(Node):

    def __init__(self):
        super().__init__("object_registry_node")

        self.objects = {
            1: {
                "label": "red_cube",
                "pose": [0.9, -0.25, 0.48],
                "bbox": [320.0, 220.0, 80.0, 80.0],
                "pickable": True,
                "status": "ACTIVE"
            },
            2: {
                "label": "blue_cube",
                "pose": [0.9, 0.25, 0.48],
                "bbox": [460.0, 220.0, 80.0, 80.0],
                "pickable": True,
                "status": "ACTIVE"
            }
        }

        self.get_object_service = self.create_service(
            GetObjectById,
            "get_object_by_id",
            self.get_object_by_id_callback
        )

        self.get_logger().info("Object Registry Node started.")

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


def main(args=None):
    rclpy.init(args=args)

    node = ObjectRegistryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
