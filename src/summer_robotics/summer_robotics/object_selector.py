#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from summer_robotics_interfaces.srv import GetAvailableObjects


class ObjectSelector(Node):

    def __init__(self):
        super().__init__("object_selector")

        self.publisher_ = self.create_publisher(
            Int32,
            "/selected_object_id",
            10
        )

        self.available_objects_client = self.create_client(
            GetAvailableObjects,
            "/get_available_objects"
        )

        self.get_logger().info("Object Selector Started.")

        self.input_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )
        self.input_thread.start()

    def get_available_objects(self):
        if not self.available_objects_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Object registry service not available.")
            return []

        request = GetAvailableObjects.Request()
        future = self.available_objects_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().warn("GetAvailableObjects service timed out.")
            return []

        response = future.result()

        active_objects = []

        for object_id, label, status in zip(
            response.ids,
            response.labels,
            response.statuses
        ):
            if status == "ACTIVE":
                active_objects.append((object_id, label, status))

        return active_objects

    def keyboard_loop(self):
        while rclpy.ok():

            active_objects = self.get_available_objects()

            if not active_objects:
                print("\nNo ACTIVE objects available. Waiting...")
                continue

            print("\nAvailable ACTIVE Objects:")
            for object_id, label, status in active_objects:
                print(f"{object_id} -> {label} ({status})")

            user_input = input("\nSelect Object ID: ")

            try:
                selected_id = int(user_input)
            except ValueError:
                print("Invalid input. Please enter a number.")
                continue

            valid_ids = [
                object_id for object_id, _, _ in active_objects
            ]

            if selected_id not in valid_ids:
                print("Invalid selection. Object is not ACTIVE or does not exist.")
                continue

            msg = Int32()
            msg.data = selected_id

            self.publisher_.publish(msg)

            self.get_logger().info(
                f"Selected object ID: {msg.data}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = ObjectSelector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()