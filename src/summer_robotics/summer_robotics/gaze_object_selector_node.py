#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from summer_robotics_interfaces.msg import EyeGaze, DetectedObjectArray


class GazeObjectSelectorNode(Node):

    def __init__(self):
        super().__init__("gaze_object_selector_node")

        self.latest_gaze_state = "NO_FACE"
        self.latest_yaw = 0.0
        self.detected_objects = []
        self.last_selected_id = None

        self.gaze_sub = self.create_subscription(
            EyeGaze,
            "/eye_gaze",
            self.gaze_callback,
            10
        )

        self.objects_sub = self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self.objects_callback,
            10
        )

        self.selected_pub = self.create_publisher(
            Int32,
            "/selected_object_id",
            10
        )

        self.get_logger().info("Gaze Object Selector Node started.")

    def gaze_callback(self, msg):
        self.latest_gaze_state = msg.gaze_state
        self.latest_yaw = msg.yaw
        self.update_selection()

    def objects_callback(self, msg):
        self.detected_objects = msg.objects
        self.update_selection()

    def get_pickable_objects(self):
        pickable_objects = []

        for obj in self.detected_objects:
            if obj.pickable and obj.status == "ACTIVE":
                pickable_objects.append(obj)

        return pickable_objects

    def update_selection(self):
        pickable_objects = self.get_pickable_objects()

        if len(pickable_objects) == 0:
            return

        selected_object = None

        if self.latest_gaze_state == "LOOKING_LEFT":
            selected_object = max(
                pickable_objects,
                key=lambda obj: obj.y
            )

        elif self.latest_gaze_state == "LOOKING_RIGHT":
            selected_object = min(
                pickable_objects,
                key=lambda obj: obj.y
            )

        else:
            return

        if selected_object is None:
            return

        selected_msg = Int32()
        selected_msg.data = selected_object.id

        self.selected_pub.publish(selected_msg)

        if self.last_selected_id != selected_object.id:
            self.get_logger().info(
                f"Gaze selected object ID {selected_object.id}: "
                f"{selected_object.label} "
                f"(state={self.latest_gaze_state}, yaw={self.latest_yaw:.2f})"
            )

        self.last_selected_id = selected_object.id


def main(args=None):
    rclpy.init(args=args)

    node = GazeObjectSelectorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()