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

        self.candidate_object_id = None
        self.candidate_start_time = None
        self.selection_published = False
        self.dwell_duration = 1.0
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

    def reset_candidate(self):
        self.candidate_object_id = None
        self.candidate_start_time = None
        self.selection_published = False

    def update_selection(self):
        pickable_objects = self.get_pickable_objects()

        if len(pickable_objects) == 0:
            self.reset_candidate()
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
            self.reset_candidate()
            return

        if selected_object is None:
            return

        current_time = self.get_clock().now()

        if self.candidate_object_id != selected_object.id:
            self.candidate_object_id = selected_object.id
            self.candidate_start_time = current_time
            self.selection_published = False

            self.get_logger().info(
                f"Gaze candidate: object ID {selected_object.id}, "
                f"{selected_object.label}"
            )
            return

        if self.candidate_start_time is None:
            self.candidate_start_time = current_time
            return

        elapsed = (
            current_time - self.candidate_start_time
        ).nanoseconds / 1_000_000_000.0

        if elapsed < self.dwell_duration:
            return

        if self.selection_published:
            return

        selected_msg = Int32()
        selected_msg.data = selected_object.id
        self.selected_pub.publish(selected_msg)

        self.selection_published = True
        self.last_selected_id = selected_object.id

        self.get_logger().info(
            f"Gaze selection confirmed after {elapsed:.2f}s: "
            f"object ID {selected_object.id}, "
            f"{selected_object.label} "
            f"(state={self.latest_gaze_state}, "
            f"yaw={self.latest_yaw:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)

    node = GazeObjectSelectorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
