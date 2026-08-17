#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from summer_robotics_interfaces.msg import EyeGaze, DetectedObjectArray


class GazeObjectSelectorNode(Node):

    def __init__(self):
        super().__init__("gaze_object_selector_node")

        self.latest_horizontal_ratio = 0.0
        self.latest_vertical_ratio = 0.0

        self.latest_horizontal_state = "NO_FACE"
        self.latest_vertical_state = "NO_FACE"

        self.detected_objects = []

        self.last_selected_id = None
        self.selection_published = False

        # Candidate timing
        self.raw_candidate_id = None
        self.raw_candidate_start_time = None
        self.candidate_stability_duration = 0.25

        self.candidate_object_id = None
        self.candidate_start_time = None
        self.dwell_duration = 1.0

        # Short gaze-loss grace period
        self.gaze_loss_start_time = None
        self.gaze_loss_grace_duration = 0.30

        # Horizontal iris-to-workspace calibration
        self.iris_left = 0.417
        self.iris_center = 0.482
        self.iris_right = 0.527

        self.workspace_left_y = 0.10
        self.workspace_center_y = 0.0
        self.workspace_right_y = -0.25

        # Vertical iris-to-workspace calibration
        self.iris_up = 0.410
        self.iris_vertical_center = 0.381
        self.iris_down = 0.344

        self.workspace_up_x = 0.75
        self.workspace_center_x = 0.90
        self.workspace_down_x = 1.05

        self.max_gaze_object_distance = 0.12

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

        self.candidate_pub = self.create_publisher(
            Int32,
            "/gaze_candidate_object_id",
            10
        )

        self.get_logger().info(
            "Gaze Object Selector Node started."
        )

    def gaze_callback(self, msg):
        self.latest_horizontal_ratio = msg.horizontal_ratio
        self.latest_vertical_ratio = msg.vertical_ratio

        self.latest_horizontal_state = msg.horizontal_state
        self.latest_vertical_state = msg.vertical_state

        self.update_selection()

    def objects_callback(self, msg):
        self.detected_objects = msg.objects

    def get_pickable_objects(self):
        return [
            obj
            for obj in self.detected_objects
            if (
                obj.pickable
                and obj.status == "ACTIVE"
                and obj.label != "green_cube"
            )
        ]

    def reset_candidate(self):
        if self.candidate_object_id is not None:
            candidate_msg = Int32()
            candidate_msg.data = -1
            self.candidate_pub.publish(candidate_msg)

        self.candidate_object_id = None
        self.candidate_start_time = None

        self.raw_candidate_id = None
        self.raw_candidate_start_time = None

        self.gaze_loss_start_time = None
        self.selection_published = False

    def estimate_workspace_y(self, horizontal_ratio):
        if horizontal_ratio <= self.iris_center:
            ratio = (
                horizontal_ratio - self.iris_left
            ) / (
                self.iris_center - self.iris_left
            )

            ratio = max(0.0, min(1.0, ratio))

            return (
                self.workspace_left_y
                + ratio
                * (
                    self.workspace_center_y
                    - self.workspace_left_y
                )
            )

        ratio = (
            horizontal_ratio - self.iris_center
        ) / (
            self.iris_right - self.iris_center
        )

        ratio = max(0.0, min(1.0, ratio))

        return (
            self.workspace_center_y
            + ratio
            * (
                self.workspace_right_y
                - self.workspace_center_y
            )
        )

    def estimate_workspace_x(self, vertical_ratio):
        if vertical_ratio >= self.iris_vertical_center:
            ratio = (
                vertical_ratio - self.iris_vertical_center
            ) / (
                self.iris_up - self.iris_vertical_center
            )

            ratio = max(0.0, min(1.0, ratio))

            return (
                self.workspace_center_x
                + ratio
                * (
                    self.workspace_up_x
                    - self.workspace_center_x
                )
            )

        ratio = (
            self.iris_vertical_center - vertical_ratio
        ) / (
            self.iris_vertical_center - self.iris_down
        )

        ratio = max(0.0, min(1.0, ratio))

        return (
            self.workspace_center_x
            + ratio
            * (
                self.workspace_down_x
                - self.workspace_center_x
            )
        )

    def update_selection(self):
        pickable_objects = self.get_pickable_objects()

        if not pickable_objects:
            self.reset_candidate()
            return

        if (
            self.latest_horizontal_state == "NO_FACE"
            or self.latest_vertical_state == "NO_FACE"
        ):
            self.reset_candidate()
            return

        estimated_y = self.estimate_workspace_y(
            self.latest_horizontal_ratio
        )

        estimated_x = self.estimate_workspace_x(
            self.latest_vertical_ratio
        )

        # Find nearest object using true 2-D distance.
        selected_object = min(
            pickable_objects,
            key=lambda obj: math.sqrt(
                (obj.x - estimated_x) ** 2
                + (obj.y - estimated_y) ** 2
            )
        )

        distance = math.sqrt(
            (selected_object.x - estimated_x) ** 2
            + (selected_object.y - estimated_y) ** 2
        )

        self.get_logger().info(
            f"Gaze estimate: "
            f"x={estimated_x:.3f}, "
            f"y={estimated_y:.3f}, "
            f"nearest={selected_object.label}, "
            f"distance={distance:.3f}"
        )

        if distance > self.max_gaze_object_distance:
            selected_object = None

        # Allow brief gaze excursions without clearing immediately.
        if selected_object is None:
            current_time = self.get_clock().now()

            if self.gaze_loss_start_time is None:
                self.gaze_loss_start_time = current_time
                return

            lost_elapsed = (
                current_time - self.gaze_loss_start_time
            ).nanoseconds / 1_000_000_000.0

            if lost_elapsed < self.gaze_loss_grace_duration:
                return

            self.reset_candidate()
            return

        self.gaze_loss_start_time = None
        current_time = self.get_clock().now()

        # Stabilize raw candidate.
        if self.raw_candidate_id != selected_object.id:
            self.raw_candidate_id = selected_object.id
            self.raw_candidate_start_time = current_time
            return

        if self.raw_candidate_start_time is None:
            self.raw_candidate_start_time = current_time
            return

        raw_elapsed = (
            current_time - self.raw_candidate_start_time
        ).nanoseconds / 1_000_000_000.0

        if raw_elapsed < self.candidate_stability_duration:
            return

        # Promote raw candidate to stable candidate.
        if self.candidate_object_id != selected_object.id:
            self.candidate_object_id = selected_object.id
            self.candidate_start_time = current_time
            self.selection_published = False

            candidate_msg = Int32()
            candidate_msg.data = selected_object.id
            self.candidate_pub.publish(candidate_msg)

            self.get_logger().info(
                f"Stable gaze candidate: "
                f"object ID {selected_object.id}, "
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

        # Final selection after dwell.
        selected_msg = Int32()
        selected_msg.data = selected_object.id
        self.selected_pub.publish(selected_msg)

        self.selection_published = True
        self.last_selected_id = selected_object.id

        self.get_logger().info(
            f"Gaze selection confirmed after {elapsed:.2f}s: "
            f"object ID {selected_object.id}, "
            f"{selected_object.label} "
            f"(horizontal_ratio={self.latest_horizontal_ratio:.3f}, "
            f"vertical_ratio={self.latest_vertical_ratio:.3f}, "
            f"estimated_x={estimated_x:.3f}, "
            f"estimated_y={estimated_y:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)

    node = GazeObjectSelectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()