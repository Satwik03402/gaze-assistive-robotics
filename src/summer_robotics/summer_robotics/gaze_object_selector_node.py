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
        self.selection_published = False

        # Candidate timing
        self.raw_candidate_id = None
        self.raw_candidate_start_time = None
        self.candidate_stability_duration = 0.25

        self.candidate_object_id = None
        self.candidate_start_time = None
        self.dwell_duration = 1.0

        # Brief tracking loss should not immediately clear a candidate
        self.gaze_loss_start_time = None
        self.gaze_loss_grace_duration = 0.30

        # Iris-to-workspace calibration
        self.iris_left = 0.417
        self.iris_center = 0.482
        self.iris_right = 0.527

        self.workspace_left_y = 0.10
        self.workspace_center_y = 0.0
        self.workspace_right_y = -0.25

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
        self.latest_gaze_state = msg.gaze_state
        self.latest_yaw = msg.yaw

        # Gaze updates drive the selection timing.
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
            msg = Int32()
            msg.data = -1
            self.candidate_pub.publish(msg)

        self.candidate_object_id = None
        self.candidate_start_time = None

        self.raw_candidate_id = None
        self.raw_candidate_start_time = None

        self.gaze_loss_start_time = None
        self.selection_published = False

    def estimate_workspace_y(self, iris_ratio):
        # Piecewise interpolation around the calibrated center.
        if iris_ratio <= self.iris_center:
            ratio = (
                iris_ratio - self.iris_left
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
            iris_ratio - self.iris_center
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

    def update_selection(self):
        pickable_objects = self.get_pickable_objects()

        if not pickable_objects:
            self.reset_candidate()
            return

        if self.latest_gaze_state == "NO_FACE":
            self.reset_candidate()
            return

        estimated_y = self.estimate_workspace_y(
            self.latest_yaw
        )

        # Find the object nearest to the estimated gaze position.
        selected_object = min(
            pickable_objects,
            key=lambda obj: abs(obj.y - estimated_y)
        )

        distance = abs(
            selected_object.y - estimated_y
        )

        if distance > self.max_gaze_object_distance:
            selected_object = None

        # Allow short periods of gaze away from an object.
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

        # Require the same raw target for a short period.
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

        # Promote the raw target to a stable candidate.
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

        # Dwell completed: publish the final object selection.
        selected_msg = Int32()
        selected_msg.data = selected_object.id
        self.selected_pub.publish(selected_msg)

        self.selection_published = True
        self.last_selected_id = selected_object.id

        self.get_logger().info(
            f"Gaze selection confirmed after {elapsed:.2f}s: "
            f"object ID {selected_object.id}, "
            f"{selected_object.label} "
            f"(iris_ratio={self.latest_yaw:.3f}, "
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