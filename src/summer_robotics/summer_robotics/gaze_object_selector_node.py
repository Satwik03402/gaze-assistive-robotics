#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from summer_robotics_interfaces.msg import EyeGaze, DetectedObjectArray


class GazeObjectSelectorNode(Node):

    def __init__(self):
        super().__init__("gaze_object_selector_node")

        # Latest gaze information
        self.latest_gaze_state = "NO_FACE"
        self.latest_yaw = 0.0

        # Latest detected objects
        self.detected_objects = []

        # Final selection state
        self.last_selected_id = None
        self.selection_published = False

        # Stable candidate state
        self.candidate_object_id = None
        self.candidate_start_time = None

        # Raw candidate stability filtering
        self.raw_candidate_id = None
        self.raw_candidate_start_time = None
        self.candidate_stability_duration = 0.25

        # Dwell selection
        self.dwell_duration = 1.0

        # Short grace period for noisy CENTER / NO_FACE frames
        self.gaze_loss_start_time = None
        self.gaze_loss_grace_duration = 0.30

        # Subscribe to iris-based gaze
        self.gaze_sub = self.create_subscription(
            EyeGaze,
            "/eye_gaze",
            self.gaze_callback,
            10
        )

        # Subscribe to detected objects
        self.objects_sub = self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self.objects_callback,
            10
        )

        # Publish confirmed object selection
        self.selected_pub = self.create_publisher(
            Int32,
            "/selected_object_id",
            10
        )

        # Publish current stable gaze candidate
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

    def objects_callback(self, msg):
        self.detected_objects = msg.objects

        self.update_selection()

    def get_pickable_objects(self):
        pickable_objects = []

        for obj in self.detected_objects:
            if (
                obj.pickable
                and obj.status == "ACTIVE"
                and obj.label != "green_cube"
            ):
                pickable_objects.append(obj)

        return pickable_objects

    def reset_candidate(self):
        # Tell the Gazebo indicator that there is
        # no longer an active candidate.
        if self.candidate_object_id is not None:
            candidate_msg = Int32()
            candidate_msg.data = -1

            self.candidate_pub.publish(candidate_msg)

        # Clear stable candidate
        self.candidate_object_id = None
        self.candidate_start_time = None

        # Clear raw candidate
        self.raw_candidate_id = None
        self.raw_candidate_start_time = None

        # Clear gaze-loss timer
        self.gaze_loss_start_time = None

        # Allow a future selection
        self.selection_published = False

    def update_selection(self):
        pickable_objects = self.get_pickable_objects()

        if len(pickable_objects) == 0:
            self.reset_candidate()
            return

        selected_object = None

        # --------------------------------------------------
        # STEP 1: Convert gaze direction into an object
        # --------------------------------------------------

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
            # CENTER / NO_FACE may occur briefly because
            # iris tracking is noisy. Do not immediately
            # destroy the current candidate.
            current_time = self.get_clock().now()

            if self.gaze_loss_start_time is None:
                self.gaze_loss_start_time = current_time
                return

            lost_elapsed = (
                current_time - self.gaze_loss_start_time
            ).nanoseconds / 1_000_000_000.0

            if lost_elapsed < self.gaze_loss_grace_duration:
                return

            # Gaze has genuinely left the object.
            self.reset_candidate()
            return

        # We have valid LEFT or RIGHT gaze again.
        self.gaze_loss_start_time = None

        if selected_object is None:
            return

        current_time = self.get_clock().now()

        # --------------------------------------------------
        # STEP 2: Stabilize the raw candidate
        # --------------------------------------------------

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

        # Ignore very short gaze fluctuations.
        if raw_elapsed < self.candidate_stability_duration:
            return

        # --------------------------------------------------
        # STEP 3: Publish stable candidate
        # --------------------------------------------------

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

        # --------------------------------------------------
        # STEP 4: Dwell on stable candidate
        # --------------------------------------------------

        if self.candidate_start_time is None:
            self.candidate_start_time = current_time
            return

        elapsed = (
            current_time - self.candidate_start_time
        ).nanoseconds / 1_000_000_000.0

        if elapsed < self.dwell_duration:
            return

        # Don't repeatedly publish the same selection
        # while the user continues looking at the object.
        if self.selection_published:
            return

        # --------------------------------------------------
        # STEP 5: Confirm selection
        # --------------------------------------------------

        selected_msg = Int32()
        selected_msg.data = selected_object.id

        self.selected_pub.publish(selected_msg)

        self.selection_published = True
        self.last_selected_id = selected_object.id

        self.get_logger().info(
            f"Gaze selection confirmed after "
            f"{elapsed:.2f}s: "
            f"object ID {selected_object.id}, "
            f"{selected_object.label} "
            f"(state={self.latest_gaze_state}, "
            f"iris_ratio={self.latest_yaw:.3f})"
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