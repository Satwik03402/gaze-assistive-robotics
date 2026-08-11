#!/usr/bin/env python3

import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node

from summer_robotics_interfaces.msg import EyeGaze


class WebcamGazeNode(Node):

    def __init__(self):
        super().__init__("webcam_gaze_node")

        # ---------------------------------------------------------
        # ROS publisher
        # ---------------------------------------------------------
        self.gaze_pub = self.create_publisher(
            EyeGaze,
            "/eye_gaze",
            10
        )

        # ---------------------------------------------------------
        # Webcam
        # ---------------------------------------------------------
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error(
                "Camera could not be opened."
            )
            return

        # ---------------------------------------------------------
        # MediaPipe Face Mesh
        # refine_landmarks=True gives us iris landmarks.
        # ---------------------------------------------------------
        self.mp_face_mesh = mp.solutions.face_mesh

        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # ---------------------------------------------------------
        # ROS timer
        # ---------------------------------------------------------
        self.timer = self.create_timer(
            0.1,
            self.process_frame
        )

        self.frame_count = 0

        # ---------------------------------------------------------
        # Latest valid iris measurement
        # ---------------------------------------------------------
        self.latest_iris_ratio = None

        # ---------------------------------------------------------
        # User calibration references
        # ---------------------------------------------------------
        self.left_reference = None
        self.center_reference = None
        self.right_reference = None

        self.calibration_complete = False

        # These will be calculated after calibration.
        self.left_threshold = None
        self.right_threshold = None

        self.get_logger().info(
            "Webcam Iris Gaze Node started."
        )

        self.get_logger().info(
            "Calibration required."
        )

        self.get_logger().info(
            "Look LEFT and press L."
        )

        self.get_logger().info(
            "Look CENTER and press C."
        )

        self.get_logger().info(
            "Look RIGHT and press R."
        )
        self.calibration_capture_target = None
        self.calibration_capture_samples = []
        self.calibration_sample_count = 15

    # -------------------------------------------------------------
    # Compute horizontal iris position
    # -------------------------------------------------------------
    def compute_iris_ratio(self, face_landmarks):

        left_iris = face_landmarks.landmark[468]
        right_iris = face_landmarks.landmark[473]

        left_outer = face_landmarks.landmark[33]
        left_inner = face_landmarks.landmark[133]

        right_inner = face_landmarks.landmark[362]
        right_outer = face_landmarks.landmark[263]

        left_eye_width = (
            left_inner.x - left_outer.x
        )

        right_eye_width = (
            right_outer.x - right_inner.x
        )

        # Avoid division by an extremely small number.
        if (
            abs(left_eye_width) < 1e-6
            or abs(right_eye_width) < 1e-6
        ):
            return None

        left_ratio = (
            left_iris.x - left_outer.x
        ) / left_eye_width

        right_ratio = (
            right_iris.x - right_inner.x
        ) / right_eye_width

        iris_ratio = (
            left_ratio + right_ratio
        ) / 2.0

        return iris_ratio

    # -------------------------------------------------------------
    # Save a calibration reference
    # -------------------------------------------------------------
    def start_calibration_capture(self, direction):
        self.calibration_capture_target = direction
        self.calibration_capture_samples = []

        self.get_logger().info(
            f"Collecting {self.calibration_sample_count} samples for {direction}. "
            "Keep looking in that direction."
        )

    def update_calibration_capture(self, iris_ratio):
        if self.calibration_capture_target is None:
            return

        if iris_ratio is None:
            return

        self.calibration_capture_samples.append(iris_ratio)

        if len(self.calibration_capture_samples) < self.calibration_sample_count:
            return

        average_ratio = (
            sum(self.calibration_capture_samples)
            / len(self.calibration_capture_samples)
        )

        direction = self.calibration_capture_target

        if direction == "LEFT":
            self.left_reference = average_ratio

        elif direction == "CENTER":
            self.center_reference = average_ratio

        elif direction == "RIGHT":
            self.right_reference = average_ratio

        self.get_logger().info(
            f"{direction} calibration captured: "
            f"{average_ratio:.3f} "
            f"from {len(self.calibration_capture_samples)} samples"
        )

        self.calibration_capture_target = None
        self.calibration_capture_samples = []

        self.check_calibration()
    # -------------------------------------------------------------
    # Check whether all three references have been captured
    # -------------------------------------------------------------
    def check_calibration(self):

        if (
            self.left_reference is None
            or self.center_reference is None
            or self.right_reference is None
        ):
            return

        # Sanity check.
        #
        # With the current mirrored webcam image, our observed
        # measurements should normally increase:
        #
        # LEFT < CENTER < RIGHT
        #
        if not (
            self.left_reference
            < self.center_reference
            < self.right_reference
        ):
            self.calibration_complete = False

            self.get_logger().warn(
                "Calibration values are not ordered correctly."
            )

            self.get_logger().warn(
                "Expected LEFT < CENTER < RIGHT."
            )

            self.get_logger().warn(
                "Please repeat calibration using L, C and R."
            )

            return

        # Threshold halfway between LEFT and CENTER.
        self.left_threshold = (
            self.left_reference
            + self.center_reference
        ) / 2.0

        # Threshold halfway between CENTER and RIGHT.
        self.right_threshold = (
            self.center_reference
            + self.right_reference
        ) / 2.0

        self.calibration_complete = True

        self.get_logger().info(
            "Calibration complete."
        )

        self.get_logger().info(
            f"LEFT={self.left_reference:.3f}, "
            f"CENTER={self.center_reference:.3f}, "
            f"RIGHT={self.right_reference:.3f}"
        )

        self.get_logger().info(
            f"Thresholds: "
            f"LEFT<{self.left_threshold:.3f}, "
            f"RIGHT>{self.right_threshold:.3f}"
        )

        self.get_logger().info(
            "Normal gaze control enabled."
        )

    # -------------------------------------------------------------
    # Convert iris ratio into gaze state
    # -------------------------------------------------------------
    def classify_gaze(self, iris_ratio):

        # Don't allow gaze commands before calibration.
        if not self.calibration_complete:
            return "CALIBRATING"

        if iris_ratio < self.left_threshold:
            return "LOOKING_LEFT"

        elif iris_ratio > self.right_threshold:
            return "LOOKING_RIGHT"

        else:
            return "LOOKING_CENTER"

    # -------------------------------------------------------------
    # Main webcam callback
    # -------------------------------------------------------------
    def process_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn(
                "Failed to read frame."
            )
            return

        # Mirror webcam image.
        frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        results = self.face_mesh.process(
            rgb_frame
        )

        iris_ratio = None
        gaze_state = "NO_FACE"
        yaw = 0.0

        # ---------------------------------------------------------
        # Face detected
        # ---------------------------------------------------------
        if results.multi_face_landmarks:

            face_landmarks = (
                results.multi_face_landmarks[0]
            )

            # -----------------------------------------------------
            # Iris landmarks
            # -----------------------------------------------------
            left_iris = (
                face_landmarks.landmark[468]
            )

            right_iris = (
                face_landmarks.landmark[473]
            )

            # Convert normalized coordinates into pixels.
            left_iris_x = int(
                left_iris.x * frame.shape[1]
            )

            left_iris_y = int(
                left_iris.y * frame.shape[0]
            )

            right_iris_x = int(
                right_iris.x * frame.shape[1]
            )

            right_iris_y = int(
                right_iris.y * frame.shape[0]
            )

            # -----------------------------------------------------
            # Draw yellow pupil/iris markers
            # -----------------------------------------------------
            cv2.circle(
                frame,
                (left_iris_x, left_iris_y),
                4,
                (0, 255, 255),
                -1
            )

            cv2.circle(
                frame,
                (right_iris_x, right_iris_y),
                4,
                (0, 255, 255),
                -1
            )

            # -----------------------------------------------------
            # Calculate iris ratio
            # -----------------------------------------------------
            iris_ratio = self.compute_iris_ratio(
                face_landmarks
            )

            if iris_ratio is not None:

                self.latest_iris_ratio = iris_ratio

                self.update_calibration_capture(
                    iris_ratio
                )

                yaw = float(iris_ratio)

                gaze_state = self.classify_gaze(
                    iris_ratio
                )

        # ---------------------------------------------------------
        # Publish EyeGaze
        # ---------------------------------------------------------

        msg = EyeGaze()
        msg.yaw = yaw

        # Important:
        # Do not expose LEFT/RIGHT commands while calibrating.
        if gaze_state == "CALIBRATING":
            msg.gaze_state = "NO_FACE"
        else:
            msg.gaze_state = gaze_state

        self.gaze_pub.publish(msg)

        # ---------------------------------------------------------
        # Display current state
        # ---------------------------------------------------------

        if self.calibration_complete:

            display_state = gaze_state

        else:

            display_state = "CALIBRATING"

        cv2.putText(
            frame,
            f"State: {display_state}",
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        # ---------------------------------------------------------
        # Display iris ratio
        # ---------------------------------------------------------

        if iris_ratio is not None:

            cv2.putText(
                frame,
                f"Iris ratio: {iris_ratio:.3f}",
                (30, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 0),
                2
            )

        # ---------------------------------------------------------
        # Display calibration instructions / values
        # ---------------------------------------------------------

        if not self.calibration_complete:

            cv2.putText(
                frame,
                "Calibration: L=Left C=Center R=Right",
                (30, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

            left_text = (
                "---"
                if self.left_reference is None
                else f"{self.left_reference:.3f}"
            )

            center_text = (
                "---"
                if self.center_reference is None
                else f"{self.center_reference:.3f}"
            )

            right_text = (
                "---"
                if self.right_reference is None
                else f"{self.right_reference:.3f}"
            )

            cv2.putText(
                frame,
                f"L:{left_text} C:{center_text} R:{right_text}",
                (30, 165),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            if self.calibration_capture_target is not None:
                cv2.putText(
                    frame,
                    f"Collecting: {self.calibration_capture_target}",
                    (30, 200),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

        # ---------------------------------------------------------
        # Occasional ROS logging
        # ---------------------------------------------------------

        self.frame_count += 1

        if (
            iris_ratio is not None
            and self.frame_count % 10 == 0
        ):
            self.get_logger().info(
                f"Iris ratio: {iris_ratio:.3f}"
            )

        # ---------------------------------------------------------
        # Show webcam
        # ---------------------------------------------------------

        cv2.imshow(
            "Webcam Iris Gaze Node",
            frame
        )

        # ---------------------------------------------------------
        # Keyboard controls
        # ---------------------------------------------------------

        key = cv2.waitKey(1) & 0xFF

        if key == ord("l"):
            self.start_calibration_capture("LEFT")

        elif key == ord("c"):
            self.start_calibration_capture("CENTER")

        elif key == ord("r"):
            self.start_calibration_capture("RIGHT")

        elif key == ord("q"):
            rclpy.shutdown()

    # -------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------
    def destroy_node(self):

        if hasattr(self, "cap"):
            if self.cap.isOpened():
                self.cap.release()

        if hasattr(self, "face_mesh"):
            self.face_mesh.close()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = WebcamGazeNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()