#!/usr/bin/env python3

import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node

from summer_robotics_interfaces.msg import EyeGaze


class WebcamGazeNode(Node):

    def __init__(self):
        super().__init__("webcam_gaze_node")

        self.gaze_pub = self.create_publisher(
            EyeGaze,
            "/eye_gaze",
            10
        )

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Camera could not be opened.")
            return

        self.mp_face_mesh = mp.solutions.face_mesh

        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.timer = self.create_timer(
            0.1,
            self.process_frame
        )

        self.frame_count = 0

        # Horizontal calibration
        self.left_reference = None
        self.center_reference = None
        self.right_reference = None

        self.left_threshold = None
        self.right_threshold = None
        self.horizontal_calibration_complete = False

        # Vertical calibration
        self.up_reference = None
        self.vertical_center_reference = None
        self.down_reference = None

        self.up_threshold = None
        self.down_threshold = None
        self.vertical_calibration_complete = False
        self.vertical_increases_upward = None

        # Calibration capture state
        self.calibration_target = None
        self.horizontal_samples = []
        self.vertical_samples = []
        self.calibration_sample_count = 15

        self.get_logger().info(
            "Webcam Iris Gaze Node started."
        )

        self.get_logger().info(
            "Calibration keys: "
            "L=Left, C=Center, R=Right, U=Up, D=Down"
        )

    def compute_iris_ratios(self, face_landmarks):

        left_iris = face_landmarks.landmark[468]
        right_iris = face_landmarks.landmark[473]

        left_outer = face_landmarks.landmark[33]
        left_inner = face_landmarks.landmark[133]

        right_inner = face_landmarks.landmark[362]
        right_outer = face_landmarks.landmark[263]

        left_top = face_landmarks.landmark[159]
        left_bottom = face_landmarks.landmark[145]

        right_top = face_landmarks.landmark[386]
        right_bottom = face_landmarks.landmark[374]

        left_eye_width = left_inner.x - left_outer.x
        right_eye_width = right_outer.x - right_inner.x

        left_eye_height = left_bottom.y - left_top.y
        right_eye_height = right_bottom.y - right_top.y

        if (
            abs(left_eye_width) < 1e-6
            or abs(right_eye_width) < 1e-6
            or abs(left_eye_height) < 1e-6
            or abs(right_eye_height) < 1e-6
        ):
            return None

        left_horizontal = (
            left_iris.x - left_outer.x
        ) / left_eye_width

        right_horizontal = (
            right_iris.x - right_inner.x
        ) / right_eye_width

        horizontal_ratio = (
            left_horizontal + right_horizontal
        ) / 2.0

        left_vertical = (
            left_iris.y - left_top.y
        ) / left_eye_height

        right_vertical = (
            right_iris.y - right_top.y
        ) / right_eye_height

        vertical_ratio = (
            left_vertical + right_vertical
        ) / 2.0

        return horizontal_ratio, vertical_ratio

    def start_calibration(self, target):

        self.calibration_target = target
        self.horizontal_samples = []
        self.vertical_samples = []

        self.get_logger().info(
            f"Collecting calibration samples for {target}. "
            "Keep looking in that direction."
        )

    def update_calibration(
        self,
        horizontal_ratio,
        vertical_ratio
    ):

        if self.calibration_target is None:
            return

        target = self.calibration_target

        if target in ["LEFT", "RIGHT"]:

            if horizontal_ratio is None:
                return

            self.horizontal_samples.append(
                horizontal_ratio
            )

            if (
                len(self.horizontal_samples)
                < self.calibration_sample_count
            ):
                return

            average_horizontal = (
                sum(self.horizontal_samples)
                / len(self.horizontal_samples)
            )

            if target == "LEFT":
                self.left_reference = average_horizontal

            elif target == "RIGHT":
                self.right_reference = average_horizontal

            self.get_logger().info(
                f"{target} captured: "
                f"H={average_horizontal:.3f}"
            )

        elif target in ["UP", "DOWN"]:

            if vertical_ratio is None:
                return

            self.vertical_samples.append(
                vertical_ratio
            )

            if (
                len(self.vertical_samples)
                < self.calibration_sample_count
            ):
                return

            average_vertical = (
                sum(self.vertical_samples)
                / len(self.vertical_samples)
            )

            if target == "UP":
                self.up_reference = average_vertical

            elif target == "DOWN":
                self.down_reference = average_vertical

            self.get_logger().info(
                f"{target} captured: "
                f"V={average_vertical:.3f}"
            )

        elif target == "CENTER":

            if (
                horizontal_ratio is None
                or vertical_ratio is None
            ):
                return

            self.horizontal_samples.append(
                horizontal_ratio
            )

            self.vertical_samples.append(
                vertical_ratio
            )

            if (
                len(self.horizontal_samples)
                < self.calibration_sample_count
            ):
                return

            average_horizontal = (
                sum(self.horizontal_samples)
                / len(self.horizontal_samples)
            )

            average_vertical = (
                sum(self.vertical_samples)
                / len(self.vertical_samples)
            )

            self.center_reference = average_horizontal
            self.vertical_center_reference = average_vertical

            self.get_logger().info(
                "CENTER captured: "
                f"H={average_horizontal:.3f}, "
                f"V={average_vertical:.3f}"
            )

        self.calibration_target = None
        self.horizontal_samples = []
        self.vertical_samples = []

        self.check_horizontal_calibration()
        self.check_vertical_calibration()

    def check_horizontal_calibration(self):

        if (
            self.left_reference is None
            or self.center_reference is None
            or self.right_reference is None
        ):
            return

        if not (
            self.left_reference
            < self.center_reference
            < self.right_reference
        ):
            self.horizontal_calibration_complete = False

            self.get_logger().warn(
                "Horizontal calibration invalid. "
                "Expected LEFT < CENTER < RIGHT."
            )
            return

        self.left_threshold = (
            self.left_reference
            + self.center_reference
        ) / 2.0

        self.right_threshold = (
            self.center_reference
            + self.right_reference
        ) / 2.0

        self.horizontal_calibration_complete = True

        self.get_logger().info(
            "Horizontal calibration complete: "
            f"L={self.left_reference:.3f}, "
            f"C={self.center_reference:.3f}, "
            f"R={self.right_reference:.3f}"
        )

    def check_vertical_calibration(self):

        if (
            self.up_reference is None
            or self.vertical_center_reference is None
            or self.down_reference is None
        ):
            return

        increasing_downward = (
            self.up_reference
            < self.vertical_center_reference
            < self.down_reference
        )

        increasing_upward = (
            self.down_reference
            < self.vertical_center_reference
            < self.up_reference
        )

        if not (
            increasing_downward
            or increasing_upward
        ):
            self.vertical_calibration_complete = False

            self.get_logger().warn(
                "Vertical calibration invalid. "
                "CENTER must lie between UP and DOWN."
            )
            return

        self.vertical_increases_upward = increasing_upward

        self.up_threshold = (
            self.up_reference
            + self.vertical_center_reference
        ) / 2.0

        self.down_threshold = (
            self.vertical_center_reference
            + self.down_reference
        ) / 2.0

        self.vertical_calibration_complete = True

        direction = (
            "increases upward"
            if self.vertical_increases_upward
            else "increases downward"
        )

        self.get_logger().info(
            "Vertical calibration complete: "
            f"U={self.up_reference:.3f}, "
            f"C={self.vertical_center_reference:.3f}, "
            f"D={self.down_reference:.3f}, "
            f"signal {direction}"
        )

    def classify_horizontal_gaze(
        self,
        horizontal_ratio
    ):

        if not self.horizontal_calibration_complete:
            return "CALIBRATING"

        if horizontal_ratio < self.left_threshold:
            return "LOOKING_LEFT"

        if horizontal_ratio > self.right_threshold:
            return "LOOKING_RIGHT"

        return "LOOKING_CENTER"

    def classify_vertical_gaze(
        self,
        vertical_ratio
    ):

        if not self.vertical_calibration_complete:
            return "CALIBRATING"

        if self.vertical_increases_upward:

            if vertical_ratio > self.up_threshold:
                return "LOOKING_UP"

            if vertical_ratio < self.down_threshold:
                return "LOOKING_DOWN"

        else:

            if vertical_ratio < self.up_threshold:
                return "LOOKING_UP"

            if vertical_ratio > self.down_threshold:
                return "LOOKING_DOWN"

        return "LOOKING_CENTER"

    def process_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn(
                "Failed to read frame."
            )
            return

        frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        results = self.face_mesh.process(
            rgb_frame
        )

        horizontal_ratio = None
        vertical_ratio = None
        raw_vertical_ratio = None

        horizontal_state = "NO_FACE"
        vertical_state = "NO_FACE"

        if results.multi_face_landmarks:

            face_landmarks = (
                results.multi_face_landmarks[0]
            )

            left_iris = (
                face_landmarks.landmark[468]
            )

            right_iris = (
                face_landmarks.landmark[473]
            )

            raw_vertical_ratio = (
                left_iris.y + right_iris.y
            ) / 2.0

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

            ratios = self.compute_iris_ratios(
                face_landmarks
            )

            if ratios is not None:

                (
                    horizontal_ratio,
                    vertical_ratio
                ) = ratios

                self.update_calibration(
                    horizontal_ratio,
                    vertical_ratio
                )

                horizontal_state = (
                    self.classify_horizontal_gaze(
                        horizontal_ratio
                    )
                )

                vertical_state = (
                    self.classify_vertical_gaze(
                        vertical_ratio
                    )
                )

        msg = EyeGaze()

        if horizontal_ratio is None:
            msg.horizontal_ratio = 0.0
        else:
            msg.horizontal_ratio = float(horizontal_ratio)

        if vertical_ratio is None:
            msg.vertical_ratio = 0.0
        else:
            msg.vertical_ratio = float(vertical_ratio)

        if horizontal_state == "CALIBRATING":
            msg.horizontal_state = "NO_FACE"
        else:
            msg.horizontal_state = horizontal_state

        if vertical_state == "CALIBRATING":
            msg.vertical_state = "NO_FACE"
        else:
            msg.vertical_state = vertical_state

        self.gaze_pub.publish(msg)

        cv2.putText(
            frame,
            f"H State: {horizontal_state}",
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"V State: {vertical_state}",
            (30, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

        if horizontal_ratio is not None:
            cv2.putText(
                frame,
                f"H ratio: {horizontal_ratio:.3f}",
                (30, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2
            )

        if vertical_ratio is not None:
            cv2.putText(
                frame,
                f"V ratio: {vertical_ratio:.3f}",
                (30, 145),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2
            )

        if raw_vertical_ratio is not None:
            cv2.putText(
                frame,
                f"Raw V: {raw_vertical_ratio:.3f}",
                (30, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2
            )

        left_text = (
            "---"
            if self.left_reference is None
            else f"{self.left_reference:.3f}"
        )

        center_h_text = (
            "---"
            if self.center_reference is None
            else f"{self.center_reference:.3f}"
        )

        right_text = (
            "---"
            if self.right_reference is None
            else f"{self.right_reference:.3f}"
        )

        up_text = (
            "---"
            if self.up_reference is None
            else f"{self.up_reference:.3f}"
        )

        center_v_text = (
            "---"
            if self.vertical_center_reference is None
            else f"{self.vertical_center_reference:.3f}"
        )

        down_text = (
            "---"
            if self.down_reference is None
            else f"{self.down_reference:.3f}"
        )

        cv2.putText(
            frame,
            "Calibration: L C R U D",
            (30, 220),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"H  L:{left_text} "
            f"C:{center_h_text} "
            f"R:{right_text}",
            (30, 250),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"V  U:{up_text} "
            f"C:{center_v_text} "
            f"D:{down_text}",
            (30, 280),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        if self.calibration_target is not None:
            cv2.putText(
                frame,
                f"Collecting: {self.calibration_target}",
                (30, 315),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        self.frame_count += 1

        if (
            horizontal_ratio is not None
            and vertical_ratio is not None
            and self.frame_count % 10 == 0
        ):
            self.get_logger().info(
                f"H={horizontal_ratio:.3f}, "
                f"V={vertical_ratio:.3f}, "
                f"HState={horizontal_state}, "
                f"VState={vertical_state}"
            )

        cv2.imshow(
            "Webcam Iris Gaze Node",
            frame
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("l"):
            self.start_calibration("LEFT")

        elif key == ord("c"):
            self.start_calibration("CENTER")

        elif key == ord("r"):
            self.start_calibration("RIGHT")

        elif key == ord("u"):
            self.start_calibration("UP")

        elif key == ord("d"):
            self.start_calibration("DOWN")

        elif key == ord("q"):
            rclpy.shutdown()

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