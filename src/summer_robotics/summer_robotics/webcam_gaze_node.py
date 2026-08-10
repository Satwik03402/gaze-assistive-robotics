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

        self.get_logger().info(
            "Webcam Iris Gaze Node started."
        )

    def compute_iris_ratio(self, face_landmarks):

        left_iris = face_landmarks.landmark[468]
        right_iris = face_landmarks.landmark[473]

        left_outer = face_landmarks.landmark[33]
        left_inner = face_landmarks.landmark[133]

        right_inner = face_landmarks.landmark[362]
        right_outer = face_landmarks.landmark[263]

        left_eye_width = left_inner.x - left_outer.x
        right_eye_width = right_outer.x - right_inner.x

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

        return (left_ratio + right_ratio) / 2.0

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

        iris_ratio = None
        gaze_state = "NO_FACE"
        yaw = 0.0

        if results.multi_face_landmarks:

            face_landmarks = results.multi_face_landmarks[0]

            left_iris = face_landmarks.landmark[468]
            right_iris = face_landmarks.landmark[473]

            left_iris_x = int(left_iris.x * frame.shape[1])
            left_iris_y = int(left_iris.y * frame.shape[0])

            right_iris_x = int(right_iris.x * frame.shape[1])
            right_iris_y = int(right_iris.y * frame.shape[0])

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

            iris_ratio = self.compute_iris_ratio(
                face_landmarks
            )

            if iris_ratio is not None:

                yaw = float(iris_ratio)

                if iris_ratio < 0.45:
                    gaze_state = "LOOKING_LEFT"

                elif iris_ratio > 0.51:
                    gaze_state = "LOOKING_RIGHT"

                else:
                    gaze_state = "LOOKING_CENTER"

        msg = EyeGaze()
        msg.yaw = yaw
        msg.gaze_state = gaze_state

        self.gaze_pub.publish(msg)

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

        cv2.putText(
            frame,
            f"State: {gaze_state}",
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        self.frame_count += 1

        if (
            iris_ratio is not None
            and self.frame_count % 10 == 0
        ):
            self.get_logger().info(
                f"Iris ratio: {iris_ratio:.3f}"
            )

        cv2.imshow(
            "Webcam Iris Gaze Node",
            frame
        )

        if (
            cv2.waitKey(1) & 0xFF
            == ord("q")
        ):
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