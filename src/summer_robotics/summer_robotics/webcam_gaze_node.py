#!/usr/bin/env python3

import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node
from summer_robotics_interfaces.msg import EyeGaze


def compute_gaze(nose_x, frame_width):
    center_x = frame_width / 2
    offset = nose_x - center_x
    yaw = offset / center_x

    if yaw < -0.2:
        gaze_state = "LOOKING_LEFT"
    elif yaw > 0.2:
        gaze_state = "LOOKING_RIGHT"
    else:
        gaze_state = "LOOKING_CENTER"

    return yaw, gaze_state


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

        self.get_logger().info("Webcam Gaze Node started.")

    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame.")
            return

        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb_frame)

        yaw = 0.0
        gaze_state = "NO_FACE"

        if results.multi_face_landmarks:
            face_landmarks = results.multi_face_landmarks[0]

            nose = face_landmarks.landmark[1]
            nose_x = int(nose.x * width)
            nose_y = int(nose.y * height)

            yaw, gaze_state = compute_gaze(nose_x, width)

            cv2.circle(frame, (nose_x, nose_y), 5, (0, 255, 0), -1)

        msg = EyeGaze()
        msg.yaw = float(yaw)
        msg.gaze_state = gaze_state
        self.gaze_pub.publish(msg)

        cv2.putText(
            frame,
            f"{gaze_state} yaw={yaw:.2f}",
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        cv2.imshow("Webcam Gaze Node", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()

        self.face_mesh.close()
        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WebcamGazeNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()