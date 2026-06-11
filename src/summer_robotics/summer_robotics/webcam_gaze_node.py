#!/usr/bin/env python3

import cv2
import mediapipe as mp


def classify_head_position(nose_x, frame_width):
    center_x = frame_width / 2
    offset = nose_x - center_x

    if offset < -60:
        return "LOOKING_LEFT"
    elif offset > 60:
        return "LOOKING_RIGHT"
    else:
        return "LOOKING_CENTER"


def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Camera could not be opened.")
        return

    mp_face_mesh = mp.solutions.face_mesh

    with mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as face_mesh:

        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to read frame.")
                break

            frame = cv2.flip(frame, 1)
            height, width, _ = frame.shape

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = face_mesh.process(rgb_frame)

            gaze_state = "NO_FACE"

            if results.multi_face_landmarks:
                face_landmarks = results.multi_face_landmarks[0]

                nose = face_landmarks.landmark[1]
                nose_x = int(nose.x * width)
                nose_y = int(nose.y * height)

                gaze_state = classify_head_position(nose_x, width)

                cv2.circle(frame, (nose_x, nose_y), 5, (0, 255, 0), -1)

            cv2.putText(
                frame,
                gaze_state,
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            cv2.imshow("Webcam Gaze Test", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()