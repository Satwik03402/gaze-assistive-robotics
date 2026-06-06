#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from summer_robotics_interfaces.msg import DetectedObject, DetectedObjectArray
from cv_bridge import CvBridge


class ColorPerceptionNode(Node):

    def __init__(self):
        super().__init__("color_perception_node")

        self.bridge = CvBridge()
        self.frame_count = 0

        self.image_sub = self.create_subscription(
            Image,
            "/overhead_camera/overhead_camera/image_raw",
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            DetectedObjectArray,
            "/detected_objects",
            10
        )

        self.get_logger().info("Color Perception Node started.")

    def detect_color_object(self, hsv_image, color_name, lower, upper):
        mask = cv2.inRange(hsv_image, lower, upper)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < 50:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            cx = x + w // 2
            cy = y + h // 2

            detections.append(
                {
                    "color": color_name,
                    "center": (cx, cy),
                    "bbox": (x, y, w, h),
                    "area": area
                }
            )

        return detections

    def detect_red_object(self, hsv_image):
        red_lower_1 = np.array([0, 100, 50])
        red_upper_1 = np.array([10, 255, 255])

        red_lower_2 = np.array([170, 100, 50])
        red_upper_2 = np.array([180, 255, 255])

        detections_1 = self.detect_color_object(
            hsv_image,
            "red_cube",
            red_lower_1,
            red_upper_1
        )

        detections_2 = self.detect_color_object(
            hsv_image,
            "red_cube",
            red_lower_2,
            red_upper_2
        )

        return detections_1 + detections_2

    def detect_blue_object(self, hsv_image):
        blue_lower = np.array([100, 100, 50])
        blue_upper = np.array([140, 255, 255])

        return self.detect_color_object(
            hsv_image,
            "blue_cube",
            blue_lower,
            blue_upper
        )

    def create_detected_object(self, object_id, label, world_pose, bbox):
        obj = DetectedObject()

        obj.id = object_id
        obj.label = label

        obj.x = world_pose[0]
        obj.y = world_pose[1]
        obj.z = world_pose[2]

        obj.bbox = [
            float(bbox[0]),
            float(bbox[1]),
            float(bbox[2]),
            float(bbox[3])
        ]

        obj.pickable = True
        obj.status = "ACTIVE"

        return obj

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        hsv_image = cv2.cvtColor(
            cv_image,
            cv2.COLOR_BGR2HSV
        )

        red_detections = self.detect_red_object(hsv_image)
        blue_detections = self.detect_blue_object(hsv_image)

        all_detections = red_detections + blue_detections

        detected_msg = DetectedObjectArray()
        detected_msg.header.stamp = self.get_clock().now().to_msg()
        detected_msg.header.frame_id = "overhead_camera_frame"

        for detection in all_detections:
            label = detection["color"]
            bbox = detection["bbox"]

            if label == "red_cube":
                detected_object = self.create_detected_object(
                    1,
                    "red_cube",
                    [0.9, -0.25, 0.48],
                    bbox
                )

                detected_msg.objects.append(detected_object)

            elif label == "blue_cube":
                detected_object = self.create_detected_object(
                    2,
                    "blue_cube",
                    [0.9, 0.25, 0.48],
                    bbox
                )

                detected_msg.objects.append(detected_object)

        self.detection_pub.publish(detected_msg)
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            for detection in all_detections:
                color = detection["color"]
                center = detection["center"]
                bbox = detection["bbox"]
                area = detection["area"]

                self.get_logger().info(
                    f"Detected {color}: center={center}, bbox={bbox}, area={area:.1f}"
                )


def main(args=None):
    rclpy.init(args=args)

    node = ColorPerceptionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()