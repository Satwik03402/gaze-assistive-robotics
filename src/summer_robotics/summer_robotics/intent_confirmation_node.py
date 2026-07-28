#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from summer_robotics_interfaces.msg import VoiceCommand
from summer_robotics_interfaces.msg import ConfirmedIntent


class IntentConfirmationNode(Node):

    def __init__(self):
        super().__init__("intent_confirmation_node")

        self.selected_object_id = -1

        self.selected_object_sub = self.create_subscription(
            Int32,
            "/selected_object_id",
            self.selected_object_callback,
            10
        )

        self.voice_command_sub = self.create_subscription(
            VoiceCommand,
            "/voice_command",
            self.voice_command_callback,
            10
        )

        self.confirmed_intent_pub = self.create_publisher(
            ConfirmedIntent,
            "/confirmed_intent",
            10
        )

        self.get_logger().info("Intent Confirmation Node started.")

    def selected_object_callback(self, msg):
        self.selected_object_id = msg.data

        self.get_logger().info(
            f"Stored selected object ID: {self.selected_object_id}"
        )

    def voice_command_callback(self, msg):
        command = msg.command.strip().lower()

        if command == "cancel":
            self.publish_intent(
                object_id=-1,
                command="cancel"
            )

            self.selected_object_id = -1

            self.get_logger().info("Selection cancelled.")
            return

        if command in ["stop", "home"]:
            self.publish_intent(
                object_id=-1,
                command=command
            )
            return

        if command == "pick":
            if self.selected_object_id < 0:
                self.get_logger().warn(
                    "Pick command received, but no object is selected."
                )
                return

            self.publish_intent(
                object_id=self.selected_object_id,
                command=command
            )
            return

        self.get_logger().warn(
            f"Unknown voice command ignored: {command}"
        )

    def publish_intent(self, object_id, command):
        intent_msg = ConfirmedIntent()
        intent_msg.object_id = object_id
        intent_msg.command = command

        self.confirmed_intent_pub.publish(intent_msg)

        self.get_logger().info(
            f"Confirmed intent published: object_id={object_id}, command={command}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = IntentConfirmationNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()