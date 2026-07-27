#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from summer_robotics_interfaces.msg import VoiceCommand


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__("voice_command_node")

        self.voice_pub = self.create_publisher(
            VoiceCommand,
            "/voice_command",
            10
        )

        self.get_logger().info("Voice Command Node started.")
        self.get_logger().info("Type commands: pick, cancel, stop, home")

    def run_keyboard_loop(self):
        while rclpy.ok():
            command = input("Voice command> ").strip().lower()

            if command == "":
                continue

            msg = VoiceCommand()
            msg.command = command

            self.voice_pub.publish(msg)

            self.get_logger().info(f"Published voice command: {command}")


def main(args=None):
    rclpy.init(args=args)

    node = VoiceCommandNode()

    try:
        node.run_keyboard_loop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
