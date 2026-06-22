#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from summer_robotics_interfaces.msg import RobotCommand, RobotStatus
from summer_robotics.robot_commands import (
    MOVE_TO_OBJECT,
    PICK,
    MOVE_TO_PLACE,
    PLACE,
    HOME
)


class RealRobotAdapterNode(Node):

    def __init__(self):
        super().__init__("real_robot_adapter_node")

        self.command_sub = self.create_subscription(
            RobotCommand,
            "/robot_command",
            self.robot_command_callback,
            10
        )

        self.status_pub = self.create_publisher(
            RobotStatus,
            "/robot_status",
            10
        )

        self.get_logger().info("Real Robot Adapter Skeleton started.")
        self.get_logger().info("This node logs real-robot actions but does not move hardware yet.")

    def publish_status(self, status, command, object_id, success, message):
        msg = RobotStatus()
        msg.status = status
        msg.current_command = command
        msg.object_id = object_id
        msg.success = success
        msg.message = message

        self.status_pub.publish(msg)

        self.get_logger().info(
            f"Robot status: {status} | {command} | {message}"
        )

    def robot_command_callback(self, msg):
        self.publish_status(
            "BUSY",
            msg.command,
            msg.object_id,
            False,
            "Real robot adapter received command"
        )

        if msg.command == MOVE_TO_OBJECT:
            self.get_logger().info(
                f"[REAL ROBOT TODO] Move to object {msg.object_id} "
                f"at pose x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
            )

        elif msg.command == PICK:
            self.get_logger().info(
                f"[REAL ROBOT TODO] Close gripper on object {msg.object_id}"
            )

        elif msg.command == MOVE_TO_PLACE:
            self.get_logger().info(
                f"[REAL ROBOT TODO] Move object {msg.object_id} "
                f"to place pose x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
            )

        elif msg.command == PLACE:
            self.get_logger().info(
                f"[REAL ROBOT TODO] Open gripper to place object {msg.object_id}"
            )

        elif msg.command == HOME:
            self.get_logger().info("[REAL ROBOT TODO] Move robot to home pose")

        else:
            self.publish_status(
                "FAILED",
                msg.command,
                msg.object_id,
                False,
                "Unknown real robot command"
            )
            return

        self.publish_status(
            "DONE",
            msg.command,
            msg.object_id,
            True,
            "Real robot skeleton command acknowledged"
        )


def main(args=None):
    rclpy.init(args=args)

    node = RealRobotAdapterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()