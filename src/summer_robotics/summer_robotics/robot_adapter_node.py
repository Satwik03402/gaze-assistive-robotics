#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from summer_robotics_interfaces.msg import RobotCommand

from summer_robotics.robot_commands import (
    MOVE_TO_OBJECT,
    PICK,
    MOVE_TO_PLACE,
    PLACE,
    HOME
)


class RobotAdapterNode(Node):

    def __init__(self):
        super().__init__("robot_adapter")

        self.command_subscriber = self.create_subscription(
            RobotCommand,
            "/robot_command",
            self.robot_command_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        self.get_logger().info("Robot Adapter started.")

    def publish_joint_goal(self, positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["joint_1", "joint_2"]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2

        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

        self.get_logger().info(f"Published joint goal: {positions}")

    def robot_command_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.command}")

        if msg.command == HOME:
            self.publish_joint_goal([0.0, 0.0])

        elif msg.command == MOVE_TO_PLACE:
            self.publish_joint_goal([1.0, 0.0])

        elif msg.command == MOVE_TO_OBJECT:
            if msg.object_id == 1:
                self.publish_joint_goal([0.7, -0.8])
            elif msg.object_id == 2:
                self.publish_joint_goal([0.7, 0.8])
            elif msg.object_id == 3:
                self.publish_joint_goal([0.7, 0.0])
            else:
                self.get_logger().warn(f"No joint mapping for object ID: {msg.object_id}")

        elif msg.command == PICK:
            self.get_logger().info("Pick action executed.")

        elif msg.command == PLACE:
            self.get_logger().info("Place action executed.")

        else:
            self.get_logger().warn(f"Unknown robot command: {msg.command}")


def main(args=None):
    rclpy.init(args=args)

    node = RobotAdapterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()