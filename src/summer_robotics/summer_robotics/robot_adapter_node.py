#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from summer_robotics_interfaces.msg import RobotCommand, RobotStatus
from sensor_msgs.msg import JointState
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

        self.status_pub = self.create_publisher(
            RobotStatus,
            "/robot_status",
            10
        )

        self.get_logger().info("Robot Adapter started.")
        self.current_joint_positions = {}
        self.active_target = None
        self.active_command = None
        self.active_object_id = 0
        self.position_tolerance = 0.03

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.monitor_timer = self.create_timer(
            0.1,
            self.monitor_motion
        )

    def publish_status(self, status, command, object_id, success, message):
        status_msg = RobotStatus()

        status_msg.status = status
        status_msg.current_command = command
        status_msg.object_id = object_id
        status_msg.success = success
        status_msg.message = message

        self.status_pub.publish(status_msg)

        self.get_logger().info(
            f"Robot status: {status} | {command} | {message}"
        )

    def publish_joint_goal(self, positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["joint_1", "joint_2"]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2

        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

        self.get_logger().info(f"Published joint goal: {positions}")
        self.active_target = positions

    def robot_command_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.command}")
        self.publish_status(
            "BUSY",
            msg.command,
            msg.object_id,
            False,
            "Executing command"
        )

        if msg.command == HOME:
            self.publish_joint_goal([0.0, 0.0])
            self.active_command = msg.command
            self.active_object_id = msg.object_id

        elif msg.command == MOVE_TO_PLACE:
            self.publish_joint_goal([1.0, 0.0])
            self.active_command = msg.command
            self.active_object_id = msg.object_id

        elif msg.command == MOVE_TO_OBJECT:
            if msg.object_id == 1:
                self.publish_joint_goal([0.7, -0.8])
                self.active_command = msg.command
                self.active_object_id = msg.object_id
            elif msg.object_id == 2:
                self.publish_joint_goal([0.7, 0.8])
                self.active_command = msg.command
                self.active_object_id = msg.object_id
                
            elif msg.object_id == 3:
                self.publish_joint_goal([0.7, 0.0])
                self.active_command = msg.command
                self.active_object_id = msg.object_id
            else:
                self.get_logger().warn(f"No joint mapping for object ID: {msg.object_id}")
                self.publish_status(
                    "FAILED",
                    msg.command,
                    msg.object_id,
                    False,
                    "No joint mapping for object ID"
                )

        elif msg.command == PICK:
            self.get_logger().info("Pick action executed.")
            self.publish_status("DONE", msg.command, msg.object_id, True, "Command completed")

        elif msg.command == PLACE:
            self.get_logger().info("Place action executed.")
            self.publish_status("DONE", msg.command, msg.object_id, True, "Command completed")

        else:
            self.get_logger().warn(f"Unknown robot command: {msg.command}")
            self.publish_status("FAILED", msg.command, msg.object_id, False, "Unknown or unsupported command")

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position


    def has_reached_target(self):
        if self.active_target is None:
            return False

        required_joints = ["joint_1", "joint_2"]

        for joint_name, target_position in zip(required_joints, self.active_target):
            if joint_name not in self.current_joint_positions:
                return False

            current_position = self.current_joint_positions[joint_name]
            error = abs(current_position - target_position)

            if error > self.position_tolerance:
                return False

        return True


    def monitor_motion(self):
        if self.active_target is None:
            return

        if self.has_reached_target():
            self.publish_status(
                "DONE",
                self.active_command,
                self.active_object_id,
                True,
                "Motion completed"
            )

            self.active_target = None
            self.active_command = None
            self.active_object_id = 0

def main(args=None):
    rclpy.init(args=args)

    node = RobotAdapterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()