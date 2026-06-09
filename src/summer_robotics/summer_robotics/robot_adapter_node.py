#!/usr/bin/env python3

import rclpy
import math 
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
        self.motion_queue = []
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
        self.link_1_length = 1.0
        self.link_2_length = 0.8
        self.end_effector_length = 0.15
        self.base_height = 0.1

        self.joint_1_lower_limit = -1.57
        self.joint_1_upper_limit = 1.57

        self.joint_2_lower_limit = -2.5
        self.joint_2_upper_limit = 2.5

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

    def compute_ik(self, target_x, target_z):
        L1 = self.link_1_length
        L2 = self.link_2_length + self.end_effector_length

        x = target_x
        z = target_z - self.base_height

        distance_squared = x * x + z * z

        cos_theta2 = (
            distance_squared - L1 * L1 - L2 * L2
        ) / (2.0 * L1 * L2)

        if cos_theta2 < -1.0 or cos_theta2 > 1.0:
            self.get_logger().warn(
                f"Target unreachable: x={target_x:.3f}, z={target_z:.3f}, cos_theta2={cos_theta2:.3f}"
            )
            return None

        theta2 = math.atan2(
            math.sqrt(1.0 - cos_theta2 * cos_theta2),
            cos_theta2
        )

        theta1 = math.atan2(x, z) - math.atan2(
            L2 * math.sin(theta2),
            L1 + L2 * math.cos(theta2)
        )

        self.get_logger().info(
            f"IK result: theta1={theta1:.3f}, theta2={theta2:.3f}"
        )

        if not (
            self.joint_1_lower_limit <= theta1 <= self.joint_1_upper_limit
            and self.joint_2_lower_limit <= theta2 <= self.joint_2_upper_limit
        ):
            self.get_logger().warn(
                f"Joint limit violation: theta1={theta1:.3f}, theta2={theta2:.3f}"
            )
            return None

        return [theta1, theta2]

    def compute_fk(self, theta1, theta2):
        L1 = self.link_1_length
        L2 = self.link_2_length + self.end_effector_length

        x = (
            L1 * math.sin(theta1)
            + L2 * math.sin(theta1 + theta2)
        )

        z = (
            self.base_height
            + L1 * math.cos(theta1)
            + L2 * math.cos(theta1 + theta2)
        )

        return x, z

    def publish_joint_goal(self, positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["joint_0", "joint_1", "joint_2"]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2

        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

        self.get_logger().info(f"Published joint goal: {positions}")
        self.active_target = positions

    def start_motion_sequence(self, command, object_id, joint_goals):
        self.motion_queue = joint_goals

        self.active_command = command
        self.active_object_id = object_id

        if len(self.motion_queue) == 0:
            self.publish_status(
                "FAILED",
                command,
                object_id,
                False,
                "No motion goals available"
            )
            return

        next_goal = self.motion_queue.pop(0)
        self.publish_joint_goal(next_goal)

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
            self.publish_joint_goal([0.0, 0.0, 0.0])
            self.active_command = msg.command
            self.active_object_id = msg.object_id

        elif msg.command == MOVE_TO_PLACE:
            self.publish_joint_goal([0.0, 1.0, 0.0])
            self.active_command = msg.command
            self.active_object_id = msg.object_id

        elif msg.command == MOVE_TO_OBJECT:
            base_yaw = self.compute_base_yaw(msg.x, msg.y)

            planar_distance = math.sqrt(
                msg.x * msg.x
                + msg.y * msg.y
            )

            approach_x = planar_distance - 0.20
            approach_z = msg.z + 0.25

            descend_x = planar_distance - 0.05
            descend_z = msg.z + 0.08

            approach_arm_goal = self.compute_ik(approach_x, approach_z)
            descend_arm_goal = self.compute_ik(descend_x, descend_z)

            if approach_arm_goal is None or descend_arm_goal is None:
                self.get_logger().warn(
                    f"IK failed for object ID {msg.object_id}"
                )
                self.publish_status(
                    "FAILED",
                    msg.command,
                    msg.object_id,
                    False,
                    "IK failed for approach or descend target"
                )
                return

            approach_goal = [
                base_yaw,
                approach_arm_goal[0],
                approach_arm_goal[1]
            ]

            descend_goal = [
                base_yaw,
                descend_arm_goal[0],
                descend_arm_goal[1]
            ]

            self.get_logger().info(
                f"Base yaw for object ID {msg.object_id}: {base_yaw:.3f} rad"
            )

            for name, arm_goal, target_x, target_z in [
                ("approach", approach_arm_goal, approach_x, approach_z),
                ("descend", descend_arm_goal, descend_x, descend_z),
            ]:
                fk_x, fk_z = self.compute_fk(
                    arm_goal[0],
                    arm_goal[1]
                )

                fk_error = math.sqrt(
                    (fk_x - target_x) ** 2
                    + (fk_z - target_z) ** 2
                )

                self.get_logger().info(
                    f"{name.upper()} FK target=({target_x:.3f}, {target_z:.3f}) "
                    f"actual=({fk_x:.3f}, {fk_z:.3f}) "
                    f"error={fk_error:.3f} m"
                )

            self.start_motion_sequence(
                msg.command,
                msg.object_id,
                [
                    approach_goal,
                    descend_goal
                ]
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

        required_joints = ["joint_0", "joint_1", "joint_2"]

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
            if len(self.motion_queue) > 0:
                next_goal = self.motion_queue.pop(0)
                self.get_logger().info("Moving to next queued goal.")
                self.publish_joint_goal(next_goal)
                return

            self.publish_status(
                "DONE",
                self.active_command,
                self.active_object_id,
                True,
                "Motion sequence completed"
            )

            self.active_target = None
            self.active_command = None
            self.active_object_id = 0
            self.motion_queue = []
    def compute_base_yaw(self, x, y):
       return math.atan2(y, x)

def main(args=None):
    rclpy.init(args=args)

    node = RobotAdapterNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()