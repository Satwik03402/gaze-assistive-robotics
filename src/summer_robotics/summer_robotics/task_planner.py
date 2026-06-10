#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from summer_robotics_interfaces.srv import GetObjectById
from summer_robotics_interfaces.msg import RobotCommand
from summer_robotics_interfaces.msg import RobotStatus

from summer_robotics.robot_commands import (
    MOVE_TO_OBJECT,
    PICK,
    MOVE_TO_PLACE,
    PLACE,
    HOME
)


class TaskPlanner(Node):
    def __init__(self):
        super().__init__("task_planner")

        self.selected_object_id = None
        self.current_state = "IDLE"
        self.object_attached = False

        self.current_object_info = None

        self.goal_sent = False
        self.state_start_time = None
        self.simulated_action_duration = 1.0

        self.lookup_future = None
        self.lookup_in_progress = False
        self.lookup_retry_start_time = None
        self.lookup_timeout_sec = 5.0

        self.selected_object_sub = self.create_subscription(
            Int32,
            "/selected_object_id",
            self.selected_object_id_callback,
            10
        )

        self.object_lookup_client = self.create_client(
            GetObjectById,
            "/get_object_by_id"
        )

        self.state_timer = self.create_timer(0.2, self.run_state_machine)

        self.get_logger().info("Task Planner started.")
        self.get_logger().info(f"Current state: {self.current_state}")
        self.robot_command_pub = self.create_publisher(
            RobotCommand,
            "/robot_command",
            10
        )

        self.waiting_for_robot = False
        self.robot_command_done = False
        self.robot_command_failed = False
        self.expected_robot_command = None
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            "/robot_status",
            self.robot_status_callback,
            10
        )

    def robot_status_callback(self, msg):
        if self.expected_robot_command != msg.current_command:
            return

        if msg.status == "DONE" and msg.success:
            self.robot_command_done = True
            self.waiting_for_robot = False

        elif msg.status == "FAILED":
            self.robot_command_failed = True
            self.waiting_for_robot = False

    def selected_object_id_callback(self, msg):
        if self.current_state != "IDLE":
            self.get_logger().warn(
                f"Cannot start new task. Current state: {self.current_state}"
            )
            return

        self.selected_object_id = msg.data
        self.lookup_retry_start_time = self.get_clock().now()
        self.current_object_info = None
        self.lookup_future = None
        self.lookup_in_progress = False
        self.goal_sent = False

        self.current_state = "MOVING_TO_OBJECT"

        self.get_logger().info(f"Selected object ID stored: {self.selected_object_id}")
        self.get_logger().info(f"Auto-starting task for object ID: {self.selected_object_id}")

    def start_object_lookup(self):
        if self.selected_object_id is None:
            self.get_logger().warn("No selected object ID to look up.")
            return False

        if not self.object_lookup_client.service_is_ready():
            self.get_logger().warn("Object registry service not ready.")
            return False

        request = GetObjectById.Request()
        request.object_id = self.selected_object_id

        self.lookup_future = self.object_lookup_client.call_async(request)
        self.lookup_in_progress = True

        self.get_logger().info(f"Requested object lookup for ID: {self.selected_object_id}")
        return True

    def send_robot_command(self, command, object_info=None):
        msg = RobotCommand()
        msg.command = command

        if object_info is not None:
            msg.object_id = self.selected_object_id
            msg.x = object_info.pose[0]
            msg.y = object_info.pose[1]
            msg.z = object_info.pose[2]
        else:
            msg.object_id = 0
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0

        msg.qx = 0.0
        msg.qy = 0.0
        msg.qz = 0.0
        msg.qw = 1.0

        self.robot_command_pub.publish(msg)

        self.get_logger().info(f"Sent robot command: {command}")
        self.expected_robot_command = command
        self.waiting_for_robot = True
        self.robot_command_done = False
        self.robot_command_failed = False

    def reset_task(self):
        self.selected_object_id = None
        self.current_object_info = None
        self.object_attached = False
        self.goal_sent = False
        self.state_start_time = None
        self.lookup_future = None
        self.lookup_in_progress = False
        self.current_state = "IDLE"
        self.get_logger().info("State: IDLE")

    def cancel_task_due_to_object_loss(self):
        self.get_logger().warn("Selected object is no longer available. Cancelling task.")
        self.reset_task()

    def run_state_machine(self):
        if self.current_state == "IDLE":
            return

        if self.robot_command_failed:
            self.get_logger().warn("Robot command failed. Cancelling task.")
            self.reset_task()
            return

        if self.current_state == "MOVING_TO_OBJECT":
            if not self.goal_sent:
                if not self.lookup_in_progress and self.current_object_info is None:
                    started = self.start_object_lookup()
                    if not started:
                        self.get_logger().warn("Could not start object lookup. Returning to IDLE.")
                        self.reset_task()
                    return

                if self.lookup_in_progress:
                    if not self.lookup_future.done():
                        return

                    response = self.lookup_future.result()
                    self.lookup_future = None
                    self.lookup_in_progress = False

                    if not response.success:
                        elapsed = (
                            self.get_clock().now()
                            - self.lookup_retry_start_time
                        ).nanoseconds / 1e9

                        if elapsed < self.lookup_timeout_sec:
                            self.get_logger().info(
                                f"Object not found yet. Retrying... ({elapsed:.1f}s)"
                            )
                            self.start_object_lookup()
                            return

                        self.get_logger().warn(
                            "Object could not be discovered within timeout."
                        )
                        self.cancel_task_due_to_object_loss()
                        return

                    if not response.pickable:
                        self.get_logger().warn("Selected object is not pickable.")
                        self.cancel_task_due_to_object_loss()
                        return

                    if response.status != "ACTIVE":
                        self.get_logger().warn(
                            f"Selected object status is {response.status}. Cannot continue."
                        )
                        self.cancel_task_due_to_object_loss()
                        return

                    self.current_object_info = response

                    self.get_logger().info(
                        f"Object lookup success: ID={self.selected_object_id}, "
                        f"label={response.label}, pose={list(response.pose)}"
                    )

                self.send_robot_command(MOVE_TO_OBJECT, self.current_object_info)
                self.goal_sent = True
                self.get_logger().info("State: MOVING_TO_OBJECT")
                return

            if self.robot_command_done:
                self.get_logger().info("Reached selected object.")
                self.current_state = "PICK_OBJECT"
                self.goal_sent = False
                self.state_start_time = self.get_clock().now()
                self.get_logger().info("State: PICK_OBJECT")

        if self.current_state == "PICK_OBJECT":
            if not self.goal_sent:
                self.send_robot_command(PICK, self.current_object_info)
                self.goal_sent = True
                self.get_logger().info("State: PICK_OBJECT")
                return

            if self.robot_command_done:
                self.object_attached = True
                self.current_state = "MOVING_TO_PLACE_ZONE"
                self.goal_sent = False
                self.get_logger().info("Object picked.")
                self.get_logger().info("State: MOVING_TO_PLACE_ZONE")

        if self.current_state == "MOVING_TO_PLACE_ZONE":
            if not self.goal_sent:
                self.send_robot_command(MOVE_TO_PLACE)
                self.goal_sent = True
                self.get_logger().info("State: MOVING_TO_PLACE_ZONE")
                return

            if self.robot_command_done:
                self.get_logger().info("Reached place zone.")
                self.current_state = "DROP_OBJECT"
                self.goal_sent = False
                self.state_start_time = self.get_clock().now()
                self.get_logger().info("State: DROP_OBJECT")

        if self.current_state == "DROP_OBJECT":
            if not self.goal_sent:
                self.send_robot_command(PLACE)
                self.goal_sent = True
                self.get_logger().info("State: DROP_OBJECT")
                return

            if self.robot_command_done:
                self.object_attached = False
                self.current_state = "RETURNING_HOME"
                self.goal_sent = False
                self.get_logger().info("Object dropped.")
                self.get_logger().info("State: RETURNING_HOME")

        if self.current_state == "RETURNING_HOME":
            if not self.goal_sent:
                self.send_robot_command(HOME)
                self.goal_sent = True
                self.get_logger().info("State: RETURNING_HOME")
                return

            if self.robot_command_done:
                self.get_logger().info("Returned home.")
                self.current_state = "TASK_COMPLETE"
                self.goal_sent = False
                self.get_logger().info("State: TASK_COMPLETE")

        if self.current_state == "TASK_COMPLETE":
            self.get_logger().info(f"Task complete for: {self.selected_object_id}")
            self.reset_task()


def main(args=None):
    rclpy.init(args=args)

    node = TaskPlanner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()