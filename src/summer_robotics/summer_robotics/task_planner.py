#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TaskPlanner(Node):
    def __init__(self):
        super().__init__("task_planner")
        # -------------------------
        # Internal state
        # -------------------------
        self.selected_object = None
        self.current_state = "IDLE"
        self.object_attached = False

        self.current_joint_positions = {}
        self.active_target = None
        self.state_timer = self.create_timer(
            0.2,
            self.run_state_machine
        )
        self.goal_sent = False
        self.position_tolerance = 0.03
        self.state_start_time = None
        self.simulated_action_duration = 1.0

        # -------------------------
        # Hardcoded joint poses
        # Version 1 only
        # -------------------------
        self.home_pose = [0.0, 0.0]
        self.red_cube_reach_pose = [0.7, -0.8]
        self.blue_cube_reach_pose = [0.7, 0.8]
        self.place_pose = [1.0, 0.0]

        self.get_logger().info("Task Planner started.")
        self.get_logger().info(f"Current state: {self.current_state}")
        
        self.selected_object_sub = self.create_subscription(
        	String,
        	"/selected_object",
        	self.selected_object_callback,
        	10
        )
        
        self.start_task_sub = self.create_subscription(
        	String,
        	"/start_task",
        	self.start_task_callback,
        	10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )
        
        self.trajectory_pub = self.create_publisher(
        	JointTrajectory,
        	"/arm_controller/joint_trajectory",
        	10
        )

    def start_task_callback(self, msg):
        if msg.data.lower() != "start":
            self.get_logger().warn(f"Unknown start command: {msg.data}")
            return

        if self.selected_object is None:
            self.get_logger().warn("Start requested, but no object selected yet.")
            return

        if self.current_state != "IDLE":
            self.get_logger().warn(f"Task already running. Current state: {self.current_state}")
            return

        self.current_state = "MOVING_TO_OBJECT"
        self.get_logger().info(f"Starting task for: {self.selected_object}")

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position
    
    def selected_object_callback(self, msg):
        if msg.data not in ["red_cube", "blue_cube"]:
            self.get_logger().warn(f"Unknown object selected: {msg.data}")
            return

        self.selected_object = msg.data
        self.get_logger().info(f"Selected object stored: {self.selected_object}")

        if self.current_state == "IDLE":
            self.current_state = "MOVING_TO_OBJECT"
            self.goal_sent = False
            self.get_logger().info(f"Auto-starting task for: {self.selected_object}")
        else:
            self.get_logger().warn(f"Cannot start new task. Current state: {self.current_state}")

    def send_joint_goal(self, joint_names, positions):

        traj_msg = JointTrajectory()

        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()

        point.positions = positions

        point.time_from_start.sec = 2

        traj_msg.points.append(point)

        self.trajectory_pub.publish(traj_msg)

        self.active_target = positions

        self.get_logger().info(
            f"Sent joint goal: {positions}"
        )

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

    def run_state_machine(self):
        if self.current_state == "IDLE":
            return

        if self.current_state == "MOVING_TO_OBJECT":
            if not self.goal_sent:
                if self.selected_object == "red_cube":
                    target = self.red_cube_reach_pose
                elif self.selected_object == "blue_cube":
                    target = self.blue_cube_reach_pose
                else:
                    self.get_logger().warn("No valid object selected.")
                    self.current_state = "IDLE"
                    return

                self.send_joint_goal(["joint_1", "joint_2"], target)
                self.goal_sent = True
                self.get_logger().info("State: MOVING_TO_OBJECT")
                return

            if self.has_reached_target():
                self.get_logger().info("Reached selected object.")
                self.current_state = "PICK_OBJECT"
                self.goal_sent = False
                self.state_start_time = self.get_clock().now()
                self.get_logger().info("State: PICK_OBJECT")
            
        if self.current_state == "PICK_OBJECT":
            elapsed_time = (
                self.get_clock().now() - self.state_start_time
            ).nanoseconds / 1e9

            if elapsed_time >= self.simulated_action_duration:
                self.object_attached = True
                self.current_state = "MOVING_TO_PLACE_ZONE"
                self.goal_sent = False
                self.get_logger().info("Object picked.")
                self.get_logger().info("State: MOVING_TO_PLACE_ZONE")

        if self.current_state == "MOVING_TO_PLACE_ZONE":
            if not self.goal_sent:
                self.send_joint_goal(["joint_1", "joint_2"], self.place_pose)
                self.goal_sent = True
                self.get_logger().info("State: MOVING_TO_PLACE_ZONE")
                return

            if self.has_reached_target():
                self.get_logger().info("Reached place zone.")
                self.current_state = "DROP_OBJECT"
                self.goal_sent = False
                self.state_start_time = self.get_clock().now()
                self.get_logger().info("State: DROP_OBJECT")
        
        if self.current_state == "DROP_OBJECT":
            elapsed_time = (
                self.get_clock().now() - self.state_start_time
            ).nanoseconds / 1e9

            if elapsed_time >= self.simulated_action_duration:
                self.object_attached = False
                self.current_state = "RETURNING_HOME"
                self.goal_sent = False
                self.get_logger().info("Object dropped.")
                self.get_logger().info("State: RETURNING_HOME")

        if self.current_state == "RETURNING_HOME":
            if not self.goal_sent:
                self.send_joint_goal(["joint_1", "joint_2"], self.home_pose)
                self.goal_sent = True
                self.get_logger().info("State: RETURNING_HOME")
                return

            if self.has_reached_target():
                self.get_logger().info("Returned home.")
                self.current_state = "TASK_COMPLETE"
                self.goal_sent = False
                self.get_logger().info("State: TASK_COMPLETE")     

        if self.current_state == "TASK_COMPLETE":
            self.get_logger().info(f"Task complete for: {self.selected_object}")

            self.selected_object = None
            self.active_target = None
            self.object_attached = False
            self.goal_sent = False
            self.state_start_time = None

            self.current_state = "IDLE"
            self.get_logger().info("State: IDLE")           
def main(args=None):
    rclpy.init(args=args)

    node = TaskPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
