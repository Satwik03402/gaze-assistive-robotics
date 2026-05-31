#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmPositionCommander(Node):
    def __init__(self):
        super().__init__("arm_position_commander")

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        self.timer_ = self.create_timer(2.0, self.send_goal_once)
        self.sent_goal = False

        self.get_logger().info("Arm Position Commander started.")

    def send_goal_once(self):
        if self.sent_goal:
            return

        msg = JointTrajectory()
        msg.joint_names = ["joint_1", "joint_2"]

        point = JointTrajectoryPoint()
        point.positions = [0.7, -0.8]
        point.time_from_start.sec = 2

        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info("Sent arm goal: joint_1=0.7, joint_2=-0.8")

        self.sent_goal = True


def main(args=None):
    rclpy.init(args=args)

    node = ArmPositionCommander()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
