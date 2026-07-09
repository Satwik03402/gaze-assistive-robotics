#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from visualization_msgs.msg import Marker
from summer_robotics_interfaces.msg import RobotStatus
from summer_robotics.robot_commands import PICK

class GazeboObjectControllerNode(Node):

    def __init__(self):
        super().__init__("gazebo_object_controller_node")

        self.attached_pose_sub = self.create_subscription(
            Marker,
            "/visualization_marker",
            self.marker_callback,
            10
        )

        self.get_logger().info("Gazebo Object Controller Node started.")
        self.get_logger().info("Listening for attached/placed object markers.")
        self.current_attached_pose = None
        self.current_placed_pose = None
        self.is_attached = False
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            "/robot_status",
            self.robot_status_callback,
            10
        )

    def marker_callback(self, msg):

        if msg.ns == "attached_object":

            self.current_attached_pose = msg.pose
            self.is_attached = True

            self.get_logger().info(
                f"Attached object updated: "
                f"({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}, "
                f"{msg.pose.position.z:.2f})"
            )

        elif msg.ns == "placed_object":

            self.current_placed_pose = msg.pose
            self.is_attached = False

            self.get_logger().info(
                f"Placed object updated: "
                f"({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}, "
                f"{msg.pose.position.z:.2f})"
            )
            self.delete_client = self.create_client(
                DeleteEntity,
                "/delete_entity"
            )

            self.spawn_client = self.create_client(
                SpawnEntity,
                "/spawn_entity"
            )

            self.object_name_map = {
                1: "red_cube",
                2: "blue_cube",
                3: "green_cube"
            }

            self.attached_object_id = None

    def robot_status_callback(self, msg):
        if msg.current_command == PICK and msg.status == "DONE" and msg.success:
            self.attached_object_id = msg.object_id

            self.get_logger().info(
                f"Gazebo controller stored attached object ID: {self.attached_object_id}"
            )
            
def main(args=None):
    rclpy.init(args=args)

    node = GazeboObjectControllerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
