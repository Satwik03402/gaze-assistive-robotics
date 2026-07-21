#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetEntityState
from visualization_msgs.msg import Marker
from summer_robotics_interfaces.msg import RobotStatus
from summer_robotics.robot_commands import PICK
from geometry_msgs.msg import Pose

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
        self.delete_client = self.create_client(
            DeleteEntity,
            "/delete_entity"
        )
        self.spawn_client = self.create_client(
            SpawnEntity,
            "/spawn_entity"
        )
        self.set_entity_state_client = self.create_client(
            SetEntityState,
            "/set_entity_state_custom"
        )
        self.object_name_map = {
                1: "red_cube",
                2: "blue_cube",
                3: "green_cube"
            }

        self.attached_object_id = None
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
            if self.attached_object_id is not None:
                object_name = self.object_name_map.get(self.attached_object_id)

                if object_name is not None:
                    self.move_gazebo_object(object_name, msg.pose)

                    self.get_logger().info(
                        f"Move request sent for Gazebo object: {object_name}"
                    )

                self.attached_object_id = None            

    def robot_status_callback(self, msg):
        if msg.current_command == PICK and msg.status == "DONE" and msg.success:
            self.attached_object_id = msg.object_id

            self.get_logger().info(
                f"Gazebo controller stored attached object ID: {self.attached_object_id}"
            )

    def create_cube_sdf(self, color_name):
        if color_name == "red_cube":
            color = "1 0 0 1"
        elif color_name == "blue_cube":
            color = "0 0 1 1"
        else:
            color = "0 1 0 1"

        return f"""
    <sdf version="1.6">
    <model name="{color_name}">
        <static>false</static>
        <link name="link">
        <inertial>
            <mass>0.2</mass>
        </inertial>

        <collision name="collision">
            <geometry>
            <box>
                <size>0.08 0.08 0.08</size>
            </box>
            </geometry>
        </collision>

        <visual name="visual">
            <geometry>
            <box>
                <size>0.08 0.08 0.08</size>
            </box>
            </geometry>
            <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            </material>
        </visual>
        </link>
    </model>
    </sdf>
    """

    def delete_gazebo_object(self, object_name):
        if not self.delete_client.service_is_ready():
            self.get_logger().warn("/delete_entity service not ready.")
            return

        request = DeleteEntity.Request()
        request.name = object_name

        future = self.delete_client.call_async(request)
        future.add_done_callback(
            lambda future_result: self.handle_delete_response(future_result, object_name)
        )

    def spawn_gazebo_object(self, object_name, pose):
        if not self.spawn_client.service_is_ready():
            self.get_logger().warn("/spawn_entity service not ready.")
            return

        request = SpawnEntity.Request()
        request.name = object_name
        request.xml = self.create_cube_sdf(object_name)
        request.robot_namespace = ""

        request.initial_pose.position.x = pose.position.x
        request.initial_pose.position.y = pose.position.y
        request.initial_pose.position.z = pose.position.z

        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 0.0
        request.initial_pose.orientation.w = 1.0

        request.reference_frame

        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda future_result: self.handle_spawn_response(future_result, object_name)
        )
        self.get_logger().info(f"Requested delete for Gazebo object: {object_name}")
        self.get_logger().info(
            f"Requested spawn for Gazebo object: {object_name} "
            f"at ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )

    def move_gazebo_object(self, object_name, pose):

        if not self.set_entity_state_client.service_is_ready():
            self.get_logger().warn(
                "/set_entity_state_custom service not ready."
            )
            return

        request = SetEntityState.Request()
        request.state.name = object_name
        request.state.pose = pose
        request.state.reference_frame = "world"

        future = self.set_entity_state_client.call_async(request)
        future.add_done_callback(
            lambda future_result: self.handle_move_response(
                future_result,
                object_name
            )
        )

        self.get_logger().info(
            f"Requested move of {object_name} to "
            f"({pose.position.x:.2f}, "
            f"{pose.position.y:.2f}, "
            f"{pose.position.z:.2f})"
        )

    def handle_spawn_response(self, future_result, object_name):
        try:
            response = future_result.result()

            if response.success:
                self.get_logger().info(
                    f"Spawn succeeded for Gazebo object: {object_name}"
                )
            else:
                self.get_logger().warn(
                    f"Spawn failed for Gazebo object: {object_name}. "
                    f"Reason: {response.status_message}"
                )

        except Exception as e:
            self.get_logger().error(
                f"Spawn service call failed for {object_name}: {e}"
            )
    
    def handle_delete_response(self, future_result, object_name):
        try:
            response = future_result.result()

            if response.success:
                self.get_logger().info(
                    f"Delete succeeded for Gazebo object: {object_name}"
                )
            else:
                self.get_logger().warn(
                    f"Delete failed for Gazebo object: {object_name}. "
                    f"Reason: {response.status_message}"
                )

        except Exception as e:
            self.get_logger().error(
                f"Delete service call failed for {object_name}: {e}"
            )

    def handle_move_response(self, future, object_name):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    f"Successfully moved Gazebo object: {object_name}"
                )
            else:
                self.get_logger().error(
                    f"Failed to move Gazebo object: {object_name}"
                )

        except Exception as error:
            self.get_logger().error(
                f"SetEntityState service call failed for "
                f"{object_name}: {error}"
            )

def main(args=None):
    rclpy.init(args=args)

    node = GazeboObjectControllerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
