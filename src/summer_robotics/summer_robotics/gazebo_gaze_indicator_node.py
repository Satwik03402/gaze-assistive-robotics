#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from gazebo_msgs.srv import SpawnEntity, SetEntityState

from summer_robotics_interfaces.msg import DetectedObjectArray


class GazeboGazeIndicatorNode(Node):

    def __init__(self):
        super().__init__("gazebo_gaze_indicator_node")

        self.candidate_object_id = -1
        self.detected_objects = []

        self.spawn_client = self.create_client(
            SpawnEntity,
            "/spawn_entity"
        )

        self.set_state_client = self.create_client(
            SetEntityState,
            "/set_entity_state_custom"
        )

        self.candidate_sub = self.create_subscription(
            Int32,
            "/gaze_candidate_object_id",
            self.candidate_callback,
            10
        )

        self.objects_sub = self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self.objects_callback,
            10
        )

        self.get_logger().info(
            "Gazebo Gaze Indicator Node started."
        )

        self.spawn_timer = self.create_timer(
            1.0,
            self.try_spawn_indicator
        )

        self.indicator_spawned = False

    def candidate_callback(self, msg):
        self.candidate_object_id = msg.data
        self.update_indicator()

    def objects_callback(self, msg):
        self.detected_objects = msg.objects
        self.update_indicator()

    def try_spawn_indicator(self):
        if self.indicator_spawned:
            return

        if not self.spawn_client.service_is_ready():
            self.get_logger().info(
                "Waiting for /spawn_entity service..."
            )
            return

        self.get_logger().info(
            "/spawn_entity is ready. Spawning gaze indicator."
        )

        self.indicator_spawned = True
        self.spawn_indicator()

    def spawn_indicator(self):
        if not self.spawn_client.service_is_ready():
            self.get_logger().warn("/spawn_entity service not ready.")
            return

        request = SpawnEntity.Request()

        request.name = "gaze_indicator"

        request.xml = """
        <sdf version="1.6">
        <model name="gaze_indicator">
            <static>false</static>

            <link name="link">
            <gravity>false</gravity>

            <inertial>
                <mass>0.001</mass>
            </inertial>

            <visual name="visual">
                <geometry>
                <cylinder>
                    <radius>0.09</radius>
                    <length>0.015</length>
                </cylinder>
                </geometry>

                <material>
                <ambient>1 1 0 0.8</ambient>
                <diffuse>1 1 0 0.8</diffuse>
                </material>
            </visual>

            </link>
        </model>
        </sdf>
        """

        request.robot_namespace = ""

        # Start hidden below the scene
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = -2.0

        request.initial_pose.orientation.w = 1.0

        request.reference_frame = "world"

        future = self.spawn_client.call_async(request)

        future.add_done_callback(
            self.handle_spawn_response
        )

        self.get_logger().info(
            "Requested Gazebo gaze indicator spawn."
        )

    def handle_spawn_response(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    "Gazebo gaze indicator spawned successfully."
                )
            else:
                self.get_logger().warn(
                    f"Failed to spawn gaze indicator: "
                    f"{response.status_message}"
                )

        except Exception as error:
            self.get_logger().error(
                f"Gazebo gaze indicator spawn service failed: {error}"
            )

    def update_indicator(self):
        if not self.indicator_spawned:
            return

        if self.candidate_object_id < 0:
            self.move_indicator(0.0, 0.0, -2.0)
            return

        selected_object = None

        for obj in self.detected_objects:
            if obj.id == self.candidate_object_id:
                selected_object = obj
                break

        if selected_object is None:
            self.move_indicator(0.0, 0.0, -2.0)
            return

        self.move_indicator(
            selected_object.x,
            selected_object.y,
            selected_object.z + 0.08
        )

    def move_indicator(self, x, y, z):
        if not self.set_state_client.service_is_ready():
            self.get_logger().warn(
                "/set_entity_state_custom service not ready."
            )
            return

        request = SetEntityState.Request()

        request.state.name = "gaze_indicator"

        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z

        request.state.pose.orientation.x = 0.0
        request.state.pose.orientation.y = 0.0
        request.state.pose.orientation.z = 0.0
        request.state.pose.orientation.w = 1.0

        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0

        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0

        request.state.reference_frame = "world"

        self.set_state_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    node = GazeboGazeIndicatorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()