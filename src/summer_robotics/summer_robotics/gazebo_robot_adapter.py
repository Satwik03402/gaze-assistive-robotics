#!/usr/bin/env python3

from summer_robotics.robot_adapter_base import RobotAdapterBase


class GazeboRobotAdapter(RobotAdapterBase):

    def move_to_pose(self, x, y, z):
        raise NotImplementedError("Gazebo move_to_pose is still handled by robot_adapter_node.")

    def pick(self, object_id):
        raise NotImplementedError("Gazebo pick is still handled by robot_adapter_node.")

    def place(self, object_id):
        raise NotImplementedError("Gazebo place is still handled by robot_adapter_node.")

    def go_home(self):
        raise NotImplementedError("Gazebo home is still handled by robot_adapter_node.")

    def publish_busy(self, command):
        raise NotImplementedError("Gazebo status publishing is still handled by robot_adapter_node.")

    def publish_done(self, command):
        raise NotImplementedError("Gazebo status publishing is still handled by robot_adapter_node.")