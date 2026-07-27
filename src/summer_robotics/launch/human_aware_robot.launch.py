#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = "summer_robotics"

    gazebo_launch_path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        "gazebo_robot.launch.py"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    color_perception_node = Node(
        package=package_name,
        executable="color_perception_node",
        output="screen"
    )

    object_registry_node = Node(
        package=package_name,
        executable="object_registry_node",
        output="screen"
    )

    robot_adapter_node = Node(
        package=package_name,
        executable="robot_adapter_node",
        output="screen"
    )

    task_planner_node = Node(
        package=package_name,
        executable="task_planner",
        output="screen"
    )

    webcam_gaze_node = Node(
        package=package_name,
        executable="webcam_gaze_node",
        output="screen"
    )

    gaze_object_selector_node = Node(
        package=package_name,
        executable="gaze_object_selector_node",
        output="screen"
    )

    intent_confirmation_node = Node(
        package=package_name,
        executable="intent_confirmation_node",
        output="screen"
    )

    object_attachment_node = Node(
        package=package_name,
        executable="object_attachment_node",
        output="screen"
    )

    visualization_marker_node = Node(
        package=package_name,
        executable="visualization_marker_node",
        output="screen"
    )

    return LaunchDescription([
        gazebo_launch,

        TimerAction(period=4.0, actions=[color_perception_node]),
        TimerAction(period=5.0, actions=[object_registry_node]),
        TimerAction(period=6.0, actions=[robot_adapter_node]),
        TimerAction(period=7.0, actions=[task_planner_node]),

        TimerAction(period=8.0, actions=[webcam_gaze_node]),
        TimerAction(period=9.0, actions=[gaze_object_selector_node]),
        TimerAction(period=10.0, actions=[intent_confirmation_node]),
        TimerAction(period=11.0, actions=[object_attachment_node]),
        TimerAction(period=12.0, actions=[visualization_marker_node]),
    ])
