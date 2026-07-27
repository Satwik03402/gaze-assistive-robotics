#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "summer_robotics"

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "simple_robot.urdf"
    )

    with open(urdf_path, "r") as file:
        robot_description = file.read()

    world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "assistive_gaze_world.world"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description}
        ],
        output="screen"
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_path,
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen"
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "assistive_robot",
            "-x", "0",
            "-y", "0",
            "-z", "0"
        ],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller"
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_robot,

        TimerAction(
            period=5.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        TimerAction(
            period=7.0,
            actions=[arm_controller_spawner]
        )
    ])