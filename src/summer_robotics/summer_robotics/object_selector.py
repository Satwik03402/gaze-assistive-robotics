#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ObjectSelector(Node):

    def __init__(self):

        super().__init__("object_selector")

        self.publisher_ = self.create_publisher(
            Int32,
            "/selected_object_id",
            10
        )

        self.get_logger().info(
            "Object Selector Started."
        )

        self.get_logger().info(
            "Press 1 for red_cube"
        )

        self.get_logger().info(
            "Press 2 for blue_cube"
        )

        self.input_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )

        self.input_thread.start()

    def keyboard_loop(self):

        while rclpy.ok():

            user_input = input(
                "\nSelect Object (1=Red, 2=Blue): "
            )

            msg = Int32()

            if user_input == "1":

                msg.data = 1

            elif user_input == "2":

                msg.data = 2

            else:

                print("Invalid selection.")
                continue

            self.publisher_.publish(msg)

            self.get_logger().info(
                f"Selected: {msg.data}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = ObjectSelector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
