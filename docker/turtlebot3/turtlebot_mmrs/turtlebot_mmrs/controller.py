#! /usr/bin/env python3

from turtlebot_mmrs.mmrs_classes import PathProcessingActionServer

import rclpy


def main(args=None):
    rclpy.init(args=args)
    server = PathProcessingActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
