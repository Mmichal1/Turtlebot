#! /usr/bin/env python3

from turtlebot_mmrs.central_controller import CentralController

import rclpy


def main(args=None):
    rclpy.init(args=args)
    central_controller = CentralController()
    rclpy.spin(central_controller)
    central_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
