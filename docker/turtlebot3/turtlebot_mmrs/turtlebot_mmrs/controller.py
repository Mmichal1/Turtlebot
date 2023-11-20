#! /usr/bin/env python3

from turtlebot_mmrs.mmrs_classes import PathCollisionServiceServer

import rclpy


def main(args=None):
    rclpy.init(args=args)
    my_service_server = PathCollisionServiceServer()
    rclpy.spin(my_service_server)
    my_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()