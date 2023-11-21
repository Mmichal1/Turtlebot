#! /usr/bin/env python3

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot_mmrs.mmrs_classes import PathProcessingActionClient

import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()

    navigator = BasicNavigator(namespace="robot2")
    path_processing_client = PathProcessingActionClient(namespace="robot2")

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    # security_route = [
    #     [0.5, -1.5],
    #     [-1.5, -1.5],
    #     [-1.5, 0.5],
    #     [0.5, 0.5],
    # ]

    security_route = [
        [0.0, -1.5],
        [0.0, 1.5],
    ]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = -1.5
    initial_pose.pose.position.z = 0.01
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    # Send our route
    route_poses = []
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.orientation.w = 1.0
    for pt in security_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        route_poses.append(deepcopy(pose))

    path = navigator.getPathThroughPoses(initial_pose, route_poses)
    path = navigator.getPathThroughPoses(initial_pose, route_poses)
    path_processing_client.send_goal("robot2", path)

    rclpy.spin(path_processing_client)

    # Do security route until dead
    while rclpy.ok():
        navigator.goThroughPoses(route_poses)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=180.0
                ):
                    print(
                        "Navigation has exceeded timeout of 180s, canceling"
                        " request."
                    )
                    navigator.cancelTask()

        # If at end of route, reverse the route to restart
        security_route.reverse()

        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Route complete! Restarting...")
        elif result == TaskResult.CANCELED:
            print("Security route was canceled, exiting.")
            exit(1)
        elif result == TaskResult.FAILED:
            print("Security route failed! Restarting from other side...")

    exit(0)


if __name__ == "__main__":
    main()
