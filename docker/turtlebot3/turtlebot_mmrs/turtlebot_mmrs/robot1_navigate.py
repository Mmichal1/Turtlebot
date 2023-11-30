#! /usr/bin/env python3

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot_mmrs.mmrs_classes import (
    PathCollisionServiceClient,
    TriggerChecker,
)

import rclpy
import threading


def main():
    rclpy.init()

    navigator_robot_1 = BasicNavigator(namespace="robot1")
    path_collision_service_client_robot_1 = PathCollisionServiceClient(
        namespace="robot1"
    )
    trigger_checker_robot_1 = TriggerChecker(
        namespace="robot1", navigator_node=navigator_robot_1
    )
    executor_robot_1 = rclpy.executors.MultiThreadedExecutor()
    executor_robot_1.add_node(trigger_checker_robot_1)
    executor_thread_robot_1 = threading.Thread(
        target=executor_robot_1.spin, daemon=True
    )
    executor_thread_robot_1.start()

    security_route = [
        [
            -1.8,
            1.8,
            0.3826834,
            -0.9238795,
        ],
        [
            1.8,
            -1.8,
            0.9238795,
            0.3826834,
        ],
    ]

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator_robot_1.get_clock().now().to_msg()
    initial_pose.pose.position.x = -1.8
    initial_pose.pose.position.y = 1.8
    initial_pose.pose.position.z = 0.01
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.3826834
    initial_pose.pose.orientation.w = -0.9238795
    navigator_robot_1.setInitialPose(initial_pose)

    navigator_robot_1.waitUntilNav2Active()

    route_poses = []
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator_robot_1.get_clock().now().to_msg()
    for pt in security_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.z = pt[2]
        pose.pose.orientation.w = pt[3]
        route_poses.append(deepcopy(pose))

    path = navigator_robot_1.getPathThroughPoses(initial_pose, route_poses)
    path_collision_service_client_robot_1.call_service_in_loop(path)
    trigger_checker_robot_1.set_triggers(
        path_collision_service_client_robot_1.get_triggers()
    )

    while rclpy.ok():
        navigator_robot_1.followWaypoints(route_poses)

        i = 0
        while not navigator_robot_1.isTaskComplete():
            i += 1
            feedback = navigator_robot_1.getFeedback()
            if feedback and i % 5 == 0:
                pass
                # print(feedback)

                # if Duration.from_msg(feedback.navigation_time) > Duration(
                #     seconds=180.0
                # ):
                #     print(
                #         "Navigation has exceeded timeout of 180s, canceling"
                #         " request."
                #     )
                #     navigator.cancelTask()

        security_route.reverse()

        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator_robot_1.get_clock().now().to_msg()
        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.z = pt[2]
            pose.pose.orientation.w = pt[3]
            route_poses.append(deepcopy(pose))

        result = navigator_robot_1.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Route complete! Restarting...")
        elif result == TaskResult.CANCELED:
            print("Security route was canceled, exiting.")
            exit(1)
        elif result == TaskResult.FAILED:
            print("Security route failed! Restarting from other side...")

    rclpy.shutdown()
    executor_thread_robot_1.join()
    exit(0)


if __name__ == "__main__":
    main()
