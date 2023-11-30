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

    navigator_robot_ = BasicNavigator(namespace="robot2")
    path_collision_service_client_robot_2 = PathCollisionServiceClient(
        namespace="robot2"
    )
    trigger_checker_robot_2 = TriggerChecker(
        namespace="robot2", navigator_node=navigator_robot_
    )
    executor_robot_2 = rclpy.executors.MultiThreadedExecutor()
    executor_robot_2.add_node(trigger_checker_robot_2)
    executor_thread_robot_2 = threading.Thread(
        target=executor_robot_2.spin, daemon=True
    )
    executor_thread_robot_2.start()

    security_route = [
        [
            1.8,
            1.8,
            0.9238795,
            -0.3826834,
        ],
        [
            -1.8,
            -1.8,
            0.3826834,
            0.9238795,
        ],
    ]

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator_robot_.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.8
    initial_pose.pose.position.y = 1.8
    initial_pose.pose.position.z = 0.01
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.9238795
    initial_pose.pose.orientation.w = -0.3826834
    navigator_robot_.setInitialPose(initial_pose)

    navigator_robot_.waitUntilNav2Active()

    route_poses = []
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator_robot_.get_clock().now().to_msg()
    for pt in security_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.z = pt[2]
        pose.pose.orientation.w = pt[3]
        route_poses.append(deepcopy(pose))

    path = navigator_robot_.getPathThroughPoses(initial_pose, route_poses)
    path_collision_service_client_robot_2.call_service_in_loop(path)
    trigger_checker_robot_2.set_triggers(
        path_collision_service_client_robot_2.get_triggers()
    )

    while rclpy.ok():
        navigator_robot_.followWaypoints(route_poses)

        i = 0
        while not navigator_robot_.isTaskComplete():
            i += 1
            feedback = navigator_robot_.getFeedback()
            if feedback and i % 5 == 0:
                pass
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
        pose.header.stamp = navigator_robot_.get_clock().now().to_msg()
        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.z = pt[2]
            pose.pose.orientation.w = pt[3]
            route_poses.append(deepcopy(pose))

        result = navigator_robot_.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Route complete! Restarting...")
        elif result == TaskResult.CANCELED:
            print("Security route was canceled, exiting.")
            exit(1)
        elif result == TaskResult.FAILED:
            print("Security route failed! Restarting from other side...")

    rclpy.shutdown()
    executor_thread_robot_2.join()
    exit(0)


if __name__ == "__main__":
    main()
