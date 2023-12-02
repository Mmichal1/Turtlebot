#! /usr/bin/env python3

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot_mmrs.robot_controller import RobotController


import rclpy


def main():
    rclpy.init()

    reverse_path = True
    last_waypoint = -1
    navigator = BasicNavigator(namespace="robot2")
    robot_controller = RobotController(namespace="robot2")

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
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.8
    initial_pose.pose.position.y = 1.8
    initial_pose.pose.position.z = 0.01
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.9238795
    initial_pose.pose.orientation.w = -0.3826834
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    route_poses = []
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    for pt in security_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.z = pt[2]
        pose.pose.orientation.w = pt[3]
        route_poses.append(deepcopy(pose))

    path = navigator.getPathThroughPoses(initial_pose, route_poses)
    robot_controller.attempt_to_get_trigger_data(path)

    if not robot_controller.triggers:
        rclpy.shutdown()
        exit(0)

    while rclpy.ok():
        if not robot_controller.is_stopping_required:
            navigator.followWaypoints(route_poses)

        rclpy.spin_once(robot_controller)

        while rclpy.ok():
            if navigator.isTaskComplete():
                # robot_controller.get_logger().info(
                #     "Task completed, reversing."
                # )
                # robot_controller.reset_previous_trigger()
                # reverse_path = not reverse_path
                break

            rclpy.spin_once(robot_controller)

            if robot_controller.is_stopping_required:
                navigator.cancelTask()

            feedback = navigator.getFeedback()
            last_waypoint = feedback.current_waypoint

        if reverse_path:
            security_route.reverse()

        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator.get_clock().now().to_msg()
        for pt in security_route[1:]:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.z = pt[2]
            pose.pose.orientation.w = pt[3]
            route_poses.append(deepcopy(pose))

        result = navigator.getResult()

        last_waypoint = 0

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
