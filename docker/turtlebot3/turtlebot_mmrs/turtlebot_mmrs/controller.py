#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration


"""
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator(namespace="robot1")

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    security_route_robot_1 = [
        [-0.5, 1.5],
        [1.5, 1.5],
        [1.5, -0.5],
        [-0.5, -0.5],
    ]

    security_route_robot_2 = [
        [0.5, -1.5],
        [-1.5, -1.5],
        [-1.5, 0.5],
        [0.5, 0.5],
    ]

    # Set our demo's initial pose
    initial_pose_robot_1 = PoseStamped()
    initial_pose_robot_1.header.frame_id = 'map'
    initial_pose_robot_1.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose_robot_1.pose.position.x = -1.5
    initial_pose_robot_1.pose.position.y = 1.5
    initial_pose_robot_1.pose.position.z = 0.01
    initial_pose_robot_1.pose.orientation.x = 0.0
    initial_pose_robot_1.pose.orientation.y = 0.0
    initial_pose_robot_1.pose.orientation.z = 0.0
    initial_pose_robot_1.pose.orientation.w = 1.0

    route_poses_robot_1 = []
    pose_robot_1 = PoseStamped()
    pose_robot_1.header.frame_id = 'map'
    pose_robot_1.header.stamp = navigator.get_clock().now().to_msg()
    pose_robot_1.pose.orientation.w = 1.0
    for pt in security_route_robot_1:
        pose_robot_1.pose.position.x = pt[0]
        pose_robot_1.pose.position.y = pt[1]
        route_poses_robot_1.append(deepcopy(pose_robot_1))


    path = navigator.getPathThroughPoses(initial_pose_robot_1, route_poses_robot_1)
    print(path)

    # Set our demo's initial pose
    initial_pose_robot_2 = PoseStamped()
    initial_pose_robot_2.header.frame_id = 'map'
    initial_pose_robot_2.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose_robot_2.pose.position.x = 1.5
    initial_pose_robot_2.pose.position.y = -1.5
    initial_pose_robot_2.pose.position.z = 0.01
    initial_pose_robot_2.pose.orientation.x = 0.0
    initial_pose_robot_2.pose.orientation.y = 0.0
    initial_pose_robot_2.pose.orientation.z = 1.0
    initial_pose_robot_2.pose.orientation.w = 0.0


    route_poses_robot_2 = []
    pose_robot_2 = PoseStamped()
    pose_robot_2.header.frame_id = 'map'
    pose_robot_2.header.stamp = navigator.get_clock().now().to_msg()
    pose_robot_2.pose.orientation.w = 1.0
    for pt in security_route_robot_2:
        pose_robot_2.pose.position.x = pt[0]
        pose_robot_2.pose.position.y = pt[1]
        route_poses_robot_2.append(deepcopy(pose_robot_2))

   

    exit(0)


if __name__ == '__main__':
    main()
