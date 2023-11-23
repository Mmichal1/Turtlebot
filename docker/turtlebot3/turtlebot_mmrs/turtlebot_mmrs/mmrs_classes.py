from typing import List, Dict, Tuple, Union
import matplotlib.pyplot as plt
import rclpy
import time
from matplotlib.pyplot import Axes
from shapely.geometry import LineString, Point
from itertools import combinations
from enum import Enum, unique
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from mmrs_interfaces.srv import PathService
from mmrs_interfaces.msg import TriggerPose

MAX_ATTEMPTS = 5


@unique
class TriggerType(Enum):
    SIGNAL_TRIGGER = 0.6
    STOP_TRIGGER = 0.4


class PathCollisionServiceServer(Node):
    def __init__(self):
        super().__init__("path_collision_service_server")
        self._define_service()
        self.robot_paths: Dict[str, Path] = {}
        self.trigger_poses: Dict[str, List[TriggerPose]] = {}
        self.restricted_segments_on_robots_path: Dict[
            str, List[LineString]
        ] = {}
        self.restricted_areas: Dict[int, LineString] = {}

    def _define_service(self):
        self.service = self.create_service(
            PathService, "path_collision_service", self.path_service_callback
        )

    def path_service_callback(self, request, response):
        """
        Process paths only when all robots have sent their path.
        If paths can't be processed then send empty list which
        is a signal for the client to wait a bit and then send the
        request again.
        """
        self.get_logger().info(f"Received path from {request.robot_id}")

        # Store the path from each client
        self.robot_paths[request.robot_id] = (
            self.convert_path_to_list_of_points(request.path)
        )
        # Initialize dictionaries
        self.trigger_poses[request.robot_id] = []
        self.restricted_segments_on_robots_path[request.robot_id] = []

        if "robot1" and "robot2" in self.robot_paths:
            self.process_paths()

        if request.robot_id in self.trigger_poses:
            response.trigger_poses = self.trigger_poses[request.robot_id]
        else:
            response.trigger_poses = []

        return response

    def process_paths(self):
        self.determine_restricted_segments(self.robot_paths, threshold=0.5)

        for robot_id in self.robot_paths:
            for restricted_segment in self.restricted_segments_on_robots_path[
                robot_id
            ]:
                self.find_trigger_points(
                    robot_id,
                    self.robot_paths[robot_id],
                    restricted_segment,
                    TriggerType.SIGNAL_TRIGGER,
                )
                self.find_trigger_points(
                    robot_id,
                    self.robot_paths[robot_id],
                    restricted_segment,
                    TriggerType.STOP_TRIGGER,
                )

    def convert_path_to_list_of_points(self, path_msg: Path) -> List[Point]:
        return [
            Point(pose.pose.position.x, pose.pose.position.y)
            for pose in path_msg.poses
        ]

    def determine_restricted_segments(
        self,
        paths_dict: Dict[str, List[Point]],
        threshold: float,
    ) -> Dict[str, List[LineString]]:
        """
        Iterate over all paths and find points of interference between them.
        If no collision segments found, then save empty LineString
        """

        line_strings = {
            robot_id: LineString(path) for robot_id, path in paths_dict.items()
        }

        for robot_i, path_i in paths_dict.items():
            restricted_segment_points = []

            for robot_j, _ in paths_dict.items():
                if robot_i != robot_j:
                    for point in path_i:
                        if (
                            line_strings[robot_j].distance(Point(point))
                            < threshold
                        ):
                            restricted_segment_points.append(point)

            self.restricted_segments_on_robots_path[robot_i].append(
                LineString(restricted_segment_points)
                if restricted_segment_points
                else LineString([])
            )

        unique_id = 0
        all_lines = [
            (robot_id, restricted_segment)
            for robot_id, lines in self.restricted_segments_on_robots_path.items()
            for restricted_segment in lines
        ]

        for (robot_id_1, linestring_1), (
            robot_id_2,
            linestring_2,
        ) in combinations(all_lines, 2):
            if (
                linestring_1.intersects(linestring_2)
                and robot_id_1 != robot_id_2
            ):
                restricted_area = linestring_1.union(linestring_2)
                self.restricted_areas[unique_id] = restricted_area
                unique_id += 1

    def find_trigger_points(
        self,
        robot_id: str,
        robot_path: List[Point],
        restricted_segment: LineString,
        trigger_type: TriggerType,
    ) -> List[Point]:
        """
        Find trigger points from each end of the restricted segment
        """

        if not restricted_segment:
            return

        start_point = Point(restricted_segment.coords[0])
        end_point = Point(restricted_segment.coords[-1])

        # Check only points that are not on the restricted segments
        points_not_on_line = [
            point
            for point in robot_path
            if not restricted_segment.intersects(point)
        ]

        closest_to_start = min(
            points_not_on_line,
            key=lambda point: abs(
                start_point.distance(point) - trigger_type.value
            ),
            default=None,
        )

        closest_to_end = min(
            points_not_on_line,
            key=lambda point: abs(
                end_point.distance(point) - trigger_type.value
            ),
            default=None,
        )

        trigger_pose_start = TriggerPose()
        trigger_pose_start.type = trigger_type.name
        trigger_pose_start.pose.position.x = closest_to_start.x
        trigger_pose_start.pose.position.y = closest_to_start.y

        trigger_pose_end = TriggerPose()
        trigger_pose_end.type = trigger_type.name
        trigger_pose_end.pose.position.x = closest_to_end.x
        trigger_pose_end.pose.position.y = closest_to_end.y

        self.trigger_poses[robot_id].extend(
            [trigger_pose_start, trigger_pose_end]
        )


class PathCollisionServiceClient(Node):
    def __init__(self, namespace: str, max_attempts: int = MAX_ATTEMPTS):
        super().__init__(f"{namespace}")
        self._define_clients()
        self.robot_id = namespace
        self.max_attempts = max_attempts
        self.request = PathService.Request()
        self.trigger_poses: List[TriggerPose] = []
        self.attempts = 0

    def _define_clients(self):
        self.client = self.create_client(PathService, "path_collision_service")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")

    def send_request(self, path_to_send):
        self.request.robot_id = self.robot_id
        self.request.path = path_to_send

        return self.client.call_async(self.request)

    def call_service_in_loop(self, path_to_send):
        while rclpy.ok() and self.attempts < self.max_attempts:
            self.attempts += 1
            future = self.send_request(path_to_send)
            rclpy.spin_until_future_complete(self, future)
            if future.done():
                response = future.result()
                if response.trigger_poses:
                    self.trigger_poses = response.trigger_poses
                    self.get_logger().info("Properly retrieved trigger poses.")
                    break
            else:
                self.get_logger().info("Retrying request.")
            time.sleep(1)
