from typing import List, Dict, Optional
from dataclasses import dataclass
import matplotlib.pyplot as plt
import rclpy
import time
from matplotlib.pyplot import Axes
from shapely.geometry import MultiLineString, LineString, Point
from itertools import combinations
from enum import Enum, unique
from rclpy.node import Node
from rclpy.task import Future
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from mmrs_interfaces.srv import PathService, ReserveArea
from mmrs_interfaces.msg import TriggerPose, ReleaseArea

MAX_ATTEMPTS = 5
"""Determines the max amount of attempts to send path processing request 
and get trigger data"""
TRIGGER_DISTANCE_M = 1.0
"""Determines how close the robot needs to be to the trigger to activate it."""


@unique
class TriggerType(Enum):
    SIGNAL_TRIGGER = 0.6
    STOP_TRIGGER = 0.4


@dataclass
class TriggerData:
    position: Point
    trigger_type: TriggerType
    restricted_area_id: int


class PathCollisionServiceServer(Node):
    def __init__(self):
        super().__init__("path_processing_service_server")
        self._define_services()
        self.robot_paths: Dict[str, List[Point]] = {}
        self.trigger_poses: Dict[str, List[TriggerPose]] = {}
        self.restricted_segments_on_path: Dict[str, Dict[str, LineString]] = {}
        self.restricted_areas_vis: Dict[int, MultiLineString] = {}
        self.area_occupancy_map: Dict[int, Optional[str]] = {}

    def _define_services(self):
        self.path_processing_service = self.create_service(
            PathService,
            "path_processing_service",
            self.path_processing_service_callback,
        )
        self.reserve_area_service = self.create_service(
            ReserveArea,
            "reserve_area_service",
            self.reserve_area_service_callback,
        )

    def path_processing_service_callback(self, request, response):
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
        self.restricted_segments_on_path[request.robot_id] = {}

        if "robot1" and "robot2" in self.robot_paths:
            self.process_paths()

        if (
            request.robot_id in self.trigger_poses
            and self.trigger_poses[request.robot_id]
        ):
            response.trigger_poses = self.trigger_poses[request.robot_id]
            self.get_logger().info(f"{response.trigger_poses}")
            self.visualize_data()
        else:
            response.trigger_poses = []

        return response

    def process_paths(self):
        self.determine_restricted_segments_and_areas(
            self.robot_paths, threshold=0.5
        )

        for robot_id in self.robot_paths:
            for (
                restricted_segment_id,
                restricted_segment,
            ) in self.restricted_segments_on_path[robot_id].items():
                self.find_trigger_points(
                    robot_id,
                    self.robot_paths[robot_id],
                    restricted_segment,
                    restricted_segment_id,
                    TriggerType.SIGNAL_TRIGGER,
                )
                self.find_trigger_points(
                    robot_id,
                    self.robot_paths[robot_id],
                    restricted_segment,
                    restricted_segment_id,
                    TriggerType.STOP_TRIGGER,
                )

    def convert_path_to_list_of_points(self, path_msg: Path) -> List[Point]:
        return [
            Point(pose.pose.position.x, pose.pose.position.y)
            for pose in path_msg.poses
        ]

    def determine_restricted_segments_and_areas(
        self,
        paths_dict: Dict[str, List[Point]],
        threshold: float,
    ) -> None:
        """
        Iterate over all paths and find points of interference between them.
        If no collision segments found, then save empty LineString
        """
        restricted_segments: Dict[str, List[LineString]] = {}
        line_strings: Dict[str, LineString] = {}

        for robot_id, path in paths_dict.items():
            restricted_segments[robot_id] = []
            line_strings[robot_id] = LineString(path)

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

            restricted_segments[robot_i].append(
                LineString(restricted_segment_points)
                if restricted_segment_points
                else LineString([])
            )

        self.assign_ids_to_segments_and_areas(restricted_segments)

    def assign_ids_to_segments_and_areas(
        self, restricted_segments: Dict[str, List[LineString]]
    ) -> None:
        unique_id = 0
        all_line_strings = [
            (robot_id, restricted_segment)
            for robot_id, line_strings in restricted_segments.items()
            for restricted_segment in line_strings
        ]

        for (
            robot_i,
            line_string_i,
        ), (
            robot_j,
            line_string_j,
        ) in combinations(all_line_strings, 2):
            if line_string_i.intersects(line_string_j) and robot_i != robot_j:
                restricted_area = line_string_i.union(line_string_j)
                self.restricted_segments_on_path[robot_i][
                    unique_id
                ] = line_string_i

                self.restricted_segments_on_path[robot_j][
                    unique_id
                ] = line_string_j

                self.restricted_areas_vis[unique_id] = restricted_area

                self.area_occupancy_map[unique_id] = None

                unique_id += 1

    def find_trigger_points(
        self,
        robot_id: str,
        robot_path: List[Point],
        restricted_segment: LineString,
        restricted_segment_id: int,
        trigger_type: TriggerType,
    ) -> None:
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
        trigger_pose_start.restricted_area_id = restricted_segment_id
        trigger_pose_start.pose.position.x = closest_to_start.x
        trigger_pose_start.pose.position.y = closest_to_start.y

        trigger_pose_end = TriggerPose()
        trigger_pose_end.type = trigger_type.name
        trigger_pose_end.restricted_area_id = restricted_segment_id
        trigger_pose_end.pose.position.x = closest_to_end.x
        trigger_pose_end.pose.position.y = closest_to_end.y

        self.trigger_poses[robot_id].extend(
            [trigger_pose_start, trigger_pose_end]
        )

    def reserve_area_service_callback(self, request, response):
        """
        Check if reserved area is free (if it is then the value is None).
        If reservation is succesful, return true, otherwise return false.
        """
        if not self.area_occupancy_map[request.restricted_area_id]:
            self.area_occupancy_map[request.restricted_area_id] = (
                request.robot_id
            )
            response.is_reserved = True

            return response
        else:
            response.is_reserved = False

            return response

    def visualize_data(self):
        fig, ax = plt.subplots()

        for _, path in self.robot_paths.items():
            x, y = zip(*[(point.x, point.y) for point in path])
            ax.plot(x, y)

        for _, triggers in self.trigger_poses.items():
            for trigger in triggers:
                ax.plot(trigger.pose.position.x, trigger.pose.position.y, "o")
                ax.text(
                    trigger.pose.position.x,
                    trigger.pose.position.y,
                    f"({trigger.pose.position.x:.2f},"
                    f" {trigger.pose.position.y:.2f})",
                )

        for _, multiline in self.restricted_areas_vis.items():
            for line in multiline.geoms:
                x, y = line.xy
                ax.plot(x, y, color="red", linewidth=5)

        ax.set_xlabel("X Coordinate")
        ax.set_ylabel("Y Coordinate")

        plt.show()


class PathCollisionServiceClient(Node):
    def __init__(self, namespace: str, max_attempts: int = MAX_ATTEMPTS):
        super().__init__(f"{namespace}_path_processing_service_client")
        self._define_clients()
        self.robot_id = namespace
        self.max_attempts = max_attempts
        self.triggers: List[TriggerData] = []
        self.attempts = 0

    def _define_clients(self):
        self.path_processing_service_client = self.create_client(
            PathService, "path_processing_service"
        )
        while not self.path_processing_service_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for service...")

    def get_triggers(self):
        return self.triggers

    def send_request(self, path_to_send) -> Future:
        request = PathService.Request()
        request.robot_id = self.robot_id
        request.path = path_to_send

        return self.path_processing_service_client.call_async(request)

    def call_service_in_loop(self, path_to_send: Path) -> None:
        while rclpy.ok() and self.attempts < self.max_attempts:
            self.attempts += 1
            future = self.send_request(path_to_send)
            rclpy.spin_until_future_complete(self, future)
            if future.done():
                response = future.result()
                if response.trigger_poses:
                    self.triggers = self.process_trigger_data(
                        response.trigger_poses
                    )
                    self.get_logger().info("Properly retrieved trigger poses.")
                    self.get_logger().info(f"{self.triggers}")
                    break
            else:
                self.get_logger().info("Retrying request.")
            time.sleep(1)

    def process_trigger_data(
        self, trigger_poses: List[TriggerPose]
    ) -> List[TriggerData]:
        processed_triggers = []

        for trigger in trigger_poses:
            trigger_processed = TriggerData(
                position=Point(
                    (
                        trigger.pose.position.x,
                        trigger.pose.position.y,
                    )
                ),
                trigger_type=trigger.type,
                restricted_area_id=trigger.restricted_area_id,
            )
            processed_triggers.append(trigger_processed)

        return processed_triggers


class TriggerChecker(Node):
    def __init__(self, namespace: str):
        super().__init__(f"{namespace}_trigger_checker")
        self._define_publishers()
        self._define_subscribers(namespace)
        self._define_transition_actions()
        self.robot_id = namespace
        self.triggers: List[TriggerData] = []
        self.previous_reserved_area_id: Optional[TriggerType] = None
        self.previous_trigger: TriggerData = TriggerData(
            Point(0.0, 0.0), TriggerType.STOP_TRIGGER, -1
        )

    def _define_publishers(self):
        self.release_area_publisher = self.create_publisher(
            ReleaseArea, "/release_area", 10
        )

    def _define_subscribers(self, namespace: str):
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{namespace}/amcl_pose",
            self.pose_callback,
            10,
        )
        self.amcl_pose_subscriber

    def _define_clients(self):
        self.reserve_area_service_client = self.create_client(
            ReserveArea,
            "reserve_area_service",
        )
        while not self.reserve_area_service_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for service...")

    def _define_transition_actions(self):
        self.transition_actions = {
            (
                TriggerType.STOP_TRIGGER,
                TriggerType.SIGNAL_TRIGGER,
            ): self.action_stop_to_signal,
            (
                TriggerType.SIGNAL_TRIGGER,
                TriggerType.STOP_TRIGGER,
            ): self.action_signal_to_stop,
        }

    def action_stop_to_signal(self):
        pass

    def action_signal_to_stop(self):
        pass

    def send_reserve_area_request(
        self, robot_id: str, restricted_area_id: int
    ) -> Future:
        request = ReserveArea.Request()
        request.robot_id = robot_id
        request.restricted_area_id = restricted_area_id

        return self.reserve_area_service_client.call_async(request)

    def pose_callback(self, message: PoseWithCovarianceStamped):
        position = Point(
            (message.pose.pose.position.x, message.pose.pose.position.x)
        )
        self.get_logger().info(f"AMCL Position: {position}")
        current_trigger = self.is_trigger_reached(position)
        if current_trigger and current_trigger != self.previous_trigger:
            transition = (self.previous_trigger, current_trigger)
            if transition in self.transition_actions:
                self.transition_actions[
                    transition
                ]()  # Execute the action for the transition

            # Check if the previous reserved area is different then the area of a trigger now
            self.previous_trigger = current_trigger
            self.get_logger().info("TRIGGER REACHED.")

    def set_triggers(self, triggers: List[TriggerData]):
        self.get_logger().info(f"{triggers}")
        self.triggers = triggers

    def is_trigger_reached(
        self, current_position: Point
    ) -> Optional[TriggerData]:
        for trigger in self.triggers:
            if (
                current_position.distance(trigger.position)
                <= TRIGGER_DISTANCE_M
            ):
                return trigger
        return None

    def activate_trigger(self):
        pass
