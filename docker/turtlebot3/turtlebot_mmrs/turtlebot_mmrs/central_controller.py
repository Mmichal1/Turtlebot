from typing import List, Dict, Optional
from queue import Queue
from shapely.geometry import MultiLineString, LineString, Point
from itertools import combinations
from rclpy.node import Node
from nav_msgs.msg import Path
from mmrs_interfaces.srv import PathService
from mmrs_interfaces.msg import TriggerPose, AreaMessage
from turtlebot_mmrs.mmrs_classes import TriggerType

ROBOT_RADIUS_M = 0.55


class CentralController(Node):
    robot_paths: Dict[str, List[Point]]
    trigger_poses: Dict[str, List[TriggerPose]]
    restricted_segments_on_path: Dict[str, Dict[str, LineString]]
    restricted_areas_vis: Dict[int, MultiLineString]
    area_occupancy_map: Dict[int, Optional[str]]
    area_queue: Dict[int, Queue]

    def __init__(self):
        super().__init__("central_controller")
        self._define_publishers()
        self._define_subscribers()
        self._define_services()
        self.robot_paths = {}
        self.trigger_poses = {}
        self.restricted_segments_on_path = {}
        self.restricted_areas_vis = {}
        self.area_occupancy_map = {}
        self.area_queue = {}

    def _define_publishers(self) -> None:
        self.entry_permission_publisher = self.create_publisher(
            AreaMessage, "/restricted_area_control/entry_permission", 10
        )

    def _define_subscribers(self) -> None:
        self.reserve_area_subscriber = self.create_subscription(
            AreaMessage,
            "/restricted_area_control/reserve_area",
            self.reserve_area_callback,
            10,
        )
        self.release_area_subscriber = self.create_subscription(
            AreaMessage,
            "/restricted_area_control/release_area",
            self.release_area_callback,
            10,
        )

    def _define_services(self):
        self.path_processing_service = self.create_service(
            PathService,
            "path_processing_service",
            self.path_processing_service_callback,
        )

    def reserve_area_callback(self, message: AreaMessage):
        """
        Check if the are that the robot is trying to reserve is free.
        If it is, then assign that area to that robot and send the permission
        to enter that area. If it isn't, then put the robot in the queue.
        """
        robot_id = message.robot_id
        area_id = message.restricted_area_id

        if (
            not self.area_occupancy_map[area_id]
            or self.area_occupancy_map[area_id] == robot_id
        ):
            self.area_occupancy_map[area_id] = robot_id
            self.send_entry_permission(robot_id, area_id)
        else:
            self.area_queue[area_id].put(robot_id)

    def release_area_callback(self, message: AreaMessage):
        robot_id = message.robot_id
        area_id = message.restricted_area_id

        if self.area_occupancy_map[area_id] == robot_id:
            self.area_occupancy_map[area_id] = None

            if not self.area_queue[area_id].empty():
                next_robot_id_from_queue = self.area_queue[area_id].get()
                self.area_occupancy_map[area_id] = next_robot_id_from_queue
                self.send_entry_permission(next_robot_id_from_queue, area_id)

    def send_entry_permission(self, robot_id: str, restricted_area_id: int):
        message = AreaMessage()
        message.robot_id = robot_id
        message.restricted_area_id = restricted_area_id

        self.entry_permission_publisher.publish(message)

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
        else:
            response.trigger_poses = []

        return response

    def process_paths(self):
        self.determine_restricted_segments_and_areas(
            self.robot_paths, threshold=ROBOT_RADIUS_M
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

                self.area_queue[unique_id] = Queue()

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
