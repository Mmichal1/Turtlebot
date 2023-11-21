from typing import List, Dict
import matplotlib.pyplot as plt
import rclpy
from matplotlib.pyplot import Axes
from shapely.geometry import LineString, Point

from enum import Enum, unique
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.duration import Duration
from mmrs_interfaces.srv import PathService
from mmrs_interfaces.msg import TriggerPose
from mmrs_interfaces.action import PathProcessing


@unique
class TriggerType(Enum):
    SIGNAL_TRIGGER = 0.6
    STOP_TRIGGER = 0.4


class PathProcessingActionServer(Node):
    def __init__(self):
        super().__init__("path_processing_action_server")
        self._action_server = ActionServer(
            self, PathProcessing, "path_processing", self.path_service_callback
        )
        self.robot_paths: Dict[str, Path] = {}
        self.trigger_poses: Dict[str, List[TriggerPose]] = {}
        self.restricted_segments: Dict[str, LineString] = {}
        self.goal_handles = {}

    def path_service_callback(self, goal_handle):
        robot_id = goal_handle.request.robot_id
        path = goal_handle.request.path

        self.get_logger().info(f"Received path from {robot_id}")

        self.robot_paths[robot_id] = self.convert_path_to_list_of_points(path)
        self.goal_handles[robot_id] = goal_handle

        # Check if paths from both robots are received
        if "robot1" in self.robot_paths and "robot2" in self.robot_paths:
            self.get_logger().info("Both paths received, processing...")

            self.process_paths()

            for robot_id, goal_handle in self.goal_handles.items():
                feedbeck = PathProcessing.Feedback()
                result = PathProcessing.Result()
                feedbeck.trigger_poses = self.trigger_poses[robot_id]
                result.succeeded = True
                goal_handle.publish_feedback(feedbeck)
                goal_handle.succeed()
                return result

            # Clear stored paths and goal handles after processing
            self.robot_paths.clear()
            self.goal_handles.clear()

        result = PathProcessing.Result()
        result.succeeded = False
        goal_handle.canceled()
        return result

    def process_paths(self):
        self.determine_restricted_segments(self.robot_paths, threshold=0.5)

        for robot_id in self.robot_paths:
            self.find_trigger_points(
                robot_id,
                self.robot_paths[robot_id],
                self.restricted_segments[robot_id],
                TriggerType.SIGNAL_TRIGGER,
            )
            self.find_trigger_points(
                robot_id,
                self.robot_paths[robot_id],
                self.restricted_segments[robot_id],
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
    ) -> Dict[str, LineString]:
        """
        Iterate over all paths and find points of interference between them.
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

            self.restricted_segments[robot_i] = (
                LineString(restricted_segment_points)
                if restricted_segment_points
                else LineString([])
            )

    def find_trigger_points(
        self,
        robot_id: str,
        robot_path: List[Point],
        restricted_segment: LineString,
        trigger_type: TriggerType,
    ) -> List[Point]:
        start_point = Point(restricted_segment.coords[0])
        end_point = Point(restricted_segment.coords[-1])

        trigger_points = []

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

        trigger_pose_start = TriggerPose
        trigger_pose_start.type = trigger_type.name
        trigger_pose_start.pose.position.x = closest_to_start.x
        trigger_pose_start.pose.position.y = closest_to_start.y

        trigger_pose_end = TriggerPose
        trigger_pose_end.type = trigger_type.name
        trigger_pose_end.pose.position.x = closest_to_end.x
        trigger_pose_end.pose.position.y = closest_to_end.y

        self.trigger_poses[robot_id].extend(
            [trigger_pose_start, trigger_pose_end]
        )

    def plot_data(self):
        wall_coords = [
            (2.3, 2.3),
            (2.3, -2.3),
            (-2.3, -2.3),
            (-2.3, 2.3),
            (2.3, 2.3),
        ]

        wall_x, wall_y = zip(*wall_coords)

        _, ax = plt.subplots(figsize=(8, 8))
        ax.plot(
            wall_x,
            wall_y,
            label="Wall (Room Boundary)",
            color="black",
            linewidth=5,
        )

        # self.plot_shapely_points(robot1_path, ax, "blue")
        # self.plot_shapely_points(robot2_path, ax, "green")
        # self.scatter_shapely_points(robot1_triggers_a, ax, "blue")
        # self.scatter_shapely_points(robot2_triggers_a, ax, "green")
        # self.scatter_shapely_points(robot1_triggers_b, ax, "blue")
        # self.scatter_shapely_points(robot2_triggers_b, ax, "green")
        # self.plot_shapely_linestring(path1_restricted_segment, ax, "red")
        # self.plot_shapely_linestring(path2_restricted_segment, ax, "red")

        # Adding details to the plot
        ax.set_title("Paths of Robot 1 and Robot 2")
        ax.set_xlabel("X Coordinate")
        ax.set_ylabel("Y Coordinate")
        ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.2))
        ax.grid(True)
        ax.set_xlim(-3.0, 3.0)
        ax.set_ylim(-3.0, 3.0)

        # Display the plot
        plt.show()

    def plot_shapely_points(
        self,
        points: List[Point],
        ax: Axes,
        color: str,
    ) -> None:
        x_coords = [point.x for point in points]
        y_coords = [point.y for point in points]

        ax.plot(x_coords, y_coords, color=color)

    def scatter_shapely_points(
        self,
        points: List[Point],
        ax: Axes,
        color: str,
    ) -> None:
        x_coords = [point.x for point in points]
        y_coords = [point.y for point in points]

        ax.scatter(x_coords, y_coords, color=color)

    def plot_shapely_linestring(
        self,
        linestring: LineString,
        ax: Axes,
        color: str,
    ) -> None:
        x, y = linestring.xy
        ax.plot(x, y, color=color, linewidth=5)  # Plotting each LineString


class PathProcessingActionClient(Node):
    def __init__(self, namespace: str):
        super().__init__(f"{namespace}")
        self.robot_id = namespace
        self._action_client = ActionClient(
            self, PathProcessing, "path_processing"
        )
        self.last_path = None
        self.last_robot_id = None

    def send_goal(self, robot_id: str, path: Path):
        self.last_path = path
        self.last_robot_id = robot_id

        goal_msg = PathProcessing.Goal()
        goal_msg.robot_id = robot_id
        goal_msg.path = path

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for the result")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        goal_status = future.result().status
        if goal_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            # Process successful result
            # ...
        elif goal_status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal aborted by the server")
        elif goal_status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal was canceled, resending the goal.")
            # Resend the goal
            self.send_goal(self.last_path, self.last_robot_id)
        # Handle the received result here

    def feedback_callback(self, feedback_msg):
        # Handle any feedback here, if provided by the server
        pass
