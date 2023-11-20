from typing import List, Tuple, Dict
import matplotlib.pyplot as plt
from matplotlib.pyplot import Axes
from shapely.geometry import LineString, Point

from nav_msgs.msg import Path
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from mmrs_interfaces.srv import PathService


class PathPublisher(Node):
    def __init__(
        self, namespace: str, node_name: str = "path_publisher"
    ) -> None:
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(
            Path, f"{namespace}/computed_path", 10
        )

    def publisher_callback(self, path: Path):
        msg = path
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing path.")


class PathSubcriber(Node):
    def __init__(
        self, namespace: str, node_name: str = "path_subscriber"
    ) -> None:
        super().__init__(node_name)
        self.subscription_ = self.create_subscription(
            Path, f"{namespace}/computed_path", self.listener_callback, 10
        )
        self.subscription_  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)


class PathCollisionServiceServer(Node):
    def __init__(self):
        super().__init__("path_collision_service_server")
        self.service = self.create_service(
            PathService, "path_collision_service", self.path_service_callback
        )
        self.robot_responses = {}

    def path_service_callback(self, request, response):
        # Assuming you identify clients based on the header frame_id of the Path message
        robot_id = request.path.header.frame_id
        self.get_logger().info(f"Received path from {robot_id}")

        # Store the path from each client
        self.robot_responses[robot_id] = request.path

        # Check if paths from both clients are received
        if (
            "robot1" in self.robot_responses
            and "robot2" in self.robot_responses
        ):
            self.process_paths()

        response.success = True
        return response

    def process_paths(self):
        # Convert paths to lists of (x, y) tuples
        robot1_path = self.convert_path_to_tuples(
            self.robot_responses["robot1"]
        )
        robot2_path = self.convert_path_to_tuples(
            self.robot_responses["robot2"]
        )

        wall_coords = [
            (2.3, 2.3),
            (2.3, -2.3),
            (-2.3, -2.3),
            (-2.3, 2.3),
            (2.3, 2.3),
        ]

        path1_restricted_segment, path2_restricted_segment = (
            self.determine_restricted_segments(
                robot1_path, robot2_path, threshold=0.5
            )
        )
        robot1_triggers = self.find_trigger_points(
            robot1_path, path1_restricted_segment, trigger_distance=0.5
        )  # Set trigger distance
        robot2_triggers = self.find_trigger_points(
            robot2_path, path2_restricted_segment, trigger_distance=0.5
        )  # Set trigger distance

        wall_x, wall_y = zip(*wall_coords)

        # Plotting
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(
            wall_x,
            wall_y,
            label="Wall (Room Boundary)",
            color="black",
            linewidth=5,
        )

        self.plot_shapely_points(robot1_path, ax)
        self.plot_shapely_points(robot2_path, ax)
        self.scatter_shapely_points(robot1_triggers, ax)
        self.scatter_shapely_points(robot2_triggers, ax)
        self.plot_shapely_linestring(path1_restricted_segment, ax)
        self.plot_shapely_linestring(path2_restricted_segment, ax)

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

    def convert_path_to_tuples(self, path_msg: Path) -> List[Point]:
        return [
            Point(pose.pose.position.x, pose.pose.position.y)
            for pose in path_msg.poses
        ]

    def determine_restricted_segments(
        self,
        path1: List[Point],
        path2: List[Point],
        threshold: float,
    ) -> Tuple[LineString]:
        path1_line_string = LineString(path1)
        path2_line_string = LineString(path2)
        path1_restricted_segment_points = []
        path2_restricted_segment_points = []

        for point in path1:
            if path2_line_string.distance(Point(point)) < threshold:
                path1_restricted_segment_points.append(point)

        for point in path2:
            if path1_line_string.distance(Point(point)) < threshold:
                path2_restricted_segment_points.append(point)

        path1_restricted_segment = LineString(path1_restricted_segment_points)
        path2_restricted_segment = LineString(path2_restricted_segment_points)

        return path1_restricted_segment, path2_restricted_segment

    def find_trigger_points(
        self,
        robot_path: List[Point],
        restricted_segment: LineString,
        trigger_distance: float,
    ) -> List[Point]:
        trigger_points = []

        points_not_on_line = [
            point
            for point in robot_path
            if not restricted_segment.intersects(point)
        ]

        # If there are no points off the LineString, return None
        if not points_not_on_line:
            return None

        # Calculate distances from each point to the LineString
        distances = [
            restricted_segment.distance(point) for point in points_not_on_line
        ]

        # Find the point with the distance closest to the predefined distance
        closest_distance = min(
            distances, key=lambda x: abs(x - trigger_distance)
        )
        trigger_points.append(
            points_not_on_line[distances.index(closest_distance)]
        )

        return trigger_points

    def plot_shapely_points(self, points: List[Point], ax: Axes) -> None:
        x_coords = [point.x for point in points]
        y_coords = [point.y for point in points]

        ax.plot(x_coords, y_coords)

    def scatter_shapely_points(self, points: List[Point], ax: Axes) -> None:
        x_coords = [point.x for point in points]
        y_coords = [point.y for point in points]

        ax.scatter(x_coords, y_coords)

    def plot_shapely_linestring(
        self, linestring: LineString, ax: Axes
    ) -> None:
        x, y = linestring.xy
        ax.plot(x, y)  # Plotting each LineString


class PathCollisionServiceClient(Node):
    def __init__(self, namespace: str):
        super().__init__(f"{namespace}")
        self.robot_id = namespace
        self.client = self.create_client(PathService, "path_collision_service")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        self.req = PathService.Request()

    def send_request(self, path_to_send):
        # Create and fill the Path message
        path = path_to_send
        path.header.frame_id = self.robot_id
        # Fill in the rest of the path message as needed
        self.req.path = path

        self.future = self.client.call_async(self.req)
