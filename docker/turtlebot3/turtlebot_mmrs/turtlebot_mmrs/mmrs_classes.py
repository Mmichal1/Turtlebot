from typing import List
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from mmrs_interfaces.srv import PathService

class PathPublisher(Node):

    def __init__(self, namespace: str, node_name: str = 'path_publisher') -> None:
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Path, f'{namespace}/computed_path', 10)

    def publisher_callback(self, path: Path):
        msg = path
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing path.')

class PathSubcriber(Node):

    def __init__(self, namespace: str, node_name: str = 'path_subscriber') -> None:
        super().__init__(node_name)
        self.subscription_ = self.create_subscription(Path, f'{namespace}/computed_path', self.listener_callback, 10)
        self.subscription_  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)

class PathCollisionServiceServer(Node):
    def __init__(self):
        super().__init__('path_collision_service_server')
        self.service = self.create_service(PathService, 'path_collision_service', self.path_service_callback)
        self.robot_responses = {}

    def path_service_callback(self, request, response):
        # Assuming you identify clients based on the header frame_id of the Path message
        robot_id = request.path.header.frame_id
        self.get_logger().info(f'Received path from {robot_id}')

        # Store the path from each client
        self.robot_responses[robot_id] = request.path

        # Check if paths from both clients are received
        if 'robot1' in self.robot_responses and 'robot2' in self.robot_responses:
            self.process_paths()

        response.success = True
        return response

    def process_paths(self):
        # Convert paths to lists of (x, y) tuples
        robot1_path = self.convert_path_to_tuples(self.robot_responses['robot1'])
        robot2_path = self.convert_path_to_tuples(self.robot_responses['robot2'])

        # You can now use robot1_path and robot2_path as needed
        self.get_logger().info(f'Robot 1 path: {robot1_path}')
        self.get_logger().info(f'Robot 2 path: {robot2_path}')

        # Unpack the points for plotting
        x1, y1 = zip(*robot1_path)
        x2, y2 = zip(*robot2_path)

        # Plotting
        plt.figure(figsize=(8, 6))
        plt.plot(x1, y1, label='Robot 1 Path', marker='o')
        plt.plot(x2, y2, label='Robot 2 Path', marker='x')

        # Adding details to the plot
        plt.title('Paths of Robot 1 and Robot 2')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.legend()
        plt.grid(True)

        # Display the plot
        plt.show()

    def convert_path_to_tuples(self, path_msg):
        # Extracts (x, y) tuples from the path
        return [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]

class PathCollisionServiceClient(Node):
    def __init__(self, namespace: str):
        super().__init__(f'{namespace}')
        self.robot_id = namespace
        self.client = self.create_client(PathService, 'path_collision_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = PathService.Request()

    def send_request(self, path_to_send):
        # Create and fill the Path message
        path = path_to_send
        path.header.frame_id = self.robot_id
        # Fill in the rest of the path message as needed
        self.req.path = path

        self.future = self.client.call_async(self.req)