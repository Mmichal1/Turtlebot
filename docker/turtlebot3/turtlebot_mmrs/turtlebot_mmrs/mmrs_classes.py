from typing import List
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
        self.client_responses = {}

    def path_service_callback(self, request, response):
        # Assuming you identify clients based on the header frame_id of the Path message
        robot_id = request.path.header.frame_id
        self.get_logger().info(f'Received path from {robot_id}')

        # Store the path from each client
        self.client_responses[robot_id] = request.path

        # Check if paths from both clients are received
        if 'robot1' in self.client_responses and 'robot2' in self.client_responses:
            self.process_paths()

        response.success = True
        return response

    def process_paths(self):
        # Implement your logic here, using the paths received from the clients
        self.get_logger().info('Received paths from both clients, processing...')

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