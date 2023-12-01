import rclpy
import time
from typing import List, Optional
from shapely.geometry import Point
from rclpy.node import Node
from rclpy.task import Future
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from mmrs_interfaces.msg import AreaMessage, TriggerPose
from mmrs_interfaces.srv import PathService
from turtlebot_mmrs.mmrs_classes import TriggerData, TriggerType

TRIGGER_DISTANCE_M = 0.2
"""Determines how close the robot needs to be to the trigger to activate it."""
MAX_ATTEMPTS = 10
"""Determines the max amount of attempts to send path processing request 
and get trigger data"""


class RobotController(Node):
    robot_id: str
    triggers: List[TriggerData]
    route_poses: List[PoseStamped]
    is_entry_allowed: bool
    is_stopping_required: bool
    current_trigger: Optional[TriggerData]
    attempts_to_get_triggers: int

    def __init__(self, namespace: str):
        super().__init__(f"{namespace}_controller")
        self.robot_id = namespace

        self._define_publishers()
        self._define_subscribers()
        self._define_clients()
        self._define_transition_events()
        self.reset_previous_trigger()

        self.triggers = []
        self.route_poses = []
        self.is_entry_allowed = False
        self.is_stopping_required = False
        self.current_trigger = None
        self.attempts_to_get_triggers = 0

    def _define_publishers(self) -> None:
        self.reserve_area_publisher = self.create_publisher(
            AreaMessage, "/restricted_area_control/reserve_area", 10
        )
        self.release_area_publisher = self.create_publisher(
            AreaMessage, "/restricted_area_control/release_area", 10
        )

    def _define_subscribers(self) -> None:
        self.entry_permission_subscriber = self.create_subscription(
            AreaMessage,
            "/restricted_area_control/entry_permission",
            self.entry_permission_callback,
            10,
        )
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{self.robot_id}/amcl_pose",
            self.activate_event_if_trigger_match,
            10,
        )

    def _define_clients(self) -> None:
        self.path_processing_service_client = self.create_client(
            PathService, "path_processing_service"
        )
        while not self.path_processing_service_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for service...")

    def _define_transition_events(self) -> None:
        """
        Based on the type of transition from trigger to trigger,
        3 different actions are defined:
            - SIGNAL -> SIGNAL, Area ID is not the same: Request access to an area,
            - SIGNAL -> STOP, Area ID is the same: Stop robot if access not granted,
            - STOP -> SIGNAL, Area ID is the same: Signal that an area is free
        """
        self.transition_events = {
            (
                TriggerType.SIGNAL_TRIGGER.name,
                TriggerType.SIGNAL_TRIGGER.name,
                False,
            ): self.event_reserve_area,
            (
                TriggerType.SIGNAL_TRIGGER.name,
                TriggerType.STOP_TRIGGER.name,
                True,
            ): self.event_stop_robot,
            (
                TriggerType.STOP_TRIGGER.name,
                TriggerType.SIGNAL_TRIGGER.name,
                True,
            ): self.event_release_area,
        }

    def reset_previous_trigger(self) -> None:
        """
        If robots turn around and follow the same path in reverse this can be used
        so that the robots can request an access to the same area as it did previously.
        """

        self.get_logger().info("Reseting previous trigger.")

        self.previous_trigger: TriggerData = TriggerData(
            position=Point(-100.0, -100.0),
            trigger_type=TriggerType.SIGNAL_TRIGGER.name,
            restricted_area_id=-1,
        )

    def entry_permission_callback(self, message: AreaMessage) -> None:
        """
        Check if the message is dedicated for this robot. If it is, then set the flags.
        """
        if (message.robot_id == self.robot_id) and (
            message.restricted_area_id
            == self.current_trigger.restricted_area_id
        ):
            self.get_logger().info(
                f"Entry into area {message.restricted_area_id} permitted."
            )
            self.is_entry_allowed = True
            self.is_stopping_required = False

    def event_release_area(self) -> None:
        """
        Send release area message. Reset entry permission flag.
        """
        self.get_logger().info(
            f"Releasing area {self.current_trigger.restricted_area_id}."
        )

        message = AreaMessage()
        message.robot_id = self.robot_id
        message.restricted_area_id = self.current_trigger.restricted_area_id

        self.is_entry_allowed = False

        self.release_area_publisher.publish(message)

    def event_stop_robot(self) -> None:
        """
        Check if entry is allowed. If it isn't, stop robot by cancelling the task
        """
        if not self.is_entry_allowed:
            self.get_logger().info(f"Stopping...")
            self.is_stopping_required = True

    def event_reserve_area(self) -> None:
        """
        Send reserve area message.
        """
        self.get_logger().info(
            "Trying to reserve area"
            f" {self.current_trigger.restricted_area_id}."
        )
        message = AreaMessage()
        message.robot_id = self.robot_id
        message.restricted_area_id = self.current_trigger.restricted_area_id

        self.reserve_area_publisher.publish(message)

    def send_request(self, path_to_send) -> Future:
        """
        Send path service request.
        """
        self.get_logger().info(f"Sending path service request.")
        request = PathService.Request()
        request.robot_id = self.robot_id
        request.path = path_to_send

        return self.path_processing_service_client.call_async(request)

    def attempt_to_get_trigger_data(self, path_to_send: Path) -> None:
        """
        Path service is called until trigger data is received or there were
        too many attempts in which case, the trigger data is not complete and
        robot should not proceed with navigation.
        """
        while rclpy.ok() and self.attempts_to_get_triggers < MAX_ATTEMPTS:
            self.attempts_to_get_triggers += 1
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
        """
        Process trigger data from the message to TriggerData class
        """
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

    def activate_event_if_trigger_match(
        self,
        message: PoseWithCovarianceStamped,
    ) -> None:
        """
        Check if the position matches the position of any trigger and activate
        that trigger if it does.
        """
        position = Point(
            (message.pose.pose.position.x, message.pose.pose.position.y)
        )

        self.current_trigger = self.find_matching_trigger(position)

        if (
            self.current_trigger
            and self.current_trigger != self.previous_trigger
        ):
            self.get_logger().info(
                f"Trigger reached, type: {self.current_trigger.trigger_type}"
            )
            transition = (
                self.previous_trigger.trigger_type,
                self.current_trigger.trigger_type,
                self.current_trigger.restricted_area_id
                == self.previous_trigger.restricted_area_id,
            )

            if transition in self.transition_events:
                self.get_logger().info(
                    f"Transition: {self.current_trigger.trigger_type}"
                )
                # Activate
                self.transition_events[transition]()

            self.previous_trigger = self.current_trigger

        elif (
            self.current_trigger
            and self.current_trigger == self.previous_trigger
        ):
            self.get_logger().info(
                f"Trigger reached, type: {self.current_trigger.trigger_type}"
            )
            transition = (
                self.previous_trigger.trigger_type,
                self.current_trigger.trigger_type,
                self.current_trigger.restricted_area_id
                == self.previous_trigger.restricted_area_id,
            )

            if transition in self.transition_events:
                self.get_logger().info(
                    f"Transition: {self.current_trigger.trigger_type}"
                )
                # Activate
                self.transition_events[transition]()

            self.previous_trigger = self.current_trigger

    def find_matching_trigger(
        self, current_position: Point
    ) -> Optional[TriggerData]:
        """
        Check if the position of any triggers corresponds to the input.
        Return trigger if it does.
        """
        for trigger in self.triggers:
            if (
                current_position.distance(trigger.position)
                <= TRIGGER_DISTANCE_M
            ):
                return trigger
        return None
