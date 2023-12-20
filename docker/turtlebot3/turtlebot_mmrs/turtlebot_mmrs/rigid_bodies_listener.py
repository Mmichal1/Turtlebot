import rclpy
from rclpy.node import Node
from mocap_msgs.msg import RigidBodies
from geometry_msgs.msg import Pose


class RigidBodySubscriber(Node):
    def __init__(self):
        super().__init__("rigid_body_subscriber")
        self.subscription = self.create_subscription(
            RigidBodies, "/rigid_bodies", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for rigid_body in msg.rigidbodies:
            if rigid_body.rigid_body_name in ["16", "17"]:
                self.get_logger().info(
                    f"Rigid Body: {rigid_body.rigid_body_name}, Pose:"
                    f" {rigid_body.pose}"
                )


def main(args=None):
    rclpy.init(args=args)
    rigid_body_subscriber = RigidBodySubscriber()
    rclpy.spin(rigid_body_subscriber)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
