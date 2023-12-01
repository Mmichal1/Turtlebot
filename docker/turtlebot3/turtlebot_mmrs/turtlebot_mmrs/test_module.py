import rclpy
from rclpy.node import Node


class MyTimerNode(Node):
    def __init__(self):
        super().__init__("my_timer_node")
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.iterator = 0

    def timer_callback(self):
        now = self.get_clock().now()
        self.get_logger().info(f"Timer callback at {now.nanoseconds}")
        self.get_logger().info(f"Iterator: {self.iterator}")
        self.iterator += 1


def main(args=None):
    rclpy.init(args=args)
    timer_node = MyTimerNode()
    while rclpy.ok():
        rclpy.spin_once(timer_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
