import random

from more_msgs.msg import RangefinderStamped

import rclpy
from rclpy.node import Node


class Rangefinder(Node):
    def __init__(
        self,
    ):
        """
        A simple rangefinder, used for testing.
        """
        super().__init__("rangefinder")
        self.last_message = None
        self.update_rate = 10
        self.initial_value = 400
        self.rand_range = 100
        self.outlier_chance = 10
        self.timer = self.create_timer(1.0 / self.update_rate, self.update)
        self._rangefinder_publisher = self.create_publisher(RangefinderStamped, "rangefinder", 10)
        self.get_logger().info("rangefinder initialized.")

    def update(self):
        """
        Update the state machine and velocity based on the elapsed time.
        """
        # Placed under lock so update can be called by ESTOP callback if needed
        if self.last_message is None:
            distance = self.initial_value
        else:
            adjustment = random.randint(-self.rand_range / 2, self.rand_range / 2)
            distance = self.last_message.distance + adjustment

        # rebound if we hit negative
        if distance < 0:
            distance = abs(distance)

        # Construct and publish telemetry update
        data = RangefinderStamped()
        data.header.stamp = self.get_clock().now().to_msg()
        data.distance = float(distance)
        data.snr = float(0)
        self._rangefinder_publisher.publish(data)
        self.last_message = data
        self.get_logger().info(f"distance: {data.distance}")


def main(args=None):
    rclpy.init(args=args)
    node = Rangefinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl-C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
